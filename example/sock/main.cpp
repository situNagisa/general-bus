#include <ibus/sock.hpp>
#include <nagisa/concurrency/semaphore.h>

// sock 的各种 port；都是同步系统调用，communicate / communicate_range 返回 sender，这里用 sync_wait 取结果。
// port 不持有 fd：socket 由这里创建、配置、关闭。
// sock::port：
//   send_pack(span) / receive_pack(span)   1 段：send / recv
//   send_pack(file_region)                 文件区间：sendfile
//   writev_port / readv_port：communicate_range(iovec[])        动态 n 段合成 1 个单元
//   sendmmsg_port / recvmmsg_port：communicate_range(mmsghdr[])  动态 n 个数据报
//   sendmsg_port / recvmsg_port：sendmsg / recvmsg，send_pack 的 gso_size 参数走 UDP GSO，recv 交出 msg_flags 和 GRO 的 gso_size
//   send_pack(span, MSG_ZEROCOPY)：零拷贝发送；errqueue_port：取错误队列里的“可复用”通知。
//     buffer 由用户自己用 nagisa::concurrency::resource_pool 管理，序号由用户自己数。
//   port + pipe_region：splice（pipe ↔ socket）

#include <algorithm>
#include <array>
#include <cstdio>
#include <tuple>
#include <cstring>
#include <exception>
#include <map>
#include <vector>
#include <span>
#include <system_error>
#include <thread>

#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/udp.h>
#include <poll.h>
#include <netinet/tcp.h>
#include <stdlib.h>
#include <unistd.h>

using namespace ibus;
namespace nc = ::nagisa::concurrency;

// TCP 回环上的一对已连接 socket（零拷贝要求 TCP/UDP，AF_UNIX 不支持）。
static std::array<int, 2> tcp_pair() {
	int ls = ::socket(AF_INET, SOCK_STREAM | SOCK_CLOEXEC, 0);
	::sockaddr_in a{.sin_family = AF_INET, .sin_port = 0, .sin_addr = {htonl(INADDR_LOOPBACK)}, .sin_zero = {}};
	::socklen_t len = sizeof a;
	::bind(ls, reinterpret_cast<::sockaddr*>(&a), sizeof a);
	::listen(ls, 1);
	::getsockname(ls, reinterpret_cast<::sockaddr*>(&a), &len);
	int c = ::socket(AF_INET, SOCK_STREAM | SOCK_CLOEXEC, 0);
	::connect(c, reinterpret_cast<::sockaddr*>(&a), sizeof a);
	int s = ::accept4(ls, nullptr, nullptr, SOCK_CLOEXEC);
	::close(ls);
	return {c, s};
}

// sync_wait 取 sender 唯一的完成值（set_error(error_code) 时 sync_wait 抛 system_error）。
static auto value(auto&& s) { return std::get<0>(stdexec::sync_wait(std::forward<decltype(s)>(s)).value()); }

int main() {
	try {
		int sv[2];
		::socketpair(AF_UNIX, SOCK_STREAM | SOCK_CLOEXEC, 0, sv);
		sock::port a{sv[0]}, b{sv[1]};

		// 1 段
		char ping[] = "ping";
		std::array<std::byte, 4> rx{};
		std::printf("send   %zu\n", value(communicate(a, send_pack(a, std::as_bytes(std::span(ping, 4))))));
		std::printf("recv   %zu \"%.4s\"\n", value(communicate(b, receive_pack(b, std::span(rx)))), reinterpret_cast<char*>(rx.data()));

		// 文件区间：sendfile
		char path[] = "/tmp/ibus_sock_XXXXXX";
		int f = ::mkstemp(path);
		::write(f, "file-data", 9);
		::unlink(path);
		std::array<std::byte, 9> rf{};
		std::printf("sendfile %zu\n", value(communicate(a, send_pack(a, sock::file_region{f, 0, 9}))));
		std::printf("recv   %zu \"%.9s\"\n", value(communicate(b, receive_pack(b, std::span(rf)))), reinterpret_cast<char*>(rf.data()));
		::close(f);
		::close(sv[0]);
		::close(sv[1]);

		// 动态 n 段合成 1 个单元：帧头 + n 块载荷，一次 writev；接收端一次 readv 收进对应的几块
		{
			int sv2[2];
			::socketpair(AF_UNIX, SOCK_STREAM | SOCK_CLOEXEC, 0, sv2);
			sock::writev_port ia{sv2[0]};
			sock::readv_port ib{sv2[1]};
			char hdr[] = "HD", p0[] = "aaaa", p1[] = "bb";
			std::vector<::iovec> tx{
				send_pack(ia, std::as_bytes(std::span(hdr, 2))),
				send_pack(ia, std::as_bytes(std::span(p0, 4))),
				send_pack(ia, std::as_bytes(std::span(p1, 2))),
			};
			std::array<std::byte, 2> rh{};
			std::array<std::byte, 6> rp{};
			std::vector<::iovec> rx2{receive_pack(ib, std::span(rh)), receive_pack(ib, std::span(rp))};
			std::printf("writev %zu\n", value(communicate_range(ia, tx)));
			std::printf("readv  %zu \"%.2s\"+\"%.6s\"\n", value(communicate_range(ib, rx2)), reinterpret_cast<char*>(rh.data()), reinterpret_cast<char*>(rp.data()));
			::close(sv2[0]);
			::close(sv2[1]);
		}

		// 动态 n 个数据报：一次 sendmmsg 发 3 个，一次 recvmmsg 收，每个的长度写回 msg_len
		{
			int sv3[2];
			::socketpair(AF_UNIX, SOCK_DGRAM | SOCK_CLOEXEC, 0, sv3);
			sock::sendmmsg_port ma{sv3[0], 0};
			sock::recvmmsg_port mb{sv3[1], 0, nullptr};
			char d0[] = "one", d1[] = "three", d2[] = "x";
			std::array<::iovec, 3> tiov{{{d0, 3}, {d1, 5}, {d2, 1}}};
			std::vector<::mmsghdr> tx;
			for (auto& v : tiov) tx.push_back(send_pack(ma, std::span(&v, 1)));
			std::array<std::array<char, 8>, 3> rbuf{};
			std::array<::iovec, 3> riov{{{rbuf[0].data(), 8}, {rbuf[1].data(), 8}, {rbuf[2].data(), 8}}};
			std::vector<::mmsghdr> rx3;
			for (auto& v : riov) rx3.push_back(receive_pack(mb, std::span(&v, 1)));
			std::printf("sendmmsg %zu\n", value(communicate_range(ma, tx)));
			std::size_t k = value(communicate_range(mb, rx3));
			for (std::size_t i = 0; i < k; ++i) std::printf("recvmmsg #%zu %u \"%.*s\"\n", i, rx3[i].msg_len, static_cast<int>(rx3[i].msg_len), rbuf[i].data());
			::close(sv3[0]);
			::close(sv3[1]);
		}

		// MSG_PEEK：receive_pack 的 flags 参数，看一眼不取走
		{
			int sp[2];
			::socketpair(AF_UNIX, SOCK_STREAM | SOCK_CLOEXEC, 0, sp);
			sock::port pa{sp[0]}, pb{sp[1]};
			char m[] = "peek";
			std::array<std::byte, 4> r1{}, r2{};
			stdexec::sync_wait(communicate(pa, send_pack(pa, std::as_bytes(std::span(m, 4)))));
			stdexec::sync_wait(communicate(pb, receive_pack(pb, std::span(r1), MSG_PEEK)));
			stdexec::sync_wait(communicate(pb, receive_pack(pb, std::span(r2))));
			std::printf("peek \"%.4s\" then recv \"%.4s\"\n", reinterpret_cast<char*>(r1.data()), reinterpret_cast<char*>(r2.data()));
			::close(sp[0]);
			::close(sp[1]);
		}

		// splice：socket → pipe → socket，数据不经过用户内存
		{
			int sp[2], pp[2];
			::socketpair(AF_UNIX, SOCK_STREAM | SOCK_CLOEXEC, 0, sp);
			::pipe2(pp, O_CLOEXEC);
			sock::port sa{sp[0]}, sb{sp[1]};
			char m[] = "spliced";
			std::array<std::byte, 7> r{};
			stdexec::sync_wait(communicate(sa, send_pack(sa, std::as_bytes(std::span(m, 7)))));
			std::printf("splice in  %zu\n", value(communicate(sb, receive_pack(sb, sock::pipe_region{pp[1], 7}))));
			std::printf("splice out %zu\n", value(communicate(sb, send_pack(sb, sock::pipe_region{pp[0], 7}))));
			stdexec::sync_wait(communicate(sa, receive_pack(sa, std::span(r))));
			std::printf("recv \"%.7s\"\n", reinterpret_cast<char*>(r.data()));
			::close(sp[0]);
			::close(sp[1]);
			::close(pp[0]);
			::close(pp[1]);
		}

		// UDP GSO / GRO：发端一次 sendmsg 交 3500 字节、gso_size=1000，内核切成 4 个包；
		// 收端 fd 开 UDP_GRO，一次 recvmsg 收到合并的一块和 gso_size
		{
			int tx = ::socket(AF_INET, SOCK_DGRAM | SOCK_CLOEXEC, 0), rxs = ::socket(AF_INET, SOCK_DGRAM | SOCK_CLOEXEC, 0);
			::sockaddr_in a{.sin_family = AF_INET, .sin_port = 0, .sin_addr = {htonl(INADDR_LOOPBACK)}, .sin_zero = {}};
			::socklen_t len = sizeof a;
			::bind(rxs, reinterpret_cast<::sockaddr*>(&a), sizeof a);
			::getsockname(rxs, reinterpret_cast<::sockaddr*>(&a), &len);
			::connect(tx, reinterpret_cast<::sockaddr*>(&a), sizeof a);
			int on = 1;
			::setsockopt(rxs, IPPROTO_UDP, UDP_GRO, &on, sizeof on);
			sock::sendmsg_port gs{tx, 0, 0};
			sock::recvmsg_port gr{rxs, {}, 0};
			static std::array<std::byte, 3500> big{};
			static std::array<std::byte, 65536> rbuf{};
			alignas(::cmsghdr) std::array<std::byte, CMSG_SPACE(sizeof(int))> ctrl;
			std::printf("sendmsg gso %zu\n", value(communicate(gs, send_pack(gs, std::span<std::byte const>(big), 0, 1000))));
			std::size_t total = 0;
			while (total < big.size()) {
				auto r = value(communicate(gr, receive_pack(gr, std::span(rbuf), std::span(ctrl))));
				// cmsg 由用户自己解析：UDP_GRO 给出每个原始包的大小
				::msghdr m{};
				m.msg_control = ctrl.data();
				m.msg_controllen = r.control_size;
				int gro = 0;
				for (::cmsghdr* c = CMSG_FIRSTHDR(&m); c; c = CMSG_NXTHDR(&m, c))
					if (c->cmsg_level == IPPROTO_UDP && c->cmsg_type == UDP_GRO) std::memcpy(&gro, CMSG_DATA(c), sizeof gro);
				std::printf("recvmsg %zu bytes, gro_size=%d, flags=%#x\n", r.size, gro, r.flags);
				total += r.size;
			}
			::close(tx);
			::close(rxs);
		}

		// 零拷贝：port 不管发送内存。用户自己用 resource_pool 管两块 buffer：
		// 借一块 → send_pack 发出（完成值是序号）→ 记住“序号 → permit”；
		// 没有空闲块时，收一个“可复用”通知，把通知覆盖的序号对应的 permit 析构（还回池里）。
		// 回环上内核要等对端读走数据才发通知，所以接收端同时在另一个线程读。
		auto [c, s] = tcp_pair();
		int one = 1;
		::setsockopt(c, SOL_SOCKET, SO_ZEROCOPY, &one, sizeof one);
		// 零拷贝发送：普通 port，send_pack(..., MSG_ZEROCOPY)；通知从同一个 fd 的错误队列取（errqueue_port）。
		sock::port zc{c}, zs{s};
		sock::errqueue_port zq{c};
		std::jthread reader([&] {
			static std::array<std::byte, 64 * 1024> sink;
			std::size_t total = 0;
			while (total < 6 * 1000) total += value(communicate(zs, receive_pack(zs, std::span(sink))));
		});

		static std::array<std::array<std::byte, 64 * 1024>, 2> blocks;
		using pool_t = nc::resource_pool<std::span<std::byte>, std::vector<std::span<std::byte>>>;
		pool_t pool{std::vector<std::span<std::byte>>{blocks[0], blocks[1]}};
		std::map<std::uint32_t, pool_t::permit> inflight;  // 序号 → 还在内核手里的 buffer
		std::uint32_t seq = 0;  // 内核给每次成功的 MSG_ZEROCOPY send 依次编号，从 0 开始；由用户自己数

		for (int i = 0; i < 6; ++i) {
			auto permit = pool.try_acquire();
			while (!permit) {
				// 错误队列的 recvmsg 不阻塞；等通知由用户自己 poll(POLLERR)。
				::pollfd pf{.fd = c, .events = 0, .revents = 0};
				::poll(&pf, 1, -1);
				alignas(::cmsghdr) static std::array<std::byte, CMSG_SPACE(sizeof(::sock_extended_err))> ctrl;
				auto const& e = *value(communicate(zq, receive_pack(zq, std::span(ctrl))));
				if (e.ee_origin != SO_EE_ORIGIN_ZEROCOPY) throw std::system_error(static_cast<int>(e.ee_errno), std::generic_category(), "errqueue");
				std::printf("  notification [%u, %u] copied=%d\n", e.ee_info, e.ee_data, (e.ee_code & SO_EE_CODE_ZEROCOPY_COPIED) != 0);
				inflight.erase(inflight.lower_bound(e.ee_info), inflight.upper_bound(e.ee_data));  // permit 析构 = 还回池里
				permit = pool.try_acquire();
			}
			auto buf = permit->token().first(1000);
			std::ranges::fill(buf, std::byte('a' + i));
			stdexec::sync_wait(communicate(zc, send_pack(zc, buf, MSG_ZEROCOPY)));
			std::printf("zerocopy send #%d seq=%u\n", i, seq);
			inflight.emplace(seq++, std::move(*permit));
		}
		reader.join();
		::close(c);
		::close(s);
	} catch (std::exception const& e) {
		std::printf("error: %s\n", e.what());
	}
}
