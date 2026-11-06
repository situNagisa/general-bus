#pragma once

#include <ibus/cpo.hpp>
#include <ibus/sender.hpp>

#include <cerrno>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <ranges>
#include <span>
#include <type_traits>

#include <fcntl.h>
#include <linux/errqueue.h>
#include <netinet/in.h>
#include <netinet/udp.h>
#include <sys/sendfile.h>
#include <sys/socket.h>
#include <sys/uio.h>

// 所有 port 都不持有 fd：socket 由用户创建、连接、配置、关闭，port 只记下 fd。
// 都是同步系统调用（阻不阻塞看 fd 自己）：communicate / communicate_range 返回 sender，start 时做一次系统调用，
// 完成值走 set_value，失败走 set_error(errno)（errno）。
// 每次调用的 flags 等是 send_pack / receive_pack 的参数；communicate / communicate_range 不接受额外参数，
// 动态 n 段要用的 flags 等是 port 的成员，由用户设置。
// 下面先是 concept（各种 port 的结构），再是满足它们的便利 port。
namespace ibus::sock {

// 文件区间：sendfile 的数据来源，不经过用户内存。
struct file_region {
	int fd;
	::off_t offset;
	std::size_t count;
};

// pipe 区间：splice 的一端（另一端是 socket），数据不经过用户内存。
struct pipe_region {
	int fd;
	std::size_t count;
};

// recvmsg 的完成值。
struct recvmsg_result {
	std::size_t size;          // 收到的字节数
	int flags;                 // msg_flags：MSG_TRUNC、MSG_CTRUNC 等
	std::size_t control_size;  // 控制缓冲里实际写了多少
};

// ---- concept：只规定 send_pack / receive_pack 要接受的参数，以及 communicate / communicate_range 支持的组合；
// 返回的 sender 要能和 R（默认是上面的 receiver）连接，完成值的具体类型不做规定。
// pack 的类型由 port 自己定。下面的便利 port 各满足其中一个或几个；用户可以按同样的结构写自己的 port（比如把 flags 写死）。

// send / recv：send_pack(data, flags)、receive_pack(data, flags)，完成值是字节数。
// stream 的参考 receiver：正好接得住 stream 里各组合发出的完成；用户的 receiver 照这个写，作为 stream 的第二个参数检查。
struct stream_receiver {
	using receiver_concept = stdexec::receiver_t;
	void set_value(std::size_t) && noexcept {}
	void set_error(int) && noexcept {}
};
template <class P, class R = stream_receiver>
concept stream = requires(P& p, std::span<std::byte const> out, std::span<std::byte> in, int flags) {
	requires stdexec::sender_to<decltype(communicate(p, send_pack(p, out, flags))), R>;
	requires stdexec::sender_to<decltype(communicate(p, receive_pack(p, in, flags))), R>;
};

// sendfile：send_pack(file_region)，完成值是字节数。
// file_send 的参考 receiver：正好接得住 file_send 里各组合发出的完成；用户的 receiver 照这个写，作为 file_send 的第二个参数检查。
struct file_send_receiver {
	using receiver_concept = stdexec::receiver_t;
	void set_value(std::size_t) && noexcept {}
	void set_error(int) && noexcept {}
};
template <class P, class R = file_send_receiver>
concept file_send = requires(P& p, file_region r) {
	requires stdexec::sender_to<decltype(communicate(p, send_pack(p, r))), R>;
};

// splice：send_pack(pipe_region, flags) 是 pipe → socket，receive_pack(pipe_region, flags) 是 socket → pipe，完成值是字节数。
// splicing 的参考 receiver：正好接得住 splicing 里各组合发出的完成；用户的 receiver 照这个写，作为 splicing 的第二个参数检查。
struct splicing_receiver {
	using receiver_concept = stdexec::receiver_t;
	void set_value(std::size_t) && noexcept {}
	void set_error(int) && noexcept {}
};
template <class P, class R = splicing_receiver>
concept splicing = requires(P& p, pipe_region r, unsigned flags) {
	requires stdexec::sender_to<decltype(communicate(p, send_pack(p, r, flags))), R>;
	requires stdexec::sender_to<decltype(communicate(p, receive_pack(p, r, flags))), R>;
};

// 错误队列：receive_pack(控制缓冲)，完成值是指向控制缓冲里 sock_extended_err 的指针。
// error_queue 的参考 receiver：正好接得住 error_queue 里各组合发出的完成；用户的 receiver 照这个写，作为 error_queue 的第二个参数检查。
struct error_queue_receiver {
	using receiver_concept = stdexec::receiver_t;
	void set_value(::sock_extended_err const*) && noexcept {}
	void set_error(int) && noexcept {}
};
template <class P, class R = error_queue_receiver>
concept error_queue = requires(P& p, std::span<std::byte> control) {
	requires stdexec::sender_to<decltype(communicate(p, receive_pack(p, control))), R>;
};

// writev：send_pack(data) 返回 iovec，communicate_range(iovec 的连续区间)，完成值是总字节数。
// gather_write 的参考 receiver：正好接得住 gather_write 里各组合发出的完成；用户的 receiver 照这个写，作为 gather_write 的第二个参数检查。
struct gather_write_receiver {
	using receiver_concept = stdexec::receiver_t;
	void set_value(std::size_t) && noexcept {}
	void set_error(int) && noexcept {}
};
template <class P, class R = gather_write_receiver>
concept gather_write = requires(P& p, std::span<std::byte const> out, std::span<::iovec> iovs) {
	{ send_pack(p, out) } -> std::same_as<::iovec>;
	requires stdexec::sender_to<decltype(communicate_range(p, iovs)), R>;
};

// readv：receive_pack(data) 返回 iovec，communicate_range(iovec 的连续区间)，完成值是总字节数。
// scatter_read 的参考 receiver：正好接得住 scatter_read 里各组合发出的完成；用户的 receiver 照这个写，作为 scatter_read 的第二个参数检查。
struct scatter_read_receiver {
	using receiver_concept = stdexec::receiver_t;
	void set_value(std::size_t) && noexcept {}
	void set_error(int) && noexcept {}
};
template <class P, class R = scatter_read_receiver>
concept scatter_read = requires(P& p, std::span<std::byte> in, std::span<::iovec> iovs) {
	{ receive_pack(p, in) } -> std::same_as<::iovec>;
	requires stdexec::sender_to<decltype(communicate_range(p, iovs)), R>;
};

// sendmsg：send_pack(data, flags, gso_size) 1 段；communicate_range(iovecs) 动态 n 段（flags、gso_size 由 port 自己定）。完成值是字节数。
// message_send 的参考 receiver：正好接得住 message_send 里各组合发出的完成；用户的 receiver 照这个写，作为 message_send 的第二个参数检查。
struct message_send_receiver {
	using receiver_concept = stdexec::receiver_t;
	void set_value(std::size_t) && noexcept {}
	void set_error(int) && noexcept {}
};
template <class P, class R = message_send_receiver>
concept message_send = requires(P& p, std::span<std::byte const> out, std::span<::iovec> iovs, int flags, std::uint16_t gso_size) {
	requires stdexec::sender_to<decltype(communicate(p, send_pack(p, out, flags, gso_size))), R>;
	requires stdexec::sender_to<decltype(communicate_range(p, iovs)), R>;
};

// recvmsg：receive_pack(data, control, flags) 1 段；communicate_range(iovecs) 动态 n 段（控制缓冲、flags 由 port 自己定）。
// 完成值是 recvmsg_result。
// message_receive 的参考 receiver：正好接得住 message_receive 里各组合发出的完成；用户的 receiver 照这个写，作为 message_receive 的第二个参数检查。
struct message_receive_receiver {
	using receiver_concept = stdexec::receiver_t;
	void set_value(recvmsg_result) && noexcept {}
	void set_error(int) && noexcept {}
};
template <class P, class R = message_receive_receiver>
concept message_receive = requires(P& p, std::span<std::byte> in, std::span<std::byte> control, std::span<::iovec> iovs, int flags) {
	requires stdexec::sender_to<decltype(communicate(p, receive_pack(p, in, control, flags))), R>;
	requires stdexec::sender_to<decltype(communicate_range(p, iovs)), R>;
};

// sendmmsg：send_pack(iovecs) 返回 mmsghdr，communicate_range(mmsghdrs)（flags 由 port 自己定），完成值是成功的数据报个数。
// multi_send 的参考 receiver：正好接得住 multi_send 里各组合发出的完成；用户的 receiver 照这个写，作为 multi_send 的第二个参数检查。
struct multi_send_receiver {
	using receiver_concept = stdexec::receiver_t;
	void set_value(std::size_t) && noexcept {}
	void set_error(int) && noexcept {}
};
template <class P, class R = multi_send_receiver>
concept multi_send = requires(P& p, std::span<::iovec> iov, std::span<::mmsghdr> msgs) {
	{ send_pack(p, iov) } -> std::same_as<::mmsghdr>;
	requires stdexec::sender_to<decltype(communicate_range(p, msgs)), R>;
};

// recvmmsg：receive_pack(iovecs) 返回 mmsghdr，communicate_range(mmsghdrs)（flags、timeout 由 port 自己定），完成值是收到的数据报个数。
// multi_receive 的参考 receiver：正好接得住 multi_receive 里各组合发出的完成；用户的 receiver 照这个写，作为 multi_receive 的第二个参数检查。
struct multi_receive_receiver {
	using receiver_concept = stdexec::receiver_t;
	void set_value(std::size_t) && noexcept {}
	void set_error(int) && noexcept {}
};
template <class P, class R = multi_receive_receiver>
concept multi_receive = requires(P& p, std::span<::iovec> iov, std::span<::mmsghdr> msgs) {
	{ receive_pack(p, iov) } -> std::same_as<::mmsghdr>;
	requires stdexec::sender_to<decltype(communicate_range(p, msgs)), R>;
};

namespace detail {

inline ::iovec iov(std::span<std::byte const> d) noexcept { return {const_cast<std::byte*>(d.data()), d.size()}; }

}  // namespace detail

// port：send / recv / sendfile / splice。
struct port {
	int fd;

	struct send_packet {
		std::span<std::byte const> data;
		int flags;
	};
	struct sendfile_packet {
		file_region region;
	};
	struct splice_out_packet {
		pipe_region from;
		unsigned flags;
	};
	struct receive_packet {
		std::span<std::byte> data;
		int flags;
	};
	struct splice_in_packet {
		pipe_region to;
		unsigned flags;
	};

	// flags：MSG_NOSIGNAL、MSG_DONTWAIT、MSG_ZEROCOPY（fd 先 setsockopt(SO_ZEROCOPY)）等。
	// MSG_ZEROCOPY 时返回只表示已交给内核；buf 何时可复用看错误队列（errqueue_port），序号由用户自己数。
	send_packet send_pack(std::span<std::byte const> d, int flags = 0) const noexcept { return {d, flags}; }
	sendfile_packet send_pack(file_region r) const noexcept { return {r}; }
	// flags：SPLICE_F_MOVE、SPLICE_F_NONBLOCK 等。
	splice_out_packet send_pack(pipe_region p, unsigned flags = 0) const noexcept { return {p, flags}; }
	// flags：MSG_PEEK（只看不取）、MSG_DONTWAIT、MSG_WAITALL、MSG_TRUNC 等。
	receive_packet receive_pack(std::span<std::byte> d, int flags = 0) const noexcept { return {d, flags}; }
	splice_in_packet receive_pack(pipe_region p, unsigned flags = 0) const noexcept { return {p, flags}; }

	// send。返回发出的字节数。
	auto communicate(send_packet p) const noexcept {
		return ibus::detail::syscall<std::size_t>([fd = fd, p] noexcept { return ::send(fd, p.data.data(), p.data.size(), p.flags); });
	}

	// sendfile：文件 → socket。返回发出的字节数。
	auto communicate(sendfile_packet p) const noexcept {
		return ibus::detail::syscall<std::size_t>([fd = fd, p] noexcept {
			::off_t off = p.region.offset;
			return ::sendfile(fd, p.region.fd, &off, p.region.count);
		});
	}

	// splice：pipe → socket。返回搬运的字节数。
	auto communicate(splice_out_packet p) const noexcept {
		return ibus::detail::syscall<std::size_t>([fd = fd, p] noexcept { return ::splice(p.from.fd, nullptr, fd, nullptr, p.from.count, p.flags); });
	}

	// recv。返回收到的字节数（0 怎么解读由用户决定；MSG_TRUNC 时是真实长度）。
	auto communicate(receive_packet p) const noexcept {
		return ibus::detail::syscall<std::size_t>([fd = fd, p] noexcept { return ::recv(fd, p.data.data(), p.data.size(), p.flags); });
	}

	// splice：socket → pipe。返回搬运的字节数。
	auto communicate(splice_in_packet p) const noexcept {
		return ibus::detail::syscall<std::size_t>([fd = fd, p] noexcept { return ::splice(fd, nullptr, p.to.fd, nullptr, p.to.count, p.flags); });
	}
};

// errqueue_port：一个 socket 的错误队列（MSG_ZEROCOPY 的“可复用”通知、IP_RECVERR 的 ICMP 错误等）。
// recvmsg(MSG_ERRQUEUE) 永不阻塞：队列空时以 EAGAIN 失败（set_error）。要等，用户自己 poll(POLLERR)。
// 控制缓冲由用户给（至少 CMSG_SPACE(sizeof(sock_extended_err))），返回指向其中 sock_extended_err 的指针，解读由用户决定。
struct errqueue_port {
	int fd;

	struct receive_packet {
		std::span<std::byte> control;
	};

	receive_packet receive_pack(std::span<std::byte> control) const noexcept { return {control}; }

	auto communicate(receive_packet p) const noexcept {
		return ibus::detail::complete<stdexec::set_value_t(::sock_extended_err const*), stdexec::set_error_t(int)>(
			[fd = fd, p](auto&& r) noexcept {
				::msghdr m{};
				m.msg_control = p.control.data();
				m.msg_controllen = p.control.size();
				if (::recvmsg(fd, &m, MSG_ERRQUEUE) < 0) return stdexec::set_error(std::move(r), errno);
				stdexec::set_value(std::move(r), reinterpret_cast<::sock_extended_err const*>(CMSG_DATA(CMSG_FIRSTHDR(&m))));
			});
	}
};

// writev_port / readv_port：动态 n 段合成 1 个单元。
// send_pack / receive_pack 直接返回 iovec；用户把任意多个放进一段连续内存（上限 IOV_MAX 由用户保证）。
// 返回总字节数（只有一个总数，每段填了多少由用户自己算）。iovec 不带方向，所以发、收是两个 port。
struct writev_port {
	int fd;

	::iovec send_pack(std::span<std::byte const> d) const noexcept { return detail::iov(d); }

private:
	auto _writev(auto v) const noexcept(std::is_nothrow_move_constructible_v<decltype(v)>) {
		return ibus::detail::syscall<std::size_t>([fd = fd, v = std::move(v)] mutable noexcept {
			return ::writev(fd, std::ranges::data(v), static_cast<int>(std::ranges::size(v)));
		});
	}

public:
	// 左值 / 视图：只记地址，元素由用户保证活到完成。
	auto communicate_range(contiguous_range_of<::iovec const> auto&& r) const noexcept
		requires std::ranges::borrowed_range<decltype(r)>
	{
		return _writev(std::span<::iovec const>(r));
	}

	// 持有元素的右值（临时 array / vector）：移进 sender。
	auto communicate_range(contiguous_range_of<::iovec const> auto&& r) const
		requires(!std::ranges::borrowed_range<decltype(r)>)
	{
		return _writev(std::move(r));
	}
};

struct readv_port {
	int fd;

	::iovec receive_pack(std::span<std::byte> d) const noexcept { return detail::iov(d); }

private:
	auto _readv(auto v) const noexcept(std::is_nothrow_move_constructible_v<decltype(v)>) {
		return ibus::detail::syscall<std::size_t>([fd = fd, v = std::move(v)] mutable noexcept {
			return ::readv(fd, std::ranges::data(v), static_cast<int>(std::ranges::size(v)));
		});
	}

public:
	// 左值 / 视图：只记地址，元素由用户保证活到完成。
	auto communicate_range(contiguous_range_of<::iovec const> auto&& r) const noexcept
		requires std::ranges::borrowed_range<decltype(r)>
	{
		return _readv(std::span<::iovec const>(r));
	}

	// 持有元素的右值（临时 array / vector）：移进 sender。
	auto communicate_range(contiguous_range_of<::iovec const> auto&& r) const
		requires(!std::ranges::borrowed_range<decltype(r)>)
	{
		return _readv(std::move(r));
	}
};

namespace detail {

// sendmsg：gso_size != 0 时附带 UDP_SEGMENT cmsg，内核把这一块按 gso_size 切成多个 UDP 包发出。
inline ::ssize_t sendmsg(int fd, ::iovec* v, std::size_t n, int flags, std::uint16_t gso_size) noexcept {
	alignas(::cmsghdr) char ctrl[CMSG_SPACE(sizeof(std::uint16_t))];
	::msghdr m{};
	m.msg_iov = v;
	m.msg_iovlen = n;
	if (gso_size != 0) {
		m.msg_control = ctrl;
		m.msg_controllen = sizeof ctrl;
		::cmsghdr* c = CMSG_FIRSTHDR(&m);
		c->cmsg_level = IPPROTO_UDP;
		c->cmsg_type = UDP_SEGMENT;
		c->cmsg_len = CMSG_LEN(sizeof(std::uint16_t));
		std::memcpy(CMSG_DATA(c), &gso_size, sizeof(std::uint16_t));
	}
	return ::sendmsg(fd, &m, flags);
}

}  // namespace detail

// sendmsg_port：sendmsg，1 段或动态 n 段 iovec 合成 1 个单元，可带 UDP GSO。
// 1 段的 flags、gso_size 是 send_pack 的参数；动态 n 段用 port 的成员 flags、gso_size（由用户设置）。
struct sendmsg_port {
	int fd;
	int flags;               // 动态 n 段用：MSG_NOSIGNAL 等
	std::uint16_t gso_size;  // 动态 n 段用：非 0 时附带 UDP_SEGMENT

	struct send_packet {
		std::span<std::byte const> data;
		int flags;
		std::uint16_t gso_size;
	};

	send_packet send_pack(std::span<std::byte const> d, int flags = 0, std::uint16_t gso_size = 0) const noexcept { return {d, flags, gso_size}; }

	// 1 段。返回发出的字节数。
	auto communicate(send_packet p) const noexcept {
		return ibus::detail::syscall<std::size_t>([fd = fd, p] noexcept {
			::iovec v = detail::iov(p.data);
			return detail::sendmsg(fd, &v, 1, p.flags, p.gso_size);
		});
	}

private:
	auto _sendmsg(auto v) const noexcept(std::is_nothrow_move_constructible_v<decltype(v)>) {
		return ibus::detail::syscall<std::size_t>([fd = fd, v = std::move(v), flags = flags, gso_size = gso_size] mutable noexcept {
			return detail::sendmsg(fd, std::ranges::data(v), std::ranges::size(v), flags, gso_size);
		});
	}

public:
	// 动态 n 段：一段连续的 iovec。完成值是发出的字节数。
	// 左值 / 视图：只记地址，元素由用户保证活到完成。
	auto communicate_range(contiguous_range_of<::iovec> auto&& r) const noexcept
		requires std::ranges::borrowed_range<decltype(r)>
	{
		return _sendmsg(std::span<::iovec>(r));
	}

	// 持有元素的右值（临时 array / vector）：移进 sender。
	auto communicate_range(contiguous_range_of<::iovec> auto&& r) const
		requires(!std::ranges::borrowed_range<decltype(r)>)
	{
		return _sendmsg(std::move(r));
	}
};

// recvmsg_port：recvmsg，1 段或动态 n 段 iovec，可带控制缓冲（cmsg：时间戳、IP_PKTINFO、UDP_GRO 的 gso_size 等）。
// cmsg 的解析由用户做（CMSG_FIRSTHDR / CMSG_NXTHDR 遍历用户给的控制缓冲）。
// 1 段的控制缓冲、flags 是 receive_pack 的参数；动态 n 段用 port 的成员 control、flags（由用户设置）。
struct recvmsg_port {
	int fd;
	std::span<std::byte> control;  // 动态 n 段用：控制缓冲（可为空）
	int flags;                     // 动态 n 段用：MSG_DONTWAIT、MSG_TRUNC 等

	using result = recvmsg_result;

	struct receive_packet {
		std::span<std::byte> data;
		std::span<std::byte> control;
		int flags;
	};

	receive_packet receive_pack(std::span<std::byte> d, std::span<std::byte> control = {}, int flags = 0) const noexcept { return {d, control, flags}; }

private:
	static auto _make(auto f) noexcept { return ibus::detail::complete<stdexec::set_value_t(result), stdexec::set_error_t(int)>(std::move(f)); }

	static void _receive(auto&& r, int fd, ::iovec* v, std::size_t n, std::span<std::byte> control, int flags) noexcept {
		::msghdr m{};
		m.msg_iov = v;
		m.msg_iovlen = n;
		m.msg_control = control.data();
		m.msg_controllen = control.size();
		::ssize_t k = ::recvmsg(fd, &m, flags);
		if (k < 0) return stdexec::set_error(std::move(r), errno);
		stdexec::set_value(std::move(r), result{static_cast<std::size_t>(k), m.msg_flags, m.msg_controllen});
	}

public:
	auto communicate(receive_packet p) const noexcept {
		return _make([fd = fd, p](auto&& r) noexcept {
			::iovec v = detail::iov(p.data);
			_receive(std::move(r), fd, &v, 1, p.control, p.flags);
		});
	}

private:
	auto _recvmsg(auto v) const noexcept(std::is_nothrow_move_constructible_v<decltype(v)>) {
		return _make([fd = fd, v = std::move(v), control = control, flags = flags](auto&& r) mutable noexcept {
			_receive(std::move(r), fd, std::ranges::data(v), std::ranges::size(v), control, flags);
		});
	}

public:
	// 动态 n 段：一段连续的 iovec。
	// 左值 / 视图：只记地址，元素由用户保证活到完成。
	auto communicate_range(contiguous_range_of<::iovec> auto&& r) const noexcept
		requires std::ranges::borrowed_range<decltype(r)>
	{
		return _recvmsg(std::span<::iovec>(r));
	}

	// 持有元素的右值（临时 array / vector）：移进 sender。
	auto communicate_range(contiguous_range_of<::iovec> auto&& r) const
		requires(!std::ranges::borrowed_range<decltype(r)>)
	{
		return _recvmsg(std::move(r));
	}
};

// sendmmsg_port / recvmmsg_port：动态 n 个数据报一次系统调用。
// send_pack / receive_pack 直接返回 mmsghdr；每个数据报的内存用一段 iovec 描述（iovec 数组由用户持有）。
// communicate_range 返回成功的数据报个数 k（前 k 个有效）；msg_len 是输出：内核写回每个数据报实际的字节数，
// 接收时 msg_hdr.msg_flags 也由内核写回（MSG_TRUNC 等）。
struct sendmmsg_port {
	int fd;
	int flags;  // sendmmsg 的 flags（由用户设置）

	::mmsghdr send_pack(std::span<::iovec> iov) const noexcept {
		::mmsghdr h{};
		h.msg_hdr.msg_iov = iov.data();
		h.msg_hdr.msg_iovlen = iov.size();
		return h;
	}

private:
	auto _sendmmsg(auto v) const noexcept(std::is_nothrow_move_constructible_v<decltype(v)>) {
		return ibus::detail::syscall<std::size_t>([fd = fd, v = std::move(v), flags = flags] mutable noexcept {
			return ::sendmmsg(fd, std::ranges::data(v), static_cast<unsigned>(std::ranges::size(v)), flags);
		});
	}

public:
	// 左值 / 视图：只记地址，元素由用户保证活到完成。
	auto communicate_range(contiguous_range_of<::mmsghdr> auto&& r) const noexcept
		requires std::ranges::borrowed_range<decltype(r)>
	{
		return _sendmmsg(std::span<::mmsghdr>(r));
	}

	// 持有元素的右值（临时 array / vector）：移进 sender。
	auto communicate_range(contiguous_range_of<::mmsghdr> auto&& r) const
		requires(!std::ranges::borrowed_range<decltype(r)>)
	{
		return _sendmmsg(std::move(r));
	}
};

struct recvmmsg_port {
	int fd;
	int flags;           // 例如 MSG_WAITFORONE（收到第一个后不再阻塞）（由用户设置）
	::timespec* timeout;  // recvmmsg 的超时参数，nullptr 表示不限（由用户设置）

	::mmsghdr receive_pack(std::span<::iovec> iov) const noexcept {
		::mmsghdr h{};
		h.msg_hdr.msg_iov = iov.data();
		h.msg_hdr.msg_iovlen = iov.size();
		return h;
	}

private:
	auto _recvmmsg(auto v) const noexcept(std::is_nothrow_move_constructible_v<decltype(v)>) {
		return ibus::detail::syscall<std::size_t>([fd = fd, v = std::move(v), flags = flags, timeout = timeout] mutable noexcept {
			return ::recvmmsg(fd, std::ranges::data(v), static_cast<unsigned>(std::ranges::size(v)), flags, timeout);
		});
	}

public:
	// 左值 / 视图：只记地址，元素由用户保证活到完成。
	auto communicate_range(contiguous_range_of<::mmsghdr> auto&& r) const noexcept
		requires std::ranges::borrowed_range<decltype(r)>
	{
		return _recvmmsg(std::span<::mmsghdr>(r));
	}

	// 持有元素的右值（临时 array / vector）：移进 sender。
	auto communicate_range(contiguous_range_of<::mmsghdr> auto&& r) const
		requires(!std::ranges::borrowed_range<decltype(r)>)
	{
		return _recvmmsg(std::move(r));
	}
};

static_assert(stream<port> && file_send<port> && splicing<port>);
static_assert(error_queue<errqueue_port>);
static_assert(gather_write<writev_port> && scatter_read<readv_port>);
static_assert(message_send<sendmsg_port> && message_receive<recvmsg_port>);
static_assert(multi_send<sendmmsg_port> && multi_receive<recvmmsg_port>);

}  // namespace ibus::sock
