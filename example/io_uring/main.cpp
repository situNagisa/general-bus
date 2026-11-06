#include <ibus/io_uring.hpp>

// 先编排、再一次提交：io_uring，原生接口（不用 liburing）。
// 把 io_uring 当作协议：
//   communicate(sq, send_pack(sq, uring::send / uring::receive, fd, buf, user_data[, sqe_flags, msg_flags]))
//       取下一个 SQE 槽位原地填写（RECV 对内核来说也是“发”一个请求，所以也用 send_pack）；sqe_flags = IOSQE_IO_LINK 串成链
//   communicate(sq, send_pack(sq, uring::submit, wait_nr))  发布 tail，一次 io_uring_enter，顺带等至少 wait_nr 个 CQE
//   communicate(cq, receive_pack(cq))                    取下一个 CQE：指向共享内存，没有就是 nullptr；靠 user_data 对应
//   communicate(cq, send_pack(cq, uring::release))       归还已取出的 CQE（发布 head）
// port 不持有资源：io_uring_setup、mmap、munmap、close 都由这里做。

#include <algorithm>
#include <array>
#include <cstdio>
#include <cstring>
#include <tuple>
#include <exception>
#include <span>
#include <system_error>

#include <sys/mman.h>
#include <sys/socket.h>
#include <sys/syscall.h>
#include <unistd.h>

// sync_wait 取 sender 唯一的完成值（set_error(errno) 时 sync_wait 把 errno 的值作为 int 抛出）。
static auto value(auto&& s) { return std::get<0>(stdexec::sync_wait(std::forward<decltype(s)>(s)).value()); }

int main() {
	using namespace ibus;
	try {
		// 建 ring：io_uring_setup 返回 fd，并在 params 里填回各字段在 mmap 区域里的偏移
		::io_uring_params p{};
		int rfd = static_cast<int>(::syscall(__NR_io_uring_setup, 16, &p));
		if (rfd < 0) throw std::system_error(errno, std::generic_category(), "io_uring_setup");
		std::size_t sq_size = p.sq_off.array + p.sq_entries * sizeof(std::uint32_t);
		std::size_t cq_size = p.cq_off.cqes + p.cq_entries * sizeof(::io_uring_cqe);
		bool single = p.features & IORING_FEAT_SINGLE_MMAP;  // SQ、CQ 两个环在同一段映射里
		if (single) sq_size = cq_size = std::max(sq_size, cq_size);
		void* sq = ::mmap(nullptr, sq_size, PROT_READ | PROT_WRITE, MAP_SHARED | MAP_POPULATE, rfd, IORING_OFF_SQ_RING);
		void* cq = single ? sq : ::mmap(nullptr, cq_size, PROT_READ | PROT_WRITE, MAP_SHARED | MAP_POPULATE, rfd, IORING_OFF_CQ_RING);
		std::size_t sqes_size = p.sq_entries * sizeof(::io_uring_sqe);
		void* sqes = ::mmap(nullptr, sqes_size, PROT_READ | PROT_WRITE, MAP_SHARED | MAP_POPULATE, rfd, IORING_OFF_SQES);
		if (sq == MAP_FAILED || cq == MAP_FAILED || sqes == MAP_FAILED) throw std::system_error(errno, std::generic_category(), "mmap");
		uring::sq_port sqp = uring::sq_port::at(rfd, p, sq, sqes);
		uring::cq_port cqp = uring::cq_port::at(p, cq);

		int sv[2];
		::socketpair(AF_UNIX, SOCK_STREAM | SOCK_CLOEXEC, 0, sv);
		char ping[] = "ping", pong[] = "pong";
		std::array<std::byte, 4> a_rx{}, b_rx{};

		// a 端：发 ping、再收，IO_LINK 串成链（按顺序执行）；b 端：先收、再回 pong。四项一次提交，一个 ring 覆盖两个 fd
		stdexec::sync_wait(communicate(sqp, send_pack(sqp, uring::send, sv[0], std::as_bytes(std::span(ping, 4)), 1, IOSQE_IO_LINK)));
		stdexec::sync_wait(communicate(sqp, send_pack(sqp, uring::receive, sv[0], std::span(a_rx), 2)));
		stdexec::sync_wait(communicate(sqp, send_pack(sqp, uring::receive, sv[1], std::span(b_rx), 3)));
		stdexec::sync_wait(communicate(sqp, send_pack(sqp, uring::send, sv[1], std::as_bytes(std::span(pong, 4)), 4)));
		// 一次 io_uring_enter：提交 4 项，并等到 4 个 CQE 都到
		std::printf("submitted %zu\n", value(communicate(sqp, send_pack(sqp, uring::submit, 4))));

		// 4 个 CQE，到达顺序不定，靠 user_data 区分；全部看完再一次归还
		while (auto* c = value(communicate(cqp, receive_pack(cqp))))
			std::printf("cqe user_data=%llu res=%d\n", static_cast<unsigned long long>(c->user_data), c->res);
		stdexec::sync_wait(communicate(cqp, send_pack(cqp, uring::release)));
		std::printf("a got %.4s, b got %.4s\n", reinterpret_cast<char*>(a_rx.data()), reinterpret_cast<char*>(b_rx.data()));

		::close(sv[0]);
		::close(sv[1]);
		::munmap(sqes, sqes_size);
		if (!single) ::munmap(cq, cq_size);
		::munmap(sq, sq_size);
		::close(rfd);
	} catch (int e) {  // sync_wait 抛出的 errno
		std::printf("error: %s\n", std::strerror(e));
	} catch (std::exception const& e) {
		std::printf("error: %s\n", e.what());
	}
}
