#include <ibus/io_uring.hpp>

// 先编排、再一次提交：io_uring，原生接口（不用 liburing）。
//   sq_link：send_pack / receive_pack 在共享内存里的 SQE 上原地填写，返回 SQE 下标（槽位由这里分配）；
//            communicate_range(下标) 写进 SQ 索引环、推进 tail，一次 io_uring_enter；
//            set_sqe_flags(IOSQE_IO_LINK) 让它们按顺序执行。
//   cq_link：communicate(receive_pack()) 等一个 CQE，返回 uring::cqe（析构时推进 CQ head）；靠 user_data 对应。
// link 不持有资源：io_uring_setup、mmap、munmap、close 都由这里做。

#include <algorithm>
#include <array>
#include <cstdio>
#include <exception>
#include <span>
#include <system_error>

#include <sys/mman.h>
#include <sys/socket.h>
#include <sys/syscall.h>
#include <unistd.h>

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
		uring::ring ring = uring::ring::at(rfd, p, sq, cq, sqes);

		int sv[2];
		::socketpair(AF_UNIX, SOCK_STREAM | SOCK_CLOEXEC, 0, sv);
		uring::sq_link a(ring, sv[0]), b(ring, sv[1]);
		uring::cq_link cqs(ring);

		// a：发 ping、再收，用 IO_LINK 串成链（按顺序执行）。SQE 槽位 0、1
		char ping[] = "ping", pong[] = "pong";
		std::array<std::byte, 4> a_rx{}, b_rx{};
		a.set_sqe_flags(IOSQE_IO_LINK);
		std::array<std::uint32_t, 2> sa;
		sa[0] = send_pack(a, 0, std::as_bytes(std::span(ping, 4)), 1);
		a.set_sqe_flags(0);
		sa[1] = receive_pack(a, 1, std::span(a_rx), 2);
		std::printf("a submitted %zu\n", communicate_range(a, sa));

		// b：先收、再回 pong，同一个 ring。非 SQPOLL 时上面提交返回后槽位 0、1 已可复用，这里仍用 2、3 便于对照
		std::array<std::uint32_t, 2> sb{receive_pack(b, 2, std::span(b_rx), 3), send_pack(b, 3, std::as_bytes(std::span(pong, 4)), 4)};
		std::printf("b submitted %zu\n", communicate_range(b, sb));

		// 4 个 CQE，到达顺序不定，靠 user_data 区分
		for (int i = 0; i < 4; ++i) {
			auto c = communicate(cqs, receive_pack(cqs));
			std::printf("cqe user_data=%llu res=%d\n", static_cast<unsigned long long>(c->user_data), c->res);
		}
		std::printf("a got %.4s, b got %.4s\n", reinterpret_cast<char*>(a_rx.data()), reinterpret_cast<char*>(b_rx.data()));

		::close(sv[0]);
		::close(sv[1]);
		::munmap(sqes, sqes_size);
		if (!single) ::munmap(cq, cq_size);
		::munmap(sq, sq_size);
		::close(rfd);
	} catch (std::exception const& e) {
		std::printf("error: %s\n", e.what());
	}
}
