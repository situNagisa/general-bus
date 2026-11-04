#include <ibus/io_uring.hpp>

// 先编排、再一次提交：io_uring。
//   sq_link：send_pack / receive_pack 返回 io_uring_sqe（带 user_data），用户放进自己的连续内存，
//            communicate_range 一次 io_uring_submit；set_sqe_flags(IOSQE_IO_LINK) 让它们按顺序执行。
//   cq_link：communicate(receive_pack()) 等一个 CQE，返回 uring::cqe（析构时 cqe_seen）；靠 user_data 对应。
// link 不持有资源：ring、fd 都由这里建、关。

#include <array>
#include <cstdio>
#include <exception>
#include <span>
#include <system_error>
#include <vector>

#include <sys/socket.h>
#include <unistd.h>

int main() {
	using namespace ibus;
	::io_uring ring;
	if (int r = ::io_uring_queue_init(16, &ring, 0); r < 0) {
		std::printf("io_uring_queue_init: %s\n", std::system_category().message(-r).c_str());
		return 1;
	}
	try {
		int sv[2];
		::socketpair(AF_UNIX, SOCK_STREAM | SOCK_CLOEXEC, 0, sv);
		uring::sq_link a(ring, sv[0]), b(ring, sv[1]);
		uring::cq_link cq(ring);

		// a：发 ping、再收，用 IO_LINK 串成链（按顺序执行）
		char ping[] = "ping", pong[] = "pong";
		std::array<std::byte, 4> a_rx{}, b_rx{};
		a.set_sqe_flags(IOSQE_IO_LINK);
		std::vector<::io_uring_sqe> sa{send_pack(a, std::as_bytes(std::span(ping, 4)), 1)};
		a.set_sqe_flags(0);
		sa.push_back(receive_pack(a, std::span(a_rx), 2));
		std::printf("a submitted %zu\n", communicate_range(a, sa));

		// b：先收、再回 pong，同一个 ring
		std::vector<::io_uring_sqe> sb{receive_pack(b, std::span(b_rx), 3), send_pack(b, std::as_bytes(std::span(pong, 4)), 4)};
		std::printf("b submitted %zu\n", communicate_range(b, sb));

		// 4 个 CQE，到达顺序不定，靠 user_data 区分
		for (int i = 0; i < 4; ++i) {
			auto c = communicate(cq, receive_pack(cq));
			std::printf("cqe user_data=%llu res=%d\n", static_cast<unsigned long long>(c->user_data), c->res);
		}
		std::printf("a got %.4s, b got %.4s\n", reinterpret_cast<char*>(a_rx.data()), reinterpret_cast<char*>(b_rx.data()));
		::close(sv[0]);
		::close(sv[1]);
	} catch (std::exception const& e) {
		std::printf("error: %s\n", e.what());
	}
	::io_uring_queue_exit(&ring);
}
