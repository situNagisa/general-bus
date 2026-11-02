#include <ibus/io_uring.hpp>

// socketpair 上的 ping-pong：a 编排“发 ping、收”，b 编排“收、发 pong”，各自一次提交；
// 之后用 communicate<kind::receive> 收 4 个 CQE（send 的也在内），按 user_data 区分。
// 最后演示出错路径：对非 socket 的 fd 发 SEND，res < 0 由用户自己解读。

#include <array>
#include <cstdio>
#include <exception>
#include <span>
#include <system_error>

#include <fcntl.h>
#include <sys/socket.h>

// 用户自己的 submission：上限 N，运行期计数；满足 ibus::uring::submission。
template <std::size_t N>
class sqe_batch {
public:
	explicit sqe_batch(int fd) noexcept : fd_(fd) {}

	void send(std::span<std::byte const> d, std::uint64_t ud) { push(IORING_OP_SEND, d, ud); }
	void receive(std::span<std::byte> d, std::uint64_t ud) { push(IORING_OP_RECV, d, ud); }

	auto begin() const noexcept { return sqes_.begin(); }
	auto end() const noexcept { return sqes_.begin() + count_; }
	std::size_t size() const noexcept { return count_; }

private:
	void push(std::uint8_t op, std::span<std::byte const> d, std::uint64_t ud) {
		::io_uring_sqe* sqe = &sqes_[count_++];
		::io_uring_initialize_sqe(sqe);
		::io_uring_prep_rw(op, sqe, fd_, d.data(), static_cast<unsigned>(d.size()), 0);
		::io_uring_sqe_set_data64(sqe, ud);
	}

	int fd_;
	std::array<::io_uring_sqe, N> sqes_;
	std::size_t count_ = 0;
};

static_assert(ibus::uring::submission<sqe_batch<2>>);

int main() {
	using namespace ibus;
	try {
		::io_uring ring;
		::io_uring_queue_init(8, &ring, 0);
		int sv[2];
		::socketpair(AF_UNIX, SOCK_STREAM | SOCK_CLOEXEC, 0, sv);
		uring::link a(ring, sv[0]), b(ring, sv[1]);

		char ping[4] = {'p', 'i', 'n', 'g'}, pong[4] = {'p', 'o', 'n', 'g'};
		std::byte a_rx[4]{}, b_rx[4]{};

		sqe_batch<2> sa(a.fd());
		ibus::send(sa, std::as_bytes(std::span(ping)), 1);
		ibus::receive(sa, std::span(a_rx), 2);
		stdexec::sync_wait(communicate<kind::submit>(a, sa));

		sqe_batch<2> sb(b.fd());
		ibus::receive(sb, std::span(b_rx), 3);
		ibus::send(sb, std::as_bytes(std::span(pong)), 4);
		stdexec::sync_wait(communicate<kind::submit>(b, sb));

		// a、b 共用一个 ring，CQE 从哪个 link 收都一样；靠 user_data 区分。
		for (int i = 0; i < 4; ++i) {
			auto [c] = stdexec::sync_wait(communicate<kind::receive>(a)).value();
			std::printf("cqe user_data=%llu, res=%d\n", static_cast<unsigned long long>(c->user_data), c->res);
		}
		std::printf("a got %.4s, b got %.4s\n", reinterpret_cast<char*>(a_rx), reinterpret_cast<char*>(b_rx));
		::io_uring_queue_exit(&ring);
	} catch (std::exception const& e) {
		std::printf("error: %s\n", e.what());
	}

	try {
		::io_uring ring;
		::io_uring_queue_init(8, &ring, 0);
		uring::link bad(ring, ::open("/dev/null", O_RDWR | O_CLOEXEC));
		char x[1]{};
		sqe_batch<1> s(bad.fd());
		ibus::send(s, std::as_bytes(std::span(x)), 0);
		stdexec::sync_wait(communicate<kind::submit>(bad, std::move(s)));
		auto [c] = stdexec::sync_wait(communicate<kind::receive>(bad)).value();
		if (c->res < 0) throw std::system_error(-c->res, std::generic_category(), "send");
	} catch (std::exception const& e) {
		std::printf("bad: error: %s\n", e.what());
	}
}
