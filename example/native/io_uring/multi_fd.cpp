/*
接口：一次 io_uring_submit 覆盖多个 fd（多路复用式提交），以及固定文件
	io_uring_register_files + IOSQE_FIXED_FILE。
要解决什么：最基础的“一次系统调用处理多个通道”。多个 SQE 可以指向不同的 fd，
一次 io_uring_enter 把它们一起交给内核；每个请求独立执行、各自完成，不要求 fd
相同，也不要求完成顺序与提交顺序一致。服务器可以用一个环管理成百上千条连接，
不需要 select/poll 那种“先问哪些 fd 就绪、再逐个 read”的两段式流程。
它的形态：
	一次 submit 提交 N 个 SQE，产生 N 个 CQE；每个 CQE 对应其 SQE 指定的那个
	fd 上的一次操作（这里是 recv）；数据内存由应用在 SQE 里逐个给出。
	固定文件是另一件正交的事：io_uring_register_files 把一组 fd 注册进环，
	之后 SQE 用“表内下标”代替 fd，并置 IOSQE_FIXED_FILE。内核不再每次请求都
	去进程 fd 表里 fdget/fdput，也避免别的线程 close 后 fd 号被复用带来的竞态。
	注册时内核持有 fd 引用，所以注册后关闭原 fd 不会让环里的操作失效。
需要的条件：普通 multi-fd 是 io_uring 基础能力；固定文件需要内核支持
	IORING_REGISTER_FILES 与 IOSQE_FIXED_FILE（基础功能很早就有，本处未核对
	具体版本；稀疏文件表需 5.19+）。本机 WSL2 内核 6.18 实测可用。
编译：
	g++ -std=c++26 -O2 -Wall -Wextra -o multi_fd multi_fd.cpp -luring
运行：
	./multi_fd
预期输出：
	第一轮 io_uring_submit 返回 2，两个 CQE 分别从两个 socketpair 收到 alpha/beta。
	第二轮先打印“注册 2 个固定文件”，再用索引 0/1 + IOSQE_FIXED_FILE 收
	ALPHA/BETA，同样一次 submit 返回 2。
*/

#include <liburing.h>

#include <array>
#include <cerrno>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <print>
#include <span>
#include <string_view>
#include <utility>

#include <sys/socket.h>
#include <unistd.h>

namespace {

constexpr unsigned kQueueDepth = 8;
constexpr std::size_t kBufSize = 32;

[[noreturn]] void fail(std::string_view what)
{
	std::println(stderr, "错误：{}: {}", what, std::strerror(errno));
	std::exit(EXIT_FAILURE);
}

void check(int ret, std::string_view what)
{
	if (ret < 0) {
		errno = -ret;
		fail(what);
	}
}

class Fd {
public:
	explicit Fd(int fd = -1) noexcept : fd_(fd) {}
	~Fd() { reset(); }

	Fd(const Fd &) = delete;
	Fd &operator=(const Fd &) = delete;

	Fd(Fd &&other) noexcept : fd_(std::exchange(other.fd_, -1)) {}
	Fd &operator=(Fd &&other) noexcept
	{
		if (this != &other) {
			reset();
			fd_ = std::exchange(other.fd_, -1);
		}
		return *this;
	}

	[[nodiscard]] int get() const noexcept { return fd_; }

	void reset(int fd = -1) noexcept
	{
		if (fd_ >= 0)
			::close(fd_);
		fd_ = fd;
	}

private:
	int fd_;
};

class Ring {
public:
	explicit Ring(unsigned entries)
	{
		check(io_uring_queue_init(entries, &ring_, 0), "io_uring_queue_init");
	}
	~Ring() { io_uring_queue_exit(&ring_); }

	Ring(const Ring &) = delete;
	Ring &operator=(const Ring &) = delete;

	[[nodiscard]] io_uring *get() noexcept { return &ring_; }

	[[nodiscard]] io_uring_sqe *get_sqe()
	{
		io_uring_sqe *sqe = io_uring_get_sqe(&ring_);
		if (sqe == nullptr) {
			std::println(stderr, "错误：SQ 环已满");
			std::exit(EXIT_FAILURE);
		}
		return sqe;
	}

	[[nodiscard]] unsigned submit()
	{
		int ret = io_uring_submit(&ring_);
		check(ret, "io_uring_submit");
		return static_cast<unsigned>(ret);
	}

private:
	io_uring ring_{};
};

[[nodiscard]] io_uring_cqe *wait_cqe(Ring &ring)
{
	io_uring_cqe *cqe = nullptr;
	check(io_uring_wait_cqe(ring.get(), &cqe), "io_uring_wait_cqe");
	return cqe;
}

void print_completion(const io_uring_cqe *cqe, std::span<char> buf)
{
	std::print("CQE: user_data=0x{:x} res={}", cqe->user_data, cqe->res);
	if (cqe->res > 0)
		std::print(" 数据=\"{}\"",
			std::string_view(buf.data(), static_cast<std::size_t>(cqe->res)));
	std::println("");
}

} // namespace

int main()
{
	// 两组 socketpair，模拟两条独立连接。sv[0] 是 io_uring 收的一端，
	// sv[1] 是对端写的一端。
	int sa[2];
	int sb[2];
	if (::socketpair(AF_UNIX, SOCK_STREAM | SOCK_CLOEXEC, 0, sa) < 0)
		fail("socketpair A");
	if (::socketpair(AF_UNIX, SOCK_STREAM | SOCK_CLOEXEC, 0, sb) < 0)
		fail("socketpair B");
	Fd a_local(sa[0]);
	Fd a_peer(sa[1]);
	Fd b_local(sb[0]);
	Fd b_peer(sb[1]);
	std::println("通道 A：recv fd={}，peer fd={}", a_local.get(), a_peer.get());
	std::println("通道 B：recv fd={}，peer fd={}", b_local.get(), b_peer.get());

	Ring ring(kQueueDepth);
	std::array<char, kBufSize> buf_a{};
	std::array<char, kBufSize> buf_b{};

	// 先把数据放进两个 socket 的接收队列，保证 recv 提交后能立即完成。
	if (::write(a_peer.get(), "alpha", 5) != 5)
		fail("write A");
	if (::write(b_peer.get(), "beta", 4) != 4)
		fail("write B");

	// 提交前先取两个 SQE，填不同的 fd。
	io_uring_sqe *sqe_a = ring.get_sqe();
	// io_uring_prep_recv 的第 2 个参数是真实 fd；这里两个 SQE 指向不同 fd。
	io_uring_prep_recv(sqe_a, a_local.get(), buf_a.data(), buf_a.size(), 0);
	sqe_a->user_data = 1; // 完成时用它分辨是哪个通道

	io_uring_sqe *sqe_b = ring.get_sqe();
	io_uring_prep_recv(sqe_b, b_local.get(), buf_b.data(), buf_b.size(), 0);
	sqe_b->user_data = 2;

	std::println("\n提交两个 SQE（fd {} 和 {}），只调用一次 io_uring_submit:",
		a_local.get(), b_local.get());
	const unsigned submitted = ring.submit();
	std::println("io_uring_submit 返回 {}（一次 io_uring_enter 提交两个请求）",
		submitted);

	// 两个完成各自到达；CQE 顺序不保证与 SQE 顺序一致，所以用 user_data 区分。
	for (int i = 0; i < 2; ++i) {
		io_uring_cqe *cqe = wait_cqe(ring);
		std::span<char> buf;
		if (cqe->user_data == 1)
			buf = buf_a;
		else
			buf = buf_b;
		print_completion(cqe, buf);
		if (cqe->res < 0)
			std::println(stderr, "错误：recv 失败 {}", std::strerror(-cqe->res));
		io_uring_cqe_seen(ring.get(), cqe);
	}

	// ---- 固定文件 ----
	// 把两个接收端注册到环的固定文件表：下标 0 -> a_local，下标 1 -> b_local。
	// 表大小由 nr_files 决定；这里恰好 2 项，所以合法下标是 0 和 1。
	std::array<int, 2> fixed_table{a_local.get(), b_local.get()};
	const unsigned nr_fixed = static_cast<unsigned>(fixed_table.size());
	check(io_uring_register_files(ring.get(), fixed_table.data(), nr_fixed),
		"io_uring_register_files");
	std::println("\n已注册 {} 个固定文件：索引 0 -> fd {}，索引 1 -> fd {}",
		fixed_table.size(), a_local.get(), b_local.get());

	if (::write(a_peer.get(), "ALPHA", 5) != 5)
		fail("write A2");
	if (::write(b_peer.get(), "BETA", 4) != 4)
		fail("write B2");

	sqe_a = ring.get_sqe();
	// 注意：一旦置了 IOSQE_FIXED_FILE，这个参数就不再是 fd，而是注册表下标。
	// 传下标 0 但忘记置标志，内核会把它当成 stdin，这是最常见的坑。
	io_uring_prep_recv(sqe_a, 0, buf_a.data(), buf_a.size(), 0);
	sqe_a->flags |= IOSQE_FIXED_FILE;
	sqe_a->user_data = 0x100;

	sqe_b = ring.get_sqe();
	io_uring_prep_recv(sqe_b, 1, buf_b.data(), buf_b.size(), 0);
	sqe_b->flags |= IOSQE_FIXED_FILE;
	sqe_b->user_data = 0x101;

	std::println("提交两个使用固定文件索引 0/1 的 SQE:");
	const unsigned submitted_fixed = ring.submit();
	std::println("io_uring_submit 返回 {}", submitted_fixed);

	for (int i = 0; i < 2; ++i) {
		io_uring_cqe *cqe = wait_cqe(ring);
		std::span<char> buf;
		if (cqe->user_data == 0x100)
			buf = buf_a;
		else
			buf = buf_b;
		print_completion(cqe, buf);
		io_uring_cqe_seen(ring.get(), cqe);
	}

	// 注销后固定文件表失效；环析构时也会清理，但显式注销便于展示生命周期。
	check(io_uring_unregister_files(ring.get()), "io_uring_unregister_files");
	std::println("\n已注销固定文件表。固定文件只是省掉每次请求的 fd 查表/引用计数，"
		"仍可继续用普通 fd 方式提交。");
	return 0;
}
