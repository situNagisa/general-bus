/*
	* cqe_skip_success.cpp — IOSQE_CQE_SKIP_SUCCESS：成功不产生 CQE，失败仍产生
	*
	* 一、这是什么、解决什么问题
	* 默认情况下“1 个 SQE = 1 个 CQE”。但很多操作应用只关心“有没有成功”，
	* 比如关闭 fd、fsync 一个同步文件、写一段不需要确认的数据。为这些操作
	* 保留完成事件会白白占用 CQ 环、增加用户态收割和唤醒开销。
	* IOSQE_CQE_SKIP_SUCCESS（内核 5.17 起）让请求照常执行，但成功完成时
	* 不往 CQ 写 CQE；失败时仍然写，res 为 -errno。于是完成事件数不再是
	* 1:1，而是 0 或 1。
	*
	* 二、形态
	*   - 每个操作仍占 1 个 SQE，仍要 submit；只是成功后没有 CQE；
	*   - 成功/失败的判定与 IOSQE_IO_LINK 断链的判定一致，具体由 opcode
	*     决定（见 io_uring_enter(2) 的 IOSQE_CQE_SKIP_SUCCESS 一节）；
	*   - 本例两个场景各提交 1 个 SQE：成功场景看到 0 个 CQE，失败场景
	*     看到 1 个 CQE；
	*   - “线上单元”：一条 UNIX 域 DGRAM 连接，每次 recv 取一条报文；
	*   - 数据缓冲区仍由应用提供。因为没有成功 CQE，应用无法知道操作何时
	*     真正结束，所以只能在“不需要立刻复用缓冲区/fd”的场合使用它。
	*
	* 三、注意事项
	*   - 没有 CQE 不等于没执行：必须靠副作用（本例再读一次得到 EAGAIN）
	*     或别的同步手段确认；
	*   - 与 IOSQE_IO_DRAIN 不兼容，同一 ring 里混用是未定义行为；
	*   - 用在链上时，如果带 skip 的请求失败并取消了后续链成员，且没有
	*     HARDLINK，后续被取消成员的 CQE 也会被省略（本机 6.18 实测）；
	*   - 给 IORING_OP_LINK_TIMEOUT 打 skip，则无论超时还是被取消都不会
	*     产生 CQE，是否产生只看它自己的 skip 标志（man page 明确说明）。
	*
	* 四、内核条件
	*   - 内核 >= 5.17；不需要特殊硬件。
	*
	* 五、编译与运行
	*   g++ -std=c++26 -O2 -Wall -Wextra -o cqe_skip_success cqe_skip_success.cpp -luring
	*   ./cqe_skip_success
	*
	* 六、预期输出（本机 6.18 实测）
	*   === 1. 成功：recv 带上 IOSQE_CQE_SKIP_SUCCESS ===
	*   提交 1 个 SQE，等待 200ms 收割完成事件……
	*   成功场景：看到 0 个 CQE（预期 0）
	*   再用普通 recv(MSG_DONTWAIT) 读一次：EAGAIN
	*   → 说明 skip 的那次 recv 确实执行了并取走了数据，只是没有报告成功。
	*
	*   === 2. 失败：recv 空套接字 + MSG_DONTWAIT，同样带 skip ===
	*   提交 1 个 SQE，等待 200ms 收割完成事件……
	*   收到 CQE：user_data=0x32 res=-11（Resource temporarily unavailable）
	*   失败场景：看到 1 个 CQE（预期 1）
	*   → 失败不会被吞掉，这正是 skip 只跳过“成功”的含义。
	*
	*   结论：成功 0 个 CQE，失败 1 个 CQE。
	*/

#include <array>
#include <cerrno>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <print>
#include <string_view>
#include <utility>

#include <sys/socket.h>
#include <unistd.h>

#include <liburing.h>

// ---------------------------------------------------------------------------
// 小工具（同 single_sqe.cpp，每个示例自包含）
// ---------------------------------------------------------------------------
[[noreturn]] void fail(std::string_view what, int err)
{
	std::println(stderr, "{} 失败：{} (errno={})", what, std::strerror(err), err);
	std::exit(EXIT_FAILURE);
}

class UniqueFd {
public:
	UniqueFd() = default;
	explicit UniqueFd(int fd) : fd_(fd) {}
	~UniqueFd()
	{
		if (fd_ >= 0)
			::close(fd_);
	}

	UniqueFd(UniqueFd &&other) noexcept : fd_(std::exchange(other.fd_, -1)) {}
	UniqueFd &operator=(UniqueFd &&other) noexcept
	{
		if (this != &other) {
			if (fd_ >= 0)
				::close(fd_);
			fd_ = std::exchange(other.fd_, -1);
		}
		return *this;
	}
	UniqueFd(const UniqueFd &) = delete;
	UniqueFd &operator=(const UniqueFd &) = delete;

	int get() const { return fd_; }

private:
	int fd_ = -1;
};

class Ring {
public:
	explicit Ring(unsigned entries)
	{
		int ret = io_uring_queue_init(entries, &ring_, 0);
		if (ret < 0)
			fail("io_uring_queue_init", -ret);
		initialized_ = true;
	}
	~Ring()
	{
		if (initialized_)
			io_uring_queue_exit(&ring_);
	}
	Ring(const Ring &) = delete;
	Ring &operator=(const Ring &) = delete;

	io_uring_sqe *get_sqe()
	{
		io_uring_sqe *sqe = io_uring_get_sqe(&ring_);
		if (sqe == nullptr)
			fail("io_uring_get_sqe（SQ 已满，应先 submit）", ENOSPC);
		return sqe;
	}

	int submit()
	{
		int ret = io_uring_submit(&ring_);
		if (ret < 0)
			fail("io_uring_submit", -ret);
		return ret;
	}

	// 带超时的等待：没有 CQE 时最多等 ts 指定的时间，然后返回 -ETIME。
	// 本例正是用它在“预期 0 个 CQE”时也能正常返回，而不是永久阻塞。
	int wait_cqe_timeout(io_uring_cqe **cqe, __kernel_timespec *ts)
	{
		return io_uring_wait_cqe_timeout(&ring_, cqe, ts);
	}

	void cqe_seen(io_uring_cqe *cqe)
	{
		io_uring_cqe_seen(&ring_, cqe);
	}

private:
	io_uring ring_{};
	bool initialized_ = false;
};

struct SocketPair {
	UniqueFd a;
	UniqueFd b;
};

SocketPair make_socketpair(int type)
{
	int sv[2];
	if (::socketpair(AF_UNIX, type, 0, sv) < 0)
		fail("socketpair", errno);
	return SocketPair{UniqueFd{sv[0]}, UniqueFd{sv[1]}};
}

// 在 timeout_ms 内收割所有 CQE，逐条打印并返回条数。
// 注意：这里故意不使用无限等待——成功场景按设计一个 CQE 都不会来。
int drain_with_timeout(Ring &ring, int timeout_ms)
{
	__kernel_timespec ts{
		.tv_sec = 0,
		.tv_nsec = static_cast<long>(timeout_ms) * 1'000'000L,
	};

	int count = 0;
	for (;;) {
		io_uring_cqe *cqe = nullptr;
		int ret = ring.wait_cqe_timeout(&cqe, &ts);
		if (ret == -ETIME)
			break; // 到点，没有更多 CQE
		if (ret < 0)
			fail("io_uring_wait_cqe_timeout", -ret);

		if (cqe->res < 0) {
			std::println("收到 CQE：user_data={:#x} res={}（{}）",
				cqe->user_data, cqe->res, std::strerror(-cqe->res));
		} else {
			std::println("收到 CQE：user_data={:#x} res={}",
				cqe->user_data, cqe->res);
		}
		ring.cqe_seen(cqe);
		++count;
	}
	return count;
}

int main()
{
	// -----------------------------------------------------------------------
	// 场景 1：操作成功 → 0 个 CQE
	// -----------------------------------------------------------------------
	std::println("=== 1. 成功：recv 带上 IOSQE_CQE_SKIP_SUCCESS ===");

	auto [tx1, rx1] = make_socketpair(SOCK_DGRAM);
	constexpr std::string_view kMsg = "skip-ok";
	ssize_t sent = ::send(tx1.get(), kMsg.data(), kMsg.size(), 0);
	if (sent != static_cast<ssize_t>(kMsg.size()))
		fail("send", errno);

	std::array<char, 32> buf1{};
	Ring ring{8};

	io_uring_sqe *sqe = ring.get_sqe();
	io_uring_prep_recv(sqe, rx1.get(), buf1.data(), buf1.size(), 0);
	// 关键标志：成功完成时不产生 CQE。
	// 对应位是 IOSQE_CQE_SKIP_SUCCESS_BIT = 6（即 1u << 6），
	// 这里用宏，避免手写位号。
	sqe->flags |= IOSQE_CQE_SKIP_SUCCESS;
	io_uring_sqe_set_data64(sqe, 0x31);

	std::println("提交 {} 个 SQE，等待 200ms 收割完成事件……", ring.submit());
	int ok_count = drain_with_timeout(ring, 200);
	std::println("成功场景：看到 {} 个 CQE（预期 0）", ok_count);
	if (ok_count != 0)
		return EXIT_FAILURE;

	// 怎么证明这次 recv 真的执行了？再读一次：数据已被取走 → EAGAIN。
	errno = 0;
	ssize_t extra = ::recv(rx1.get(), buf1.data(), buf1.size(), MSG_DONTWAIT);
	if (extra < 0 && errno == EAGAIN) {
		std::println("再用普通 recv(MSG_DONTWAIT) 读一次：EAGAIN");
		std::println("→ 说明 skip 的那次 recv 确实执行了并取走了数据，只是没有报告成功。");
	} else {
		std::println(stderr, "意外：再次读取返回 {}（errno={}），数据可能没被消费",
			extra, errno);
		return EXIT_FAILURE;
	}

	// -----------------------------------------------------------------------
	// 场景 2：操作失败 → 仍然产生 1 个 CQE
	// -----------------------------------------------------------------------
	std::println("\n=== 2. 失败：recv 空套接字 + MSG_DONTWAIT，同样带 skip ===");

	auto [tx2, rx2] = make_socketpair(SOCK_DGRAM);
	(void)tx2; // 不写数据，让 recv 立刻失败

	std::array<char, 32> buf2{};
	Ring ring2{8};

	sqe = ring2.get_sqe();
	// MSG_DONTWAIT：套接字为空时不要挂起，直接返回 -EAGAIN。
	// skip 只省略“成功”的 CQE，失败事件一定投递。
	io_uring_prep_recv(sqe, rx2.get(), buf2.data(), buf2.size(), MSG_DONTWAIT);
	sqe->flags |= IOSQE_CQE_SKIP_SUCCESS;
	io_uring_sqe_set_data64(sqe, 0x32);

	std::println("提交 {} 个 SQE，等待 200ms 收割完成事件……", ring2.submit());
	int fail_count = drain_with_timeout(ring2, 200);
	std::println("失败场景：看到 {} 个 CQE（预期 1）", fail_count);
	if (fail_count != 1)
		return EXIT_FAILURE;

	std::println("\n结论：成功 0 个 CQE，失败 1 个 CQE。");
	return 0;
}
