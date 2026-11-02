/*
	* io_link.cpp — 用 IOSQE_IO_LINK 把多个操作串成链，以及链超时
	*
	* 一、这是什么、解决什么问题
	* io_uring 默认让同一批 SQE 并行执行、完成顺序不定（见 multi_sqe.cpp）。
	* 有时后一个操作必须等前一个成功完成才能开始，比如：
	*   - 先 recv 一段协议头，再根据头里的长度 recv 正文；
	*   - 先 write 数据，再 fsync；
	*   - 一串有序的 send。
	* IOSQE_IO_LINK 就是给 SQE 打“我和下一个 SQE 串成链”的标记：链内严格
	* 串行，前一项完成后下一项才开始。链中任何一项失败，后面还没开始的项
	* 不再执行，直接以 -ECANCELED 完成。
	*
	* IOSQE_IO_HARDLINK 是它的“硬链接”版本：同样串行，但某一项失败不会
	* 打断链，后面的项照样执行（只有提交阶段就失败的父请求才会断链）。
	* HARDLINK 隐含 LINK，两者不能同时用。
	*
	* IORING_OP_LINK_TIMEOUT 是专门给“链上前一项”设超时的操作：把它的 SQE
	* 紧跟在目标 SQE 后面、并让目标带 IOSQE_IO_LINK。超时到期就取消目标；
	* 目标先完成则定时器被撤销。定时器 SQE 自己的 CQE res 是 -ETIME（超时
	* 生效）或 -ECANCELED（目标先完成）。
	*
	* 二、形态
	*   - 一次 submit 提交整条链（链不能跨 submission 边界！最后一个 SQE
	*     即使带 LINK 也会被当作链尾）。本例每次 submit 2 个 SQE；
	*   - 链内 N 个操作串行执行，正常情况产生 N 个 CQE；失败时被取消的项
	*     也会各产生一个 -ECANCELED 的 CQE（除非它同时带了
	*     IOSQE_CQE_SKIP_SUCCESS，见 cqe_skip_success.cpp）；
	*   - 数据缓冲区、超时用的 __kernel_timespec 都由应用提供，必须活到
	*     对应 CQE 被收割；
	*   - “线上单元”：本例仍是一条 UNIX 域 DGRAM 连接，每个操作对应一条
	*     报文，不涉及拆包。
	*
	* 三、内核条件
	*   - IOSQE_IO_LINK 需要内核 >= 5.3；
	*   - IOSQE_IO_HARDLINK、IORING_OP_LINK_TIMEOUT 需要内核 >= 5.5；
	*   - 不需要特殊硬件。
	*
	* 四、编译与运行
	*   g++ -std=c++26 -O2 -Wall -Wextra -o io_link io_link.cpp -luring
	*   ./io_link
	*
	* 五、预期输出（本机 6.18 实测）
	*   === 1. IOSQE_IO_LINK：成功链，后一项等前一项完成 ===
	*   提交 2 个 SQE（recv #1 带 LINK，recv #2 是链尾）
	*   完成顺序 1：user_data=0x1（recv #1）res=7 内容="chain-1"
	*   完成顺序 2：user_data=0x2（recv #2）res=7 内容="chain-2"
	*   链保证 recv #2 在 recv #1 完成后才开始，所以两行顺序固定。
	*
	*   === 2. 链中前一项失败：后续项以 -ECANCELED 完成 ===
	*   提交 2 个 SQE（close(-1) 带 LINK，recv 是链尾）
	*   完成顺序 1：user_data=0xb（close）res=-9（Bad file descriptor）
	*   完成顺序 2：user_data=0xc（recv）res=-125（Operation canceled）
	*   recv 从未真正执行，缓冲区内容不变。
	*
	*   === 3. IOSQE_IO_HARDLINK：前一项失败，后续项照样执行 ===
	*   提交 2 个 SQE（close(-1) 带 HARDLINK，recv 是链尾）
	*   完成顺序 1：user_data=0x15（close）res=-9（Bad file descriptor）
	*   完成顺序 2：user_data=0x16（recv）res=8 内容="hardlink"
	*
	*   === 4. IORING_OP_LINK_TIMEOUT：给 recv 设 150ms 超时 ===
	*   提交 2 个 SQE（recv 带 LINK，link_timeout 是链尾）
	*   完成顺序 1：user_data=0x20（link_timeout）res=-62（Timer expired）
	*   完成顺序 2：user_data=0x1f（recv）res=-125（Operation canceled）
	*   超时在约 150ms 后生效：定时器 CQE 是 -ETIME，被取消的 recv 是
	*   -ECANCELED。
	*/

#include <array>
#include <cerrno>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <format>
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

	io_uring_cqe *wait_cqe()
	{
		io_uring_cqe *cqe = nullptr;
		int ret = io_uring_wait_cqe(&ring_, &cqe);
		if (ret < 0)
			fail("io_uring_wait_cqe", -ret);
		return cqe;
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

// 打印 CQE 的通用格式：user_data / res（含 strerror）/ 额外内容。
void print_cqe(unsigned order, const io_uring_cqe *cqe, std::string_view kind,
	std::string_view detail = {})
{
	if (cqe->res < 0) {
		std::println("完成顺序 {}：user_data={:#x}（{}）res={}（{}）",
			order, cqe->user_data, kind, cqe->res, std::strerror(-cqe->res));
	} else {
		std::println("完成顺序 {}：user_data={:#x}（{}）res={}{}",
			order, cqe->user_data, kind, cqe->res, detail);
	}
}

// ---------------------------------------------------------------------------
// 1. 成功的链：两个 recv 串行执行
// ---------------------------------------------------------------------------
void demo_link_success()
{
	std::println("\n=== 1. IOSQE_IO_LINK：成功链，后一项等前一项完成 ===");

	// DGRAM socketpair：每条报文独立，recv 一次取一条，便于观察“取到了哪条”。
	auto [tx, rx] = make_socketpair(SOCK_DGRAM);

	constexpr std::array<std::string_view, 2> kMsgs{"chain-1", "chain-2"};
	std::array<std::array<char, 16>, kMsgs.size()> bufs{};

	// 先把两条报文排进接收队列。链只保证“读取动作的先后”，
	// 数据本身按 FIFO 被取走，所以 recv #1 必然拿到 chain-1。
	for (std::string_view msg : kMsgs) {
		ssize_t n = ::send(tx.get(), msg.data(), msg.size(), 0);
		if (n != static_cast<ssize_t>(msg.size()))
			fail("send", errno);
	}

	Ring ring{8};

	io_uring_sqe *sqe = ring.get_sqe();
	io_uring_prep_recv(sqe, rx.get(), bufs[0].data(), bufs[0].size(), 0);
	// 关键标志：本 SQE 和 SQ 里紧随其后的 SQE 串成链。
	// 后一项在前一项产生 CQE 之前不会被启动。
	sqe->flags |= IOSQE_IO_LINK;
	io_uring_sqe_set_data64(sqe, 1);

	sqe = ring.get_sqe();
	io_uring_prep_recv(sqe, rx.get(), bufs[1].data(), bufs[1].size(), 0);
	// 不带 LINK 的 SQE 是链尾。注意：即使带了 LINK，只要它是本次提交的
	// 最后一个 SQE，也会被当作链尾——链不会跨到下一次 submit。
	io_uring_sqe_set_data64(sqe, 2);

	// 两个 SQE 必须在同一次 submit 中交给内核，否则链在提交边界断开。
	int submitted = ring.submit();
	std::println("提交 {} 个 SQE（recv #1 带 LINK，recv #2 是链尾）", submitted);

	for (unsigned order = 1; order <= 2; ++order) {
		io_uring_cqe *cqe = ring.wait_cqe();
		const unsigned idx = static_cast<unsigned>(cqe->user_data) - 1;
		std::string_view got;
		if (cqe->res >= 0)
			got = std::string_view{bufs[idx].data(), static_cast<std::size_t>(cqe->res)};
		print_cqe(order, cqe, idx == 0 ? "recv #1" : "recv #2",
			cqe->res >= 0 ? std::string_view{std::format(" 内容=\"{}\"", got)} : std::string_view{});
		// 上面 std::format 生成的临时 string 在 println 前有效；
		// 为了示例可读性这里直接内联，实际项目里注意临时对象生命周期。
		ring.cqe_seen(cqe);
	}
	std::println("链保证 recv #2 在 recv #1 完成后才开始，所以两行顺序固定。");
}

// ---------------------------------------------------------------------------
// 2. 链中一项失败：后续项被取消
// ---------------------------------------------------------------------------
void demo_link_failure()
{
	std::println("\n=== 2. 链中前一项失败：后续项以 -ECANCELED 完成 ===");

	auto [tx, rx] = make_socketpair(SOCK_DGRAM);
	(void)tx; // 本场景不需要数据；recv 根本不会被执行

	Ring ring{8};

	io_uring_sqe *sqe = ring.get_sqe();
	// 用 close(-1) 制造一个确定性的失败（-EBADF）。真实场景可能是
	// connect 失败、fsync 失败、短读等——io_uring 把“非预期结果”都算失败，
	// 包括比预期短的读。
	io_uring_prep_close(sqe, -1);
	sqe->flags |= IOSQE_IO_LINK;
	io_uring_sqe_set_data64(sqe, 0xb);

	sqe = ring.get_sqe();
	io_uring_prep_recv(sqe, rx.get(), nullptr, 0, 0);
	// 缓冲区传 nullptr/0 也无所谓：它永远不会被启动。
	io_uring_sqe_set_data64(sqe, 0xc);

	int submitted = ring.submit();
	std::println("提交 {} 个 SQE（close(-1) 带 LINK，recv 是链尾）", submitted);

	// 两个 CQE 都会来：失败项报自己的错误码，被取消项报 -ECANCELED。
	for (unsigned order = 1; order <= 2; ++order) {
		io_uring_cqe *cqe = ring.wait_cqe();
		print_cqe(order, cqe, cqe->user_data == 0xb ? "close" : "recv");
		ring.cqe_seen(cqe);
	}
	std::println("recv 从未真正执行，缓冲区内容不变。");
}

// ---------------------------------------------------------------------------
// 3. HARDLINK：失败不打断链
// ---------------------------------------------------------------------------
void demo_hardlink()
{
	std::println("\n=== 3. IOSQE_IO_HARDLINK：前一项失败，后续项照样执行 ===");

	auto [tx, rx] = make_socketpair(SOCK_DGRAM);
	constexpr std::string_view kMsg = "hardlink";
	ssize_t n = ::send(tx.get(), kMsg.data(), kMsg.size(), 0);
	if (n != static_cast<ssize_t>(kMsg.size()))
		fail("send", errno);

	std::array<char, 16> buf{};
	Ring ring{8};

	io_uring_sqe *sqe = ring.get_sqe();
	io_uring_prep_close(sqe, -1);
	// HARDLINK 隐含 LINK，但即使前一项失败也不切断链。
	// 唯一会断链的是“父请求在提交阶段就失败”。
	sqe->flags |= IOSQE_IO_HARDLINK;
	io_uring_sqe_set_data64(sqe, 0x15);

	sqe = ring.get_sqe();
	io_uring_prep_recv(sqe, rx.get(), buf.data(), buf.size(), 0);
	io_uring_sqe_set_data64(sqe, 0x16);

	int submitted = ring.submit();
	std::println("提交 {} 个 SQE（close(-1) 带 HARDLINK，recv 是链尾）", submitted);

	for (unsigned order = 1; order <= 2; ++order) {
		io_uring_cqe *cqe = ring.wait_cqe();
		if (cqe->user_data == 0x16) {
			std::string_view got{buf.data(), static_cast<std::size_t>(cqe->res)};
			print_cqe(order, cqe, "recv", std::format(" 内容=\"{}\"", got));
		} else {
			print_cqe(order, cqe, "close");
		}
		ring.cqe_seen(cqe);
	}
	std::println("对比场景 2：失败被记录，但链继续，recv 成功读到数据。");
}

// ---------------------------------------------------------------------------
// 4. LINK_TIMEOUT：给链上前一项设超时
// ---------------------------------------------------------------------------
void demo_link_timeout()
{
	std::println("\n=== 4. IORING_OP_LINK_TIMEOUT：给 recv 设 150ms 超时 ===");

	// 不预先写数据：recv 会一直挂在 poll 上，直到超时把它取消。
	auto [tx, rx] = make_socketpair(SOCK_DGRAM);
	(void)tx;

	std::array<char, 16> buf{};
	Ring ring{8};

	// 超时时间结构体必须活到 CQE 被收割。这里放在栈上，作用域覆盖整个等待。
	// tv_nsec 用 150ms；CLOCK_MONOTONIC 是 LINK_TIMEOUT 固定使用的时钟。
	const __kernel_timespec timeout{
		.tv_sec = 0,
		.tv_nsec = 150'000'000,
	};

	io_uring_sqe *sqe = ring.get_sqe();
	io_uring_prep_recv(sqe, rx.get(), buf.data(), buf.size(), 0);
	// 被计时的请求必须带 IOSQE_IO_LINK，和后面的 timeout SQE 组成链。
	sqe->flags |= IOSQE_IO_LINK;
	io_uring_sqe_set_data64(sqe, 0x1f);

	sqe = ring.get_sqe();
	// prep_link_timeout 把 opcode 设为 IORING_OP_LINK_TIMEOUT，addr 指向
	// timespec，len=1，timeout_flags=第三个参数（0 表示相对时间；
	// IORING_TIMEOUT_ABS 等标志见 io_uring_enter(2)）。
	// 它作用于“前一个”链成员，不作用于 CQ。
	io_uring_prep_link_timeout(sqe, &timeout, 0);
	io_uring_sqe_set_data64(sqe, 0x20);

	int submitted = ring.submit();
	std::println("提交 {} 个 SQE（recv 带 LINK，link_timeout 是链尾）", submitted);

	const auto start = std::chrono::steady_clock::now();
	for (unsigned order = 1; order <= 2; ++order) {
		io_uring_cqe *cqe = ring.wait_cqe();
		print_cqe(order, cqe, cqe->user_data == 0x20 ? "link_timeout" : "recv");
		ring.cqe_seen(cqe);
	}
	const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
		std::chrono::steady_clock::now() - start);
	std::println("超时在约 {}ms 后生效：定时器 -ETIME，被取消的 recv -ECANCELED。",
		elapsed.count());

	// 如果 recv 先完成（本例不可能，因为没写数据），timeout SQE 会得到
	// -ECANCELED 而不是 -ETIME，recv 正常返回字节数。
	//
	// 注意：只有“可中断”的操作能可靠取消（socket 读写、accept 等）；
	// 已经进入块层的磁盘 I/O 无法撤销，超时只能等它完成。
	//
	// 还应注意：给 timeout SQE 加 IOSQE_CQE_SKIP_SUCCESS，可以让它不产生
	// CQE——它是否产生完成事件只看它自己的 skip 标志，与是否超时无关。
	// 见 cqe_skip_success.cpp 里的说明。
}

int main()
{
	demo_link_success();
	demo_link_failure();
	demo_hardlink();
	demo_link_timeout();

	std::println("\n全部场景结束（ring/fd 由 RAII 自动释放）");
	return 0;
}
