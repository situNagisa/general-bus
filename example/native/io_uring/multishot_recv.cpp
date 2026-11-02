/*
接口：io_uring 多 shot recv（IORING_OP_RECV + IORING_RECV_MULTISHOT）
要解决什么：普通一次 recv 提交一次、完成一次，每收一份数据都要重新填 SQE、重新
io_uring_enter；多 shot recv 只提交一次，内核在同一个请求上不断重试，每来一份数据就
向 CQ 投递一个 CQE。适合长连接、每连接只需要挂一个“收数据”请求的服务器。
它的形态：
	一次提交 1 个 SQE；对应 0..N 个 CQE；每个 CQE 对应一次 recv 的系统语义
	（即内核一次 recv 拿到的字节段，不是“一个网络包”，TCP 是字节流，可能合并多次
	write）。第一个数据 CQE 带 IORING_CQE_F_MORE，表示该 SQE 还挂在核心里、还会
	继续产生 CQE；不带 F_MORE 的 CQE 是这个 SQE 的最后一个 CQE（EOF、错误、
	-ENOBUFS、取消等）。
	内存：必须配合 provided buffer ring（缓冲区由应用提供），提交时 SQE 不给缓冲
	地址，只给 buf_group；内核每次 recv 时从环里取一块，完成时把缓冲编号放在
	CQE flags 高 16 位。原因：多 shot 请求在提交时不知道会被执行多少次，无法提
	前绑定一块用户内存。
需要的条件：多 shot recv 需要内核 6.0+，provided buffer ring 需要内核 5.19+
	（liburing man page 明确写了 available since 6.0 / 5.19）。此外多 shot recv
	要求 len=0、必须置 IOSQE_BUFFER_SELECT、不能带 MSG_WAITALL。本机 WSL2
	内核 6.18 实测可用。
编译：
	g++ -std=c++26 -O2 -Wall -Wextra -o multishot_recv multishot_recv.cpp -luring
运行：
	./multishot_recv
预期输出：
	场景 1 每次 write 后等一个 CQE，因此三份数据各产生一个 CQE（res=7/7/5，
	F_MORE=1，bid 依次为 0/1/2），最后 shutdown 产生 res=0 且 F_MORE=0 的 CQE。
	场景 2 只放一块缓冲：第一份数据正常完成（F_MORE=1），第二份数据到达时环里
	没有缓冲，内核投递 res=-ENOBUFS、F_MORE=0 的 CQE，多 shot 请求就此结束；
	把缓冲放回环并重新提交后，之前卡在 socket 里的数据仍能被收走。
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

// 队列深度：SQ/CQ 都是 8 项。这个演示同时最多只有 1 个在途请求，8 绰绰有余。
constexpr unsigned kQueueDepth = 8;
// provided buffer ring 的项数。API 要求是 2 的幂（内部用 entries-1 做掩码）。
constexpr unsigned kBufRingEntries = 4;
// 缓冲区组号（buf_group），提交 recv 时靠它告诉内核“从哪一组取缓冲”。
constexpr int kBufGroupId = 1;
// 每块缓冲的大小，也是内核对一次 recv 能写入的上限。
constexpr std::size_t kBufSize = 128;
// user_data 是应用私有字段，完成时原样回到 CQE，用来区分是哪个请求。
constexpr std::uint64_t kUserData = 0x5245'4356'0001ULL;

// 统一的错误处理：这些演示追求可读性，启动/参数错误直接打印并退出。
[[noreturn]] void fail(std::string_view what)
{
	std::println(stderr, "错误：{}: {}", what, std::strerror(errno));
	std::exit(EXIT_FAILURE);
}

// liburing 和大部分系统调用用“返回负数 = -errno”的约定，这里统一转成 errno 再报错。
void check(int ret, std::string_view what)
{
	if (ret < 0) {
		errno = -ret;
		fail(what);
	}
}

// RAII 管理 fd：析构自动 close，移动后源对象置 -1，避免双关闭。
// 原生代码里 fd 泄漏是最常见的资源问题，示例里用一个小类彻底消除它。
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

// RAII 管理 io_uring 实例。io_uring_queue_init 负责 mmap SQ/CQ 环并注册。
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

	// 取一个空闲 SQE；环满时返回 nullptr（演示里不会发生，仍按错误处理）。
	[[nodiscard]] io_uring_sqe *get_sqe()
	{
		io_uring_sqe *sqe = io_uring_get_sqe(&ring_);
		if (sqe == nullptr) {
			std::println(stderr, "错误：SQ 环已满，无法再取 SQE");
			std::exit(EXIT_FAILURE);
		}
		return sqe;
	}

	// io_uring_submit 内部调用一次 io_uring_enter，返回真正提交的 SQE 数。
	[[nodiscard]] unsigned submit()
	{
		int ret = io_uring_submit(&ring_);
		check(ret, "io_uring_submit");
		return static_cast<unsigned>(ret);
	}

private:
	io_uring ring_{};
};

// RAII 管理 provided buffer ring：注册用内核操作 IORING_REGISTER_PBUF_RING，
// 释放用 IORING_UNREGISTER_PBUF_RING，liburing 把它包成了 setup/free 两个函数。
// io_uring_setup_buf_ring 自己负责环内存的分配（man page：handles the allocation
// of the ring），flags 目前用于 IOU_PBUF_RING_INC（增量消费，6.12+），
// 本示例用 0 表示“一次完成整块消费”。
class BufRing {
public:
	BufRing(Ring &ring, unsigned entries, int bgid)
		: ring_(ring), entries_(entries), bgid_(bgid)
	{
		// 环项数必须是 2 的幂，否则 io_uring_buf_ring_add 的掩码语义不成立。
		if (entries_ == 0 || (entries_ & (entries_ - 1)) != 0) {
			std::println(stderr, "错误：buffer ring 项数必须是 2 的幂");
			std::exit(EXIT_FAILURE);
		}
		int err = 0;
		br_ = io_uring_setup_buf_ring(ring_.get(), entries_, bgid_, 0, &err);
		if (br_ == nullptr) {
			errno = -err;
			fail("io_uring_setup_buf_ring");
		}
		// tail 清零：环里还没有任何可用缓冲。tail 只能通过 liburing 的 helper 写，
		// 它带 release 语义，保证内核看到 tail 时也看到了前面的 buf 项。
		io_uring_buf_ring_init(br_);
	}
	~BufRing()
	{
		if (br_ != nullptr)
			io_uring_free_buf_ring(ring_.get(), br_, entries_, bgid_);
	}

	BufRing(const BufRing &) = delete;
	BufRing &operator=(const BufRing &) = delete;

	[[nodiscard]] io_uring_buf_ring *get() noexcept { return br_; }

	// 往环的 tail 处追加一项：地址、长度、缓冲编号（bid）。
	// mask=entries-1。buf_offset 是“相对当前 tail 的第几项”，连续 add 一批缓冲
	// 时必须传 0,1,2,...；因为 tail 要等 publish 才前进，全传 0 会反复覆盖同一格。
	void add(std::span<char> buf, unsigned short bid, int buf_offset = 0) noexcept
	{
		io_uring_buf_ring_add(br_, buf.data(), static_cast<unsigned>(buf.size()),
			bid, static_cast<int>(entries_ - 1), buf_offset);
	}

	// 发布 count 项：把 tail 前移，内核从此可以取用这些缓冲。
	void publish(unsigned count) noexcept
	{
		io_uring_buf_ring_advance(br_, static_cast<int>(count));
	}

private:
	Ring &ring_;
	io_uring_buf_ring *br_ = nullptr;
	unsigned entries_;
	int bgid_;
};

// 等一个 CQE。io_uring_wait_cqe 会睡眠到至少有一个完成。返回的指针在下一次
// 调 io_uring_cqe_seen 之前一直有效；看到后必须 seen，否则 CQ 环很快填满。
[[nodiscard]] io_uring_cqe *wait_cqe(Ring &ring)
{
	io_uring_cqe *cqe = nullptr;
	check(io_uring_wait_cqe(ring.get(), &cqe), "io_uring_wait_cqe");
	return cqe;
}

// 解码 CQE flags，打印成人类可读的形式。
void print_cqe(const io_uring_cqe *cqe, std::span<char> data = {})
{
	const unsigned flags = cqe->flags;
	const bool more = (flags & IORING_CQE_F_MORE) != 0;
	const bool has_buf = (flags & IORING_CQE_F_BUFFER) != 0;
	const unsigned bid = flags >> IORING_CQE_BUFFER_SHIFT;

	std::print("CQE: res={} flags=0x{:x} F_MORE={} F_BUFFER={}", cqe->res, flags,
		more, has_buf);
	if (has_buf)
		std::print(" bid={}", bid);
	if (cqe->res > 0 && !data.empty())
		std::print(" 数据=\"{}\"",
			std::string_view(data.data(), static_cast<std::size_t>(cqe->res)));
	std::println("");
}

// 场景 1：一次提交，连续收三份数据，最后对端关闭产生“最后一个 CQE”。
// 关键观察：前三个 CQE 带 F_MORE，第四个不带，表示该 SQE 生命结束。
void demo_until_eof()
{
	std::println("--- 场景 1：一次提交，多份数据，直到对端关闭 ---");

	Ring ring(kQueueDepth);
	// 3 份数据各消耗一块缓冲，第 4 块留作余量（EOF 一般不需要缓冲，这里不依赖
	// 这个细节，留一块更稳妥）。
	BufRing br(ring, kBufRingEntries, kBufGroupId);
	std::array<std::array<char, kBufSize>, kBufRingEntries> bufs{};

	// 把所有缓冲都放进环并发布。bid 就是数组下标，便于对照输出。
	for (unsigned i = 0; i < bufs.size(); ++i)
		br.add(bufs[i], static_cast<unsigned short>(i), static_cast<int>(i));
	br.publish(kBufRingEntries);
	std::println("已向 buffer ring（bgid={}）放入 {} 块缓冲", kBufGroupId, kBufRingEntries);

	int sv[2];
	if (::socketpair(AF_UNIX, SOCK_STREAM | SOCK_CLOEXEC, 0, sv) < 0)
		fail("socketpair");
	Fd local(sv[0]); // io_uring 从这里 recv
	Fd peer(sv[1]);	 // 演示程序从这里 write，扮演对端

	// 只构造一个 recv SQE，但它在核心里会被反复执行。
	io_uring_sqe *sqe = ring.get_sqe();
	// buf=nullptr、len=0：多 shot recv 必须用缓冲选择，地址由内核在完成时决定。
	// 这个 helper 内部会把 sqe->ioprio 的 IORING_RECV_MULTISHOT 位置 1，
	// 内核看到它就知道“同一个 SQE 可以反复执行、反复投递 CQE”。
	io_uring_prep_recv_multishot(sqe, local.get(), nullptr, 0, 0);
	// IOSQE_BUFFER_SELECT：告诉内核“这次 recv 从 buffer ring 里选缓冲”。
	sqe->flags |= IOSQE_BUFFER_SELECT;
	// 选哪一组（注册时用的 bgid 必须一致）。
	sqe->buf_group = kBufGroupId;
	sqe->user_data = kUserData;
	std::println("提交 SQE：1 个（多 shot recv），io_uring_submit 返回 {}",
		ring.submit());

	// 每写一份、等一个 CQE，保证每次只有一份数据可读，从而看到多个数据 CQE。
	// 实际网络中多份数据可能被合并进一个 CQE，这是 TCP 字节流的正常行为。
	constexpr std::array<std::string_view, 3> messages{"chunk-1", "chunk-2", "bye"};
	for (std::string_view msg : messages) {
		ssize_t n = ::write(peer.get(), msg.data(), msg.size());
		if (n != static_cast<ssize_t>(msg.size()))
			fail("write");

		io_uring_cqe *cqe = wait_cqe(ring);
		const unsigned bid = cqe->flags >> IORING_CQE_BUFFER_SHIFT;
		if (cqe->res >= 0 && (cqe->flags & IORING_CQE_F_BUFFER) &&
			bid < bufs.size())
			print_cqe(cqe, bufs[bid]);
		else
			print_cqe(cqe);
		// 这块缓冲用完归应用所有；本场景不复用，但一定要 seen 释放 CQE。
		io_uring_cqe_seen(ring.get(), cqe);
	}

	// 对端关闭写方向。多 shot recv 会得到一次 res=0 的完成，然后不再产生 CQE。
	::shutdown(peer.get(), SHUT_WR);
	io_uring_cqe *cqe = wait_cqe(ring);
	print_cqe(cqe);
	if (cqe->res != 0 || (cqe->flags & IORING_CQE_F_MORE))
		std::println(stderr, "错误：预期 res=0 且不带 F_MORE");
	io_uring_cqe_seen(ring.get(), cqe);
	std::println("说明：最后的 res=0 是 orderly shutdown；F_MORE=0 表示该 SQE 已结束。");
}

// 场景 2：缓冲耗尽。多 shot recv 在环里取不到缓冲时投递 -ENOBUFS，
// 并结束该请求；补回缓冲后必须重新提交一个新的多 shot recv。
void demo_enobufs()
{
	std::println("\n--- 场景 2：缓冲耗尽（-ENOBUFS）后补环并重新提交 ---");

	Ring ring(kQueueDepth);
	BufRing br(ring, kBufRingEntries, kBufGroupId);
	std::array<std::array<char, kBufSize>, kBufRingEntries> bufs{};

	// 故意只发布 1 块缓冲，编号 7，和数组其它元素区分开。
	constexpr unsigned short only_bid = 7;
	br.add(bufs[0], only_bid);
	br.publish(1);
	std::println("buffer ring 有 {} 个槽位，但只放了 1 块缓冲（bid={}）",
		kBufRingEntries, only_bid);

	int sv[2];
	if (::socketpair(AF_UNIX, SOCK_STREAM | SOCK_CLOEXEC, 0, sv) < 0)
		fail("socketpair");
	Fd local(sv[0]);
	Fd peer(sv[1]);

	auto submit_multishot = [&] {
		io_uring_sqe *sqe = ring.get_sqe();
		io_uring_prep_recv_multishot(sqe, local.get(), nullptr, 0, 0);
		sqe->flags |= IOSQE_BUFFER_SELECT;
		sqe->buf_group = kBufGroupId;
		sqe->user_data = kUserData;
		if (ring.submit() != 1) {
			std::println(stderr, "错误：io_uring_submit 未提交 1 个 SQE");
			std::exit(EXIT_FAILURE);
		}
	};
	submit_multishot();

	// 第一份数据：有一块缓冲，正常完成，F_MORE=1。
	ssize_t n = ::write(peer.get(), "first", 5);
	if (n != 5)
		fail("write");
	io_uring_cqe *cqe = wait_cqe(ring);
	const unsigned short used_bid =
		static_cast<unsigned short>(cqe->flags >> IORING_CQE_BUFFER_SHIFT);
	print_cqe(cqe, bufs[0]);
	if (cqe->res != 5 || !(cqe->flags & IORING_CQE_F_MORE))
		std::println(stderr, "错误：预期第一份数据完成且带 F_MORE");
	io_uring_cqe_seen(ring.get(), cqe);

	// 第二份数据：环里已经没有缓冲，内核无法 recv，投递 -ENOBUFS，且不再带
	// F_MORE —— 因为多 shot 请求已经终止，不会再有后续 CQE。
	n = ::write(peer.get(), "second", 6);
	if (n != 6)
		fail("write");
	cqe = wait_cqe(ring);
	if (cqe->res < 0)
		std::println("CQE: res={}（{}） flags=0x{:x} F_MORE={}", cqe->res,
			std::strerror(-cqe->res), cqe->flags,
			(cqe->flags & IORING_CQE_F_MORE) != 0);
	else
		print_cqe(cqe);
	if (cqe->res != -ENOBUFS || (cqe->flags & IORING_CQE_F_MORE))
		std::println(stderr, "错误：预期 -ENOBUFS 且不带 F_MORE");
	io_uring_cqe_seen(ring.get(), cqe);

	// 把刚才用完的缓冲放回环并发布，然后重新挂一个多 shot recv。
	// 注意：这时的 bid 仍是第一次完成时内核告诉我们的编号。
	br.add(bufs[0], used_bid);
	br.publish(1);
	submit_multishot();
	std::println("已把 bid={} 放回环并重新提交多 shot recv", used_bid);

	// 关键细节：-ENOBUFS 只说明“这次取不到缓冲”，数据并没有被丢掉，仍留在
	// socket 接收队列里。新请求挂上后不需要再写，之前卡住的 "second" 立刻被收走。
	cqe = wait_cqe(ring);
	{
		const unsigned bid2 = cqe->flags >> IORING_CQE_BUFFER_SHIFT;
		if (cqe->res >= 0 && bid2 == used_bid)
			print_cqe(cqe, bufs[0]);
		else
			print_cqe(cqe);
	}
	io_uring_cqe_seen(ring.get(), cqe);

	// 再补一块缓冲，继续收下一份数据（原请求收到数据后仍带 F_MORE，还活着）。
	br.add(bufs[0], used_bid);
	br.publish(1);
	n = ::write(peer.get(), "third", 5);
	if (n != 5)
		fail("write");
	cqe = wait_cqe(ring);
	{
		const unsigned bid3 = cqe->flags >> IORING_CQE_BUFFER_SHIFT;
		if (cqe->res >= 0 && bid3 == used_bid)
			print_cqe(cqe, bufs[0]);
		else
			print_cqe(cqe);
	}
	io_uring_cqe_seen(ring.get(), cqe);

	// 收尾：补一块缓冲后关闭对端写方向，拿到最后一个 res=0、无 F_MORE 的 CQE。
	br.add(bufs[0], used_bid);
	br.publish(1);
	::shutdown(peer.get(), SHUT_WR);
	cqe = wait_cqe(ring);
	print_cqe(cqe);
	io_uring_cqe_seen(ring.get(), cqe);
}

} // namespace

int main()
{
	demo_until_eof();
	demo_enobufs();
	return 0;
}
