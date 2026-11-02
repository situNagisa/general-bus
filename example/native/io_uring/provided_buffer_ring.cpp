/*
接口：provided buffer ring（预备缓冲区环）
	io_uring_setup_buf_ring / io_uring_buf_ring_add / io_uring_buf_ring_advance，
	配合 recv 的 IOSQE_BUFFER_SELECT + sqe->buf_group 使用。
要解决什么：普通 recv 必须在提交 SQE 时就写死“数据写到哪块内存”。但在高并发
服务器里，收包请求往往先挂起、数据何时来并不知道；应用希望预先交出一批缓冲，让
内核对每个完成事件自己挑一块。provided buffer ring 把“提交”和“选缓冲”解耦：
	提交时：SQE 只写 buf_group，buf=nullptr、len=0；
	完成时：CQE 的 IORING_CQE_F_BUFFER 置位，flags 高 16 位是内核选中的缓冲编号
	（bid）。用完之后应用把这块缓冲重新 add 回环并 advance，内核才能再次使用。
它的形态：
	一次提交 1 个 SQE、完成 1 个 CQE；一次 recv 消费 1 块缓冲（普通模式下整块
	归这次完成使用）；缓冲内存由应用提供，环本身通过 IORING_REGISTER_PBUF_RING
	注册给内核（liburing 的 setup 函数把注册和分配包在一起）。
需要的条件：provided buffer ring 需要内核 5.19+（liburing man page 明确写了
	available since 5.19），并且 recv 支持 IOSQE_BUFFER_SELECT。
	本机 WSL2 内核 6.18 实测可用。
编译：
	g++ -std=c++26 -O2 -Wall -Wextra -o provided_buffer_ring provided_buffer_ring.cpp -luring
运行：
	./provided_buffer_ring
预期输出：
	三次 recv 分别打印内核选中的 bid=0、bid=1、bid=2（环是 FIFO，第一次把 bid=0
	放回环后，它排到队尾，所以下一块自然是 bid=1）。每次提交打印的 SQE 字段里
	buf=nullptr、len=0，证明提交时确实不知道会用哪块内存。
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
// buffer ring 项数必须是 2 的幂，内部用 entries-1 做下标掩码。
constexpr unsigned kBufRingEntries = 4;
constexpr int kBufGroupId = 2;
constexpr std::size_t kBufSize = 64;

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

// provided buffer ring 的 RAII 封装。注册对应内核操作
// IORING_REGISTER_PBUF_RING；liburing 的 io_uring_setup_buf_ring 自己负责
// 分配环内存并完成注册（flags 目前可用 IOU_PBUF_RING_INC 表示增量消费，
// 本示例用 0），io_uring_free_buf_ring 负责注销并释放。
class BufRing {
public:
	BufRing(Ring &ring, unsigned entries, int bgid)
		: ring_(ring), entries_(entries), bgid_(bgid)
	{
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
		// tail=0：环里还没有已发布的缓冲。tail 只能由 liburing helper 修改，
		// 它们带 release 语义，保证内核看到 tail 时能看到完整的 buf 项。
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

	// 在 tail 后第 buf_offset 个槽位写一项。批量 add 时必须传 0,1,2,...：
	// tail 要等 advance 才前进，若全传 0 会反复覆盖同一格（常见坑）。
	void add(std::span<char> buf, unsigned short bid, int buf_offset = 0) noexcept
	{
		io_uring_buf_ring_add(br_, buf.data(), static_cast<unsigned>(buf.size()),
			bid, static_cast<int>(entries_ - 1), buf_offset);
	}

	// 发布 count 项：tail 前移，内核从此可以取用。
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

[[nodiscard]] io_uring_cqe *wait_cqe(Ring &ring)
{
	io_uring_cqe *cqe = nullptr;
	check(io_uring_wait_cqe(ring.get(), &cqe), "io_uring_wait_cqe");
	return cqe;
}

} // namespace

int main()
{
	Ring ring(kQueueDepth);
	BufRing br(ring, kBufRingEntries, kBufGroupId);

	// 真正存放数据的用户内存。内核只持有地址，不会替我们分配这部分。
	// 生命周期必须覆盖整个注册期，绝不能是已经离开作用域的临时数组。
	std::array<std::array<char, kBufSize>, kBufRingEntries> bufs{};

	// 把 4 块缓冲放进环并发布：bid 就用数组下标，便于观察“内核按顺序取”。
	for (unsigned i = 0; i < bufs.size(); ++i)
		br.add(bufs[i], static_cast<unsigned short>(i), static_cast<int>(i));
	br.publish(kBufRingEntries);
	std::println("注册 buffer ring：{} 项，bgid={}，已发布 {} 块缓冲",
		kBufRingEntries, kBufGroupId, kBufRingEntries);

	int sv[2];
	if (::socketpair(AF_UNIX, SOCK_STREAM | SOCK_CLOEXEC, 0, sv) < 0)
		fail("socketpair");
	Fd local(sv[0]);
	Fd peer(sv[1]);

	constexpr std::array<std::string_view, 3> messages{
		"buffer-select-0", "buffer-select-1", "buffer-select-2"};

	for (unsigned round = 0; round < messages.size(); ++round) {
		std::string_view msg = messages[round];
		ssize_t w = ::write(peer.get(), msg.data(), msg.size());
		if (w != static_cast<ssize_t>(msg.size()))
			fail("write");

		io_uring_sqe *sqe = ring.get_sqe();
		// 关键：buf=nullptr、len=0。recv 的缓冲区不在这里指定，而是由内核在
		// 执行这个 SQE 时从 buf_group 对应的 provided buffer ring 里领取。
		io_uring_prep_recv(sqe, local.get(), nullptr, 0, 0);
		// IOSQE_BUFFER_SELECT 打开“缓冲选择”模式；少了它，nullptr 缓冲会直接失败。
		sqe->flags |= IOSQE_BUFFER_SELECT;
		// 告诉内核去哪个 buffer group 取；必须与 setup 时的 bgid 一致。
		sqe->buf_group = kBufGroupId;
		// 应用私有标记，完成时原样返回，用来对应第几次请求。
		sqe->user_data = round;

		std::println("\n第 {} 轮：提交 recv，SQE 中 buf=nullptr len=0 buf_group={}",
			round, kBufGroupId);
		std::println("io_uring_submit 返回 {}（一个 SQE）", ring.submit());

		io_uring_cqe *cqe = wait_cqe(ring);
		const unsigned flags = cqe->flags;
		const bool has_buf = (flags & IORING_CQE_F_BUFFER) != 0;
		// buffer id 藏在 flags 的高 16 位，低 16 位是标志位。
		const unsigned bid = flags >> IORING_CQE_BUFFER_SHIFT;

		std::print("CQE: res={} flags=0x{:x} F_BUFFER={}", cqe->res, flags, has_buf);
		if (has_buf)
			std::print(" 内核选中的 bid={}", bid);
		if (cqe->res > 0 && has_buf && bid < bufs.size())
			std::print(" 数据=\"{}\"",
				std::string_view(bufs[bid].data(), static_cast<std::size_t>(cqe->res)));
		std::println("");

		if (cqe->res < 0 || !has_buf || bid >= bufs.size())
			std::println(stderr, "错误：预期成功且带 IORING_CQE_F_BUFFER");
		else if (bid != round)
			std::println(stderr, "警告：bid 与预期不同（环顺序被其它因素改变）");

		// 内核用完这块缓冲后，所有权回到应用。必须把它重新放回环，否则环会
		// 逐渐变空，之后 recv 会拿到 -ENOBUFS。
		br.add(bufs[bid], static_cast<unsigned short>(bid));
		// io_uring_buf_ring_cq_advance 等价于“发布 count 块缓冲 + cqe seen count 个”，
		// 一次性完成两件事，省掉一次单独的原子操作。这里 count=1。
		io_uring_buf_ring_cq_advance(ring.get(), br.get(), 1);
		std::println("已把 bid={} 放回环尾（cq_advance 同时消费了该 CQE）", bid);
	}

	std::println("\n完成：三次提交各自在完成时才知道用了哪块缓冲，bid=0/1/2。");
	return 0;
}
