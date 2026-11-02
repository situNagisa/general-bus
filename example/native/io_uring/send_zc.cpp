/*
接口：io_uring 零拷贝发送（io_uring_prep_send_zc / IORING_OP_SEND_ZC）
要解决什么：普通 send 在内核返回时，数据已经拷进内核 socket 缓冲，用户缓冲可以
立刻复用；但这一次拷贝在高吞吐下很贵。send_zc 让内核尽量直接引用用户缓冲发送，
代价是“发送完成”和“缓冲可以复用”变成两个不同时刻，必须用两个 CQE 分别通知：
	CQE #1：发送结果（res=实际排入发送路径的字节数），带 IORING_CQE_F_MORE，
		表示同一个 SQE 后面还会有一个 CQE；此时数据已经交给协议栈，但内核/
		NIC 可能仍在读这块用户内存，应用绝不能修改或释放它。
	CQE #2：通知（IORING_CQE_F_NOTIF），表示内核已经不再引用这块缓冲，应用
		可以安全复用/释放。用 IORING_SEND_ZC_REPORT_USAGE 时，这个 CQE 的
		res 还兼作“是否真的零拷贝”的报告：0 表示未报告复制，
		IORING_NOTIF_USAGE_ZC_COPIED 位表示至少部分数据被复制（本机头文件
		说明这个 res 应按标志位解读，其它位保留）。
它的形态：
	一次提交 1 个 SQE，完成 2 个 CQE；对应一个 TCP 字节段的发送（TCP 没有消息
	边界，两个 CQE 不是两个包）；缓冲由应用提供，应用负责在 NOTIF 之前保持其
	内容不变、生命周期有效。
需要的条件：内核支持 IORING_OP_SEND_ZC（约 6.0 起，具体版本未核对）。
	真正“零拷贝”还要求网卡/驱动支持 tx zerocopy；回环或不支持时内核会退回复制，
	但两个 CQE 的语义不变。本机 WSL2 内核 6.18 实测可用。
编译：
	g++ -std=c++26 -O2 -Wall -Wextra -o send_zc send_zc.cpp -luring
运行：
	./send_zc
预期输出：
	每次 send_zc 打印两个 CQE：第一个 flags=0x2（F_MORE）、res=20；第二个
	flags=0x8（F_NOTIF）、res=0（REPORT_USAGE 报告未复制）。两次发送复用同一块
	缓冲，接收端最后收到 40 字节的两条消息。
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

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

namespace {

constexpr unsigned kQueueDepth = 8;
// 用户发送缓冲。send_zc 期间内核可能一直引用它，所以不能是临时对象。
constexpr std::size_t kSendBufSize = 64;

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

// 从 fd 读满 out.size() 字节。这里只在所有 send_zc 都完成后调用，数据已经在
// 对端接收队列里，循环读不会永久阻塞。
void read_exact(int fd, std::span<char> out)
{
	std::size_t off = 0;
	while (off < out.size()) {
		ssize_t n = ::read(fd, out.data() + off, out.size() - off);
		if (n < 0)
			fail("read");
		if (n == 0) {
			std::println(stderr, "错误：对端提前关闭");
			std::exit(EXIT_FAILURE);
		}
		off += static_cast<std::size_t>(n);
	}
}

// 提交一次 send_zc，并分别等待“发送结果”和“缓冲可复用通知”两个 CQE。
void send_zc_and_wait(Ring &ring, int fd, std::span<const char> payload,
	std::uint64_t user_data)
{
	io_uring_sqe *sqe = ring.get_sqe();
	// 这个 helper 把 opcode 设为 IORING_OP_SEND_ZC，并让 sqe->msg_flags=flags、
	// sqe->ioprio=zc_flags。flags 是 send(2) 的 flags（这里 0）；zc_flags 是
	// io_uring 私有的发送选项，IORING_SEND_ZC_REPORT_USAGE 让通知 CQE 的 res
	// 报告是否发生了复制。
	io_uring_prep_send_zc(sqe, fd, payload.data(), payload.size(), 0,
		IORING_SEND_ZC_REPORT_USAGE);
	// user_data 会出现在两个 CQE 上，用它可以确认它们属于同一个请求。
	sqe->user_data = user_data;

	std::println("\n提交 IORING_OP_SEND_ZC：len={} user_data=0x{:x}",
		payload.size(), user_data);
	if (ring.submit() != 1) {
		std::println(stderr, "错误：io_uring_submit 未提交 1 个 SQE");
		std::exit(EXIT_FAILURE);
	}
	std::println("一个 SQE 已提交，接下来应收到两个 CQE。");

	// ---- 第一个 CQE：发送结果 ----
	io_uring_cqe *cqe = wait_cqe(ring);
	const unsigned first_flags = cqe->flags;
	const bool more = (first_flags & IORING_CQE_F_MORE) != 0;
	const bool first_notif = (first_flags & IORING_CQE_F_NOTIF) != 0;
	std::println("CQE#1: user_data=0x{:x} res={} flags=0x{:x} F_MORE={} F_NOTIF={}",
		cqe->user_data, cqe->res, first_flags, more, first_notif);
	if (cqe->res < 0)
		std::println(stderr, "含义：发送失败（{}），通常不会再有通知",
			std::strerror(-cqe->res));
	else
		std::println("含义：{} 字节已进入发送路径；但缓冲仍可能被内核引用，"
			"不能修改。", cqe->res);
	// F_MORE 在这里不是“还有数据”，而是“同一个 SQE 还会再产生一个 CQE”。
	if (!more) {
		std::println(stderr,
			"注意：没有 F_MORE，说明不会再有通知 CQE（通常是立即失败）");
	}
	io_uring_cqe_seen(ring.get(), cqe);
	if (!more)
		return;

	// ---- 第二个 CQE：通知，缓冲可以复用 ----
	cqe = wait_cqe(ring);
	const unsigned second_flags = cqe->flags;
	const bool notif = (second_flags & IORING_CQE_F_NOTIF) != 0;
	std::println("CQE#2: user_data=0x{:x} res=0x{:x} flags=0x{:x} F_MORE={} F_NOTIF={}",
		cqe->user_data, cqe->res, second_flags,
		(second_flags & IORING_CQE_F_MORE) != 0, notif);
	if (!notif)
		std::println(stderr, "错误：第二个 CQE 未带 IORING_CQE_F_NOTIF");
	// REPORT_USAGE 下 res 不是普通错误码，而是按标志位解读：0 表示未报告复制，
	// 最高位 IORING_NOTIF_USAGE_ZC_COPIED 表示至少部分复制。
	if (cqe->res == 0)
		std::println("含义：内核未报告发生复制，缓冲现在可以安全复用。");
	else if (cqe->res & IORING_NOTIF_USAGE_ZC_COPIED)
		std::println("含义：数据至少被部分复制（回环/网卡不支持零拷贝时会这样），"
			"缓冲仍可以安全复用。");
	else
		std::println("含义：通知已到，res=0x{:x} 为其它报告值，缓冲可复用。",
			cqe->res);
	io_uring_cqe_seen(ring.get(), cqe);
}

} // namespace

int main()
{
	// TCP 回环：一个进程里同时扮演服务端和客户端，自产自销。
	Fd listener(::socket(AF_INET, SOCK_STREAM | SOCK_CLOEXEC, 0));
	if (listener.get() < 0)
		fail("socket(listener)");

	sockaddr_in addr{};
	addr.sin_family = AF_INET;
	addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK); // 127.0.0.1
	addr.sin_port = 0;			       // 让内核选临时端口
	if (::bind(listener.get(), reinterpret_cast<const sockaddr *>(&addr),
			sizeof(addr)) < 0)
		fail("bind");
	if (::listen(listener.get(), 1) < 0)
		fail("listen");

	socklen_t addr_len = sizeof(addr);
	if (::getsockname(listener.get(), reinterpret_cast<sockaddr *>(&addr),
			&addr_len) < 0)
		fail("getsockname");
	std::println("监听 127.0.0.1:{}（端口由内核分配）", ntohs(addr.sin_port));

	Fd client(::socket(AF_INET, SOCK_STREAM | SOCK_CLOEXEC, 0));
	if (client.get() < 0)
		fail("socket(client)");
	if (::connect(client.get(), reinterpret_cast<const sockaddr *>(&addr),
			sizeof(addr)) < 0)
		fail("connect");
	Fd peer(::accept(listener.get(), nullptr, nullptr));
	if (peer.get() < 0)
		fail("accept");
	std::println("连接建立：发送端 fd={}，接收端 fd={}", client.get(), peer.get());
	// 监听 socket 已经没用了，RAII 会在 main 结束时关闭。

	Ring ring(kQueueDepth);

	// 同一块用户缓冲被两次 send_zc 复用：第二次必须等第一次的 NOTIF。
	std::array<char, kSendBufSize> send_buf{};

	constexpr std::string_view msg1 = "zero-copy message #1";
	constexpr std::string_view msg2 = "zero-copy message #2";
	static_assert(msg1.size() + msg2.size() <= kSendBufSize);

	std::memcpy(send_buf.data(), msg1.data(), msg1.size());
	std::println("\n第一次发送：把 msg1 拷进缓冲，提交 send_zc。");
	send_zc_and_wait(ring, client.get(), std::span<const char>(send_buf).first(msg1.size()),
		0x1001);

	// 走到这里说明已经收到 NOTIF，内核不再引用 send_buf，可以安全改写。
	// 如果在上面的第一个 CQE 之后、NOTIF 之前改写，就是典型 use-after-free
	// 语义错误（数据可能被发成半新半旧，且内核 UAF 风险）。
	std::memcpy(send_buf.data(), msg2.data(), msg2.size());
	std::println("\n收到 NOTIF 后改写同一块缓冲为 msg2，再次提交 send_zc。");
	send_zc_and_wait(ring, client.get(), std::span<const char>(send_buf).first(msg2.size()),
		0x1002);

	// 两次发送都完成且进入接收队列，现在把数据读出来核对。
	std::array<char, kSendBufSize> recv_buf{};
	read_exact(peer.get(), std::span<char>(recv_buf).first(msg1.size() + msg2.size()));
	std::println("\n接收端收到 {} 字节：\"{}\"", msg1.size() + msg2.size(),
		std::string_view(recv_buf.data(), msg1.size() + msg2.size()));

	return 0;
}
