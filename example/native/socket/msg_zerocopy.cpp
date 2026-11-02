/*
	msg_zerocopy.cpp —— SO_ZEROCOPY + send(MSG_ZEROCOPY) + 错误队列完成通知

	一、这是什么
	普通 send() 把用户缓冲复制进内核 socket 缓冲；大块数据（>10KB 左右）
	时这次复制很贵。SO_ZEROCOPY 打开后，带 MSG_ZEROCOPY 的 send 改为让
	内核“pin 住”用户页、直接引用它们组包，省掉复制。代价是：内核必须在
	真正不再引用这些页时通知你，通知从 socket 的“错误队列”读，格式是
	struct sock_extended_err。

	二、最关键的一件事：send 返回 ≠ 缓冲可复用
	send(MSG_ZEROCOPY) 返回只表示“这段内存已经交给内核”，内核仍然持有
	这些页的引用，可能正在 DMA / 排队 / 重传。此时改写这段内存会改坏
	在途数据（你自己的数据流会损坏，内核不会崩）。
	只有从错误队列读到覆盖该 send 调用的完成通知后，才能安全复用。
	完成通知也不等于“对端收到”，它只表示内核放开了这些页。

	  send() 返回           完成通知到达
	      |                      |
	      v                      v
	  buf 不能改  --------->  buf 可以复用（也可能可以更早，但不可依赖）

	三、形态
	  - 一次 send 调用交 1 段用户内存（sendmsg 可带 iovec 数组，多段时的
	    通知粒度本示例未涉及，未核对）。
	  - 线上单元：TCP 按 MSS 切成多个 skb；UDP 按 MTU 分片，一个数据报
	    可能对应多个 skb。完成通知与线上单元无关。
	  - 提交：每次成功的 send 调用在 socket 内部计数器上 +1（按调用次数，
	    不按字节数，见内核文档 Documentation/networking/msg_zerocopy.rst）。
	  - 完成：内核把连续完成的调用合并成一条通知，用 [ee_info, ee_data]
	    这个闭区间表示一批 send 调用；所以“提交 N 次”可能只收到几条通知。
	  - 内存：由调用者提供，内核只 pin 不复制，直到通知到达才放开。
	  - 失败：send 返回 -1/ENOBUFS 表示超过 socket 的 optmem 限制或进程的
	    RLIMIT_MEMLOCK；不是“零拷贝不可用”，而是这次没排上。

	四、需要的条件
	  - 内核 >= 4.14（SO_ZEROCOPY 与 MSG_ZEROCOPY 都是 4.14 引入）。
	  - 仅 AF_INET / AF_INET6 的 SOCK_STREAM / SOCK_DGRAM（AF_VSOCK 的
	    virtio 传输也支持）。AF_UNIX 上 setsockopt(SO_ZEROCOPY) 会失败。
	  - 必须先 setsockopt(SO_ZEROCOPY, 1) 表明意图；否则内核静默忽略
	    MSG_ZEROCOPY（避免老程序误传标志就改变语义）。
	  - 读错误队列：TCP 直接可用；UDP 需要额外打开 IP_RECVERR（man ip(7)，
	    本机没有 man page，未核对）。
	  - 回环上的特殊行为：内核文档明确写了，MSG_ZEROCOPY 发往本机 socket
	    的包一律会做一次“延迟拷贝”（否则接收方不读时页引用永远不释放），
	    并在 ee_code 里置 SO_EE_CODE_ZEROCOPY_COPIED。所以本示例在回环上
	    运行时看到“内核其实做了拷贝”是预期结果，不是失败。

	五、编译与运行
	  g++ -std=c++26 -O2 -Wall -Wextra -o /tmp/native_msg_zerocopy msg_zerocopy.cpp
	  /tmp/native_msg_zerocopy
	  不需要额外链接库（std::thread 没用；只用同步 API）。

	六、预期输出（回环）
	  == SO_ZEROCOPY 演示：16 次 64 KiB 的 send(MSG_ZEROCOPY) ==
	  ...
	  send() 全部返回（1 MiB 已交给内核），此刻 buf 还不能改写
	  接收端收满 1048576 字节，校验和一致
	  完成通知 [1, 16]：覆盖 16 次 send
	  含 SO_EE_CODE_ZEROCOPY_COPIED：回环上内核退化成了拷贝（预期）
	  收到全部完成通知，现在才允许改写 buf
*/

#include <arpa/inet.h>
#include <fcntl.h>
#include <linux/errqueue.h>
#include <netinet/in.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <cerrno>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <optional>
#include <span>
#include <utility>
#include <vector>

namespace {

/*
	fd 的 RAII 包装：析构 close，禁止拷贝，支持移动。
	错误队列、监听 socket、连接 socket 都用它管理。
*/
class Fd {
public:
	explicit Fd(int fd = -1) noexcept : fd_(fd) {}
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
	~Fd() { reset(); }

	[[nodiscard]] int get() const noexcept { return fd_; }
	explicit operator bool() const noexcept { return fd_ >= 0; }

private:
	void reset() noexcept
	{
		if (fd_ >= 0)
			::close(fd_);
		fd_ = -1;
	}

	int fd_;
};

[[noreturn]] void die(const char *what)
{
	std::fprintf(stderr, "错误：%s: %s\n", what, std::strerror(errno));
	std::exit(EXIT_FAILURE);
}

void check(bool ok, const char *what)
{
	if (!ok)
		die(what);
}

void check(const Fd &fd, const char *what)
{
	if (!fd)
		die(what);
}

constexpr std::size_t kChunk = 64 * 1024;	// 每块 64 KiB：远大于 10KB 的门槛
constexpr std::size_t kCount = 16;		// 16 次 send，共 1 MiB
constexpr std::size_t kTotalBytes = kChunk * kCount;

/*
	FNV-1a 64 位，用来校验接收端拿到的字节和发送前完全一致。
*/
std::uint64_t fnv1a(std::span<const std::byte> data, std::uint64_t h = 1469598103934665603ull)
{
	for (const std::byte b : data) {
		h ^= static_cast<std::uint8_t>(b);
		h *= 1099511628211ull;
	}
	return h;
}

/*
	建只监听回环的 TCP 监听器；端口让内核选。
*/
Fd make_tcp_listener(sockaddr_in &addr)
{
	Fd listener{::socket(AF_INET, SOCK_STREAM | SOCK_CLOEXEC, 0)};
	check(listener, "socket(listener)");

	const int one = 1;
	check(::setsockopt(listener.get(), SOL_SOCKET, SO_REUSEADDR, &one, sizeof one) == 0,
		"setsockopt(SO_REUSEADDR)");

	addr = {};
	addr.sin_family = AF_INET;
	addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
	addr.sin_port = 0;
	check(::bind(listener.get(), reinterpret_cast<sockaddr *>(&addr), sizeof addr) == 0, "bind");

	socklen_t len = sizeof addr;
	check(::getsockname(listener.get(), reinterpret_cast<sockaddr *>(&addr), &len) == 0,
		"getsockname");
	check(::listen(listener.get(), 1) == 0, "listen");
	return listener;
}

/*
	从 socket 的错误队列读一条 zerocopy 完成通知。
	返回 std::nullopt 表示当前没有通知（EAGAIN）。
*/
struct Completion {
	bool copied;		// ee_code 里有没有 SO_EE_CODE_ZEROCOPY_COPIED
	std::uint32_t lo;	// ee_info：这批完成的最小“send 序号”
	std::uint32_t hi;	// ee_data：最大序号（闭区间，含 hi）
};

std::optional<Completion> read_completion(int fd)
{
	// 错误队列里的报文本身可能带原始数据（ICMP 错误等），这里给一个小缓冲
	// 把它接住即可；真正的信息在控制消息里。
	std::array<std::byte, 64> payload{};
	iovec iov{payload.data(), payload.size()};

	// 控制消息要装下 cmsghdr + sock_extended_err，512 字节绰绰有余。
	std::array<std::byte, 512> control{};

	msghdr msg{};
	msg.msg_iov = &iov;
	msg.msg_iovlen = 1;
	msg.msg_control = control.data();
	msg.msg_controllen = control.size();

	// MSG_ERRQUEUE：从错误队列取消息，而不是从接收队列。
	// 读错误队列永远是非阻塞的，没有消息时返回 EAGAIN。
	const ssize_t ret = ::recvmsg(fd, &msg, MSG_ERRQUEUE);
	if (ret < 0) {
		if (errno == EAGAIN || errno == EWOULDBLOCK)
			return std::nullopt;
		if (errno == EINTR)
			return std::nullopt;
		die("recvmsg(MSG_ERRQUEUE)");
	}

	// 控制消息可能有多条（例如同时打开了时间戳），逐条找 zerocopy 的。
	for (cmsghdr *cm = CMSG_FIRSTHDR(&msg); cm != nullptr; cm = CMSG_NXTHDR(&msg, cm)) {
		// IPv4 上是 SOL_IP + IP_RECVERR；IPv6 上是 SOL_IPV6 + IPV6_RECVERR。
		const bool v4 = cm->cmsg_level == SOL_IP && cm->cmsg_type == IP_RECVERR;
		const bool v6 = cm->cmsg_level == SOL_IPV6 && cm->cmsg_type == IPV6_RECVERR;
		if (!v4 && !v6)
			continue;
		if (cm->cmsg_len < CMSG_LEN(sizeof(sock_extended_err))) {
			std::fputs("控制消息长度不足以容纳 sock_extended_err\n", stderr);
			continue;
		}

		sock_extended_err err{};
		std::memcpy(&err, CMSG_DATA(cm), sizeof err);

		// ee_errno == 0：不是真错误。内核文档特意把它设成 0，避免这
		// 条通知阻塞正常的 send/recv（有错误的错误队列会使操作返回错误）。
		// ee_origin：这条错误消息的来源，SO_EE_ORIGIN_ZEROCOPY 表示是
		// zerocopy 完成通知，不是 ICMP 之类的网络错误。
		if (err.ee_errno != 0 || err.ee_origin != SO_EE_ORIGIN_ZEROCOPY)
			continue;

		// ee_info / ee_data：完成通知覆盖的 send 调用序号闭区间。
		// 计数器按“成功的 MSG_ZEROCOPY send 调用”递增，与字节数无关，
		// 32 位会回绕。
		// ee_code：SO_EE_CODE_ZEROCOPY_COPIED 表示内核最终没能沿用用户页，
		// 而是做了一次（延迟）拷贝来收尾。
		return Completion{
			.copied = (err.ee_code & SO_EE_CODE_ZEROCOPY_COPIED) != 0,
			.lo = err.ee_info,
			.hi = err.ee_data,
		};
	}
	return std::nullopt;
}

/*
	阻塞等到错误队列里有东西。按内核文档：读错误队列是非阻塞的，
	要先 poll；错误事件会无条件报告，不需要在 events 里写 POLLERR。
*/
bool wait_error_queue(int fd, int timeout_ms)
{
	pollfd pfd{};
	pfd.fd = fd;
	pfd.events = 0;
	const int r = ::poll(&pfd, 1, timeout_ms);
	if (r < 0) {
		if (errno == EINTR)
			return false;
		die("poll");
	}
	return r > 0 && (pfd.revents & POLLERR);
}

/*
	把接收端已经到达的数据都读走，更新计数与校验和；EAGAIN 就返回。
	conn 设成非阻塞，这样“顺手排空”不会卡住发送循环。
*/
void drain_some(int fd, std::span<std::byte> scratch, std::size_t &received, std::uint64_t &hash)
{
	for (;;) {
		const ssize_t n = ::recv(fd, scratch.data(), scratch.size(), MSG_DONTWAIT);
		if (n < 0) {
			if (errno == EINTR)
				continue;
			if (errno == EAGAIN || errno == EWOULDBLOCK)
				return;
			die("recv");
		}
		if (n == 0) {
			std::fputs("接收端提前看到 EOF\n", stderr);
			std::exit(EXIT_FAILURE);
		}
		const auto got = static_cast<std::size_t>(n);
		hash = fnv1a({scratch.data(), got}, hash);
		received += got;
	}
}

void set_nonblocking(int fd)
{
	const int flags = ::fcntl(fd, F_GETFL, 0);
	check(flags >= 0, "fcntl(F_GETFL)");
	check(::fcntl(fd, F_SETFL, flags | O_NONBLOCK) == 0, "fcntl(F_SETFL, O_NONBLOCK)");
}

} // namespace

int main()
{
	std::puts("== SO_ZEROCOPY 演示：16 次 64 KiB 的 send(MSG_ZEROCOPY) ==");

	// ---- 1. 建一条 TCP 回环连接：client 发送，conn 接收 ----
	sockaddr_in addr{};
	Fd listener = make_tcp_listener(addr);

	Fd client{::socket(AF_INET, SOCK_STREAM | SOCK_CLOEXEC, 0)};
	check(client, "socket(client)");
	check(::connect(client.get(), reinterpret_cast<sockaddr *>(&addr), sizeof addr) == 0, "connect");
	Fd conn{::accept4(listener.get(), nullptr, nullptr, SOCK_CLOEXEC)};
	check(conn, "accept4");

	// ---- 2. 在发送 socket 上声明“我要用零拷贝” ----
	// 不设这一项，MSG_ZEROCOPY 会被内核静默忽略（send 照常工作、照常拷贝），
	// 程序很难察觉。optval 是 int，值非 0 即开启。
	const int one = 1;
	if (::setsockopt(client.get(), SOL_SOCKET, SO_ZEROCOPY, &one, sizeof one) != 0) {
		std::fprintf(stderr, "setsockopt(SO_ZEROCOPY) 失败：%s\n", std::strerror(errno));
		std::fputs("（内核 < 4.14、或 socket 类型不支持；AF_UNIX 不支持）\n", stderr);
		return EXIT_FAILURE;
	}

	// 接收端设非阻塞，便于在发送间隙“顺手”排空，避免发送缓冲被填满。
	set_nonblocking(conn.get());

	// ---- 3. 准备数据。发送前填好，之后直到完成通知到来前都不许改 ----
	std::vector<std::byte> arena(kTotalBytes);
	for (std::size_t i = 0; i < arena.size(); ++i)
		arena[i] = static_cast<std::byte>((i * 131 + 7) & 0xff);
	const std::span<std::byte> all{arena};
	const std::uint64_t expected_hash = fnv1a(all);

	std::array<std::byte, 64 * 1024> scratch{};
	std::size_t received = 0;
	std::uint64_t recv_hash = 1469598103934665603ull;

	// ---- 4. 逐块发送，每次 send 都带 MSG_ZEROCOPY ----
	std::size_t sends = 0;	// 成功的 send 调用次数 = 预期完成数（未合并时）
	const auto t0 = std::chrono::steady_clock::now();

	for (std::size_t i = 0; i < kCount; ++i) {
		const std::span<const std::byte> chunk = all.subspan(i * kChunk, kChunk);

		// send 可能只接受一部分；每个成功的（哪怕部分的）send 调用都算
		// 一次提交、各占一个完成序号，所以循环到整块发完。
		std::size_t off = 0;
		while (off < chunk.size()) {
			const ssize_t n = ::send(client.get(), chunk.data() + off, chunk.size() - off,
				MSG_ZEROCOPY | MSG_NOSIGNAL);
			if (n < 0) {
				if (errno == EINTR)
					continue;
				if (errno == ENOBUFS) {
					// 超过 socket optmem 或 RLIMIT_MEMLOCK。真实程序应当
					// 退避/重试，或这次改用普通 send。
					std::fprintf(stderr, "send(MSG_ZEROCOPY) 返回 ENOBUFS："
						"optmem 或 RLIMIT_MEMLOCK 不够\n");
					return EXIT_FAILURE;
				}
				die("send(MSG_ZEROCOPY)");
			}
			off += static_cast<std::size_t>(n);
			++sends;
		}

		// 发送间隙排空接收端：回环上不排空会把发送缓冲填满，send 阻塞。
		drain_some(conn.get(), scratch, received, recv_hash);
	}

	const auto t_sent = std::chrono::steady_clock::now();
	std::printf("send() 全部返回：%zu 次调用、%zu 字节已交给内核，耗时 %lld µs\n", sends,
		kTotalBytes,
		static_cast<long long>(
			std::chrono::duration_cast<std::chrono::microseconds>(t_sent - t0).count()));
	std::puts("此刻 buf 仍不能改写：内核可能还引用着这些页");

	// ---- 5. 把接收端剩下的数据收完（保证连接健康、数据真的到过对端） ----
	while (received < kTotalBytes) {
		drain_some(conn.get(), scratch, received, recv_hash);
		if (received >= kTotalBytes)
			break;
		pollfd pfd{conn.get(), POLLIN, 0};
		const int r = ::poll(&pfd, 1, 2000);
		if (r <= 0) {
			std::fprintf(stderr, "等待接收数据超时，已收 %zu / %zu\n", received, kTotalBytes);
			return EXIT_FAILURE;
		}
	}
	std::printf("接收端收满 %zu 字节，校验和%s\n", received,
		recv_hash == expected_hash ? "一致" : "不一致（数据被破坏了！）");
	check(recv_hash == expected_hash, "接收数据校验");

	// ---- 6. 读错误队列，等所有 send 的完成通知 ----
	// 关键：完成通知只说明页可以被复用，不说明对端收到；它甚至可能在对端
	// 收完之前就到（尤其是“延迟拷贝”的情况）。
	std::size_t completed = 0;	// 按区间长度累加，应等于 sends
	std::size_t notifications = 0;
	bool saw_copied = false;
	bool have_next = false;
	std::uint32_t next_id = 0;
	bool gap = false;

	while (completed < sends) {
		if (const auto c = read_completion(client.get())) {
			++notifications;
			// 闭区间 [lo, hi]，长度是 hi - lo + 1。
			const std::size_t range = static_cast<std::size_t>(c->hi - c->lo) + 1;
			completed += range;
			saw_copied = saw_copied || c->copied;

			// 检查区间是否连续：理论上按序完成时应首尾相接。重传/关闭时
			// 可能乱序或出现空洞（内核文档），这里只报告不视为致命错误。
			if (!have_next) {
				have_next = true;
			} else if (c->lo != next_id) {
				gap = true;
			}
			next_id = c->hi + 1;

			std::printf("完成通知 [%u, %u]：覆盖 %zu 次 send%s%s\n", c->lo, c->hi, range,
				c->copied ? "，内核做了拷贝（COPIED）" : "",
				gap ? "，与上一条不连续" : "");
			continue;
		}
		// 没有就等到有。等待期间错误队列可能被合并更新，不影响循环。
		if (!wait_error_queue(client.get(), 2000)) {
			std::fprintf(stderr, "等待完成通知超时：已覆盖 %zu / %zu 次 send\n", completed,
				sends);
			return EXIT_FAILURE;
		}
	}
	const auto t_done = std::chrono::steady_clock::now();

	std::printf("共 %zu 条通知覆盖 %zu 次 send；%s\n", notifications, completed,
		saw_copied ? "出现 SO_EE_CODE_ZEROCOPY_COPIED（回环上必然如此）"
			   : "全部按零拷贝完成");
	std::printf("从 send 返回到完成通知到齐：%lld µs\n",
		static_cast<long long>(
			std::chrono::duration_cast<std::chrono::microseconds>(t_done - t_sent).count()));

	// ---- 7. 到这里才允许改写刚才发出去的缓冲 ----
	// 真实程序会在这里把 buf 回收进内存池、写入下一批数据、或者复用给
	// 下一次 send(MSG_ZEROCOPY)。提前改写会造成“自己的数据流被改坏”。
	std::fill(all.begin(), all.end(), std::byte{0});
	std::puts("收到全部完成通知，现在才允许改写 buf（已清零）");

	/*
		常见坑：
		1. 忘记 setsockopt(SO_ZEROCOPY)：内核不报错，悄悄退回拷贝，
		   程序以为在零拷贝。
		2. 把 send 返回当成可复用：产生偶发的“数据错乱”，且只在网卡
		   忙/重传时才出现，极难查。
		3. 只读一次错误队列：通知会被合并、会乱序，也可能在对端关闭时
		   丢失；正确做法是按区间累计，直到覆盖所有成功的 send。关闭
		   socket 会丢弃未读通知，所以复用 buf 前必须先读完。
		4. 小块数据用零拷贝往往更慢：pin 页、错误队列通知的开销按“次”
		   计，10KB 以下通常不如直接拷贝。
		5. 回环/本机 socket 一定会退化成延迟拷贝，这是内核文档写明的；
		   想观察真正的零拷贝要跨机器，或换支持 SG 的网卡。
		6. send 返回 ENOBUFS 是资源限制，不是参数错误；需要重试或回退。
	*/
	return 0;
}
