/*
	msg_peek.cpp —— recv(2) 的 MSG_PEEK 标志

	一、这是什么
	MSG_PEEK 让 recv()/recvfrom()/recvmsg() 只“看一眼”接收队列里的数据，
	把数据复制到用户缓冲，但不把数据从队列里摘掉。下一次不带 MSG_PEEK 的
	recv() 仍然会读到同一段数据。它本身不是“零拷贝”接口，恰恰相反：
	每 peek 一次就多复制一次。

	二、解决什么问题 / 什么时候用
	典型场景是“先看头、再决定收多少”：
	- 自定义协议前面是固定长度的长度头，后面是变长负载。先 peek 4 字节
	长度头，算出整帧长度，再一次性把整帧收下来，避免“先读头再读体”
	带来的两次系统调用与两次拷贝。
	- UDP/TCP 上判断“下一个数据报/下一段数据是不是我要的类型”，不是就
	留给别人处理。
	- 调试时观察接收队列，而不破坏它。

	三、形态
	- 一次调用交给内核 1 段用户内存（recvmsg 可以给多个 iovec，但本示例
	只用 recv）；内核把接收队列头部的数据复制进这段内存，仅此而已。
	- 对应 0 个线上单元：不发包、不回 ACK。字节流上“队列头部”是一个
	位置；数据报上“队列头部”是一个完整数据报。
	- 提交 0 次、完成 0 次：它是同步调用，返回时复制已经做完。
	- 内存由调用者提供；内核只读，不改队列。
	- 返回值：实际复制到用户缓冲的字节数；0 表示对端已关闭（仅流式）；
	-1 表示出错。对 SOCK_STREAM，若队列里有 1..n 字节，最多返回 n；
	count 再大也不会等，除非队列为空（阻塞 socket 会睡眠）。
	- 对 SOCK_DGRAM，一次 peek 永远只对应一个数据报，缓冲不够就截断，
	没被复制的那部分在本次 peek 中看不到（加 MSG_TRUNC 可以拿回真实
	长度）。

	四、内核 / 平台条件
	- Linux 及绝大多数 Unix。MSG_PEEK 自 BSD 时代就有，值 0x02
	（/usr/include/bits/socket.h）。
	- 本示例只用回环地址（127.0.0.1）和本机内核，无特殊权限要求。

	五、编译与运行
	g++ -std=c++26 -O2 -Wall -Wextra -o /tmp/native_msg_peek msg_peek.cpp
	/tmp/native_msg_peek
	不需要额外链接库。

	六、预期输出（数值可能因内核缓冲策略略有出入，逻辑不变）
	== 1. TCP 回环：peek 4 字节长度头，再按长度正式收 ==
	第 1 次 peek 返回 4，长度头解析出 payload = 56 字节
	peek 之后 FIONREAD = 4 —— 数据没被取走
	整帧到齐后，peek 64 字节返回 60（4 字节头 + 56 字节体）
	正式 recv 头 4 字节后 FIONREAD = 56
	整帧内容校验通过：...
	== 2. UDP 回环：peek 一个数据报后它仍在队列里 ==
	第 1 次 peek：返回 5，内容 "alpha"
	此刻 FIONREAD = 5（队头仍是 alpha）
	第 2 次 peek：返回 5，内容 "alpha"（仍是同一个数据报）
	正式 recv："alpha"（之后 FIONREAD = 4）
	再 recv："beta" —— 第二个数据报才被取走
	用 10 字节缓冲 peek 100 字节数据报：返回 10（被截断）
	同一数据报再加 MSG_TRUNC：返回 100 = 真实长度
	正式 recv 仍然拿到完整的 100 字节
*/

#include <arpa/inet.h>
#include <netinet/in.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <array>
#include <cerrno>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <span>
#include <string_view>
#include <utility>
#include <vector>

namespace {

/*
	最小的 fd RAII 包装：析构时 close，禁止拷贝，允许移动。
	示例里所有 fd 都经过它，异常/提前 return 时不会泄漏。
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

/*
	统一的错误出口：打印 errno 文本后直接退出。
	示例代码不追求恢复，只把失败原因说清楚。
*/
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

std::span<const std::byte> bytes_of(std::string_view s)
{
	return {reinterpret_cast<const std::byte *>(s.data()), s.size()};
}

std::string_view text_of(std::span<const std::byte> b)
{
	return {reinterpret_cast<const char *>(b.data()), b.size()};
}

/*
	阻塞 send，直到整段数据交给内核发送缓冲。
	MSG_NOSIGNAL：对端已关闭时让 send 返回 EPIPE，而不是给进程发 SIGPIPE
	把进程打死（只在 Linux 上有这个标志）。
*/
void send_all(int fd, std::span<const std::byte> data)
{
	std::size_t off = 0;
	while (off < data.size()) {
		const ssize_t n = ::send(fd, data.data() + off, data.size() - off, MSG_NOSIGNAL);
		if (n < 0) {
			if (errno == EINTR)
				continue;
			die("send");
		}
		if (n == 0) {
			std::fputs("send 返回 0，不应发生\n", stderr);
			std::exit(EXIT_FAILURE);
		}
		off += static_cast<std::size_t>(n);
	}
}

/*
	阻塞 recv，直到把 buf 填满。用于“已经知道长度”的正式读取。
	对端提前关闭（recv 返回 0）按错误处理，因为演示里帧必须完整。
*/
void recv_exact(int fd, std::span<std::byte> buf)
{
	std::size_t off = 0;
	while (off < buf.size()) {
		const ssize_t n = ::recv(fd, buf.data() + off, buf.size() - off, 0);
		if (n < 0) {
			if (errno == EINTR)
				continue;
			die("recv");
		}
		if (n == 0) {
			std::fputs("对端提前关闭了连接，帧不完整\n", stderr);
			std::exit(EXIT_FAILURE);
		}
		off += static_cast<std::size_t>(n);
	}
}

/*
	只做一次 peek，不处理“只 peek 到一部分”的情况。
	返回值语义与 recv 完全一样，这里原样返回给调用者观察。
*/
ssize_t peek_once(int fd, std::span<std::byte> buf)
{
	ssize_t n;
	do {
		n = ::recv(fd, buf.data(), buf.size(), MSG_PEEK);
	} while (n < 0 && errno == EINTR);
	if (n < 0)
		die("recv(MSG_PEEK)");
	return n;
}

/*
	FIONREAD 是 Linux 上探测“接收队列里还有多少字节可读”的老接口：
	- TCP：接收缓冲里已就绪的字节数，不等于整条连接剩余总量；
	- UDP：下一个数据报的长度。
	它不改动队列，正好用来证明 peek 没有取走数据。
*/
int readable_bytes(int fd)
{
	int avail = 0;
	check(::ioctl(fd, FIONREAD, &avail) == 0, "ioctl(FIONREAD)");
	return avail;
}

/*
	回环上 send() 返回时，数据不一定已经进入对端的接收队列（还要经过
	softirq 里的协议栈处理），所以紧接 send 之后读 FIONREAD 可能得到 0。
	要“等整帧到齐”就轮询 FIONREAD，直到至少 want 字节可见。
	peek 本身不需要这个：阻塞的 recv(MSG_PEEK) 会一直睡到数据到达。
*/
void wait_bytes_available(int fd, int want)
{
	// poll(nullptr, 0, 1) 在这里只当 1ms 的睡眠用，不关心 fd 事件。
	for (int i = 0; i < 2000; ++i) {
		if (readable_bytes(fd) >= want)
			return;
		::poll(nullptr, 0, 1);
	}
	std::fputs("等待数据到齐超时\n", stderr);
	std::exit(EXIT_FAILURE);
}

/*
	建一个只监听 127.0.0.1 临时端口的 TCP 监听器，并返回它的地址。
	端口填 0 表示让内核挑一个空闲端口，再用 getsockname 取回来。
	这样示例不依赖任何固定端口，也不会和别的程序冲突。
*/
Fd make_tcp_listener(sockaddr_in &addr)
{
	Fd listener{::socket(AF_INET, SOCK_STREAM | SOCK_CLOEXEC, 0)};
	check(listener, "socket(AF_INET, SOCK_STREAM)");

	// 回环上仍建议打开 SO_REUSEADDR，避免上次运行留下的 TIME_WAIT 干扰。
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

	// 单进程演示：先 listen(1)，再 connect，再 accept，不需要第二个进程。
	check(::listen(listener.get(), 1) == 0, "listen");
	return listener;
}

/*
	演示一：TCP 流上的“先 peek 头，再按长度收整帧”。
	长度头用 4 字节大端，这是网络协议里最常见的写法。
*/
void demo_tcp()
{
	std::puts("== 1. TCP 回环：peek 4 字节长度头，再按长度正式收 ==");

	sockaddr_in addr{};
	Fd listener = make_tcp_listener(addr);

	Fd client{::socket(AF_INET, SOCK_STREAM | SOCK_CLOEXEC, 0)};
	check(client, "socket(client)");
	check(::connect(client.get(), reinterpret_cast<sockaddr *>(&addr), sizeof addr) == 0, "connect");
	Fd conn{::accept4(listener.get(), nullptr, nullptr, SOCK_CLOEXEC)};
	check(conn, "accept4");

	constexpr std::string_view kPayload = "hello, MSG_PEEK: 先看长度头，再按长度收整帧";
	const std::uint32_t net_len = htonl(static_cast<std::uint32_t>(kPayload.size()));
	std::array<std::byte, 4> wire_len{};
	std::memcpy(wire_len.data(), &net_len, sizeof net_len);

	// conn 是服务端一侧，client 是客户端一侧：所有“到达”的数据都在 client
	// 的接收队列里，所以 peek/recv 一律在 client 上做，send 在 conn 上做。
	// 服务端先只发 4 字节长度头，模拟“头先到、体后到”的真实网络。
	send_all(conn.get(), wire_len);

	// 第一次 peek：只请求 4 字节，队列里也只有 4 字节。
	// 阻塞 socket 上如果数据还没到，recv(MSG_PEEK) 会睡到有数据为止。
	std::array<std::byte, 4> peeked_len{};
	const ssize_t n = peek_once(client.get(), peeked_len);
	std::printf("第 1 次 peek 返回 %zd，长度头解析出 payload = %u 字节\n", n,
		static_cast<unsigned>(ntohl(net_len)));

	// 关键点：peek 返回后 FIONREAD 是 4，说明 4 个字节仍在队列里 —— 没被取走。
	std::printf("peek 之后 FIONREAD = %d —— 数据没被取走\n", readable_bytes(client.get()));

	// 再 peek 一次，拿到的仍然是同一份长度头。这证明 peek 可重复调用。
	std::array<std::byte, 4> peeked_again{};
	check(peek_once(client.get(), peeked_again) == 4, "peek 第二次");
	check(std::memcmp(peeked_again.data(), wire_len.data(), wire_len.size()) == 0,
		"两次 peek 的内容一致");

	// 服务端再发负载。现在接收队列里是“头 + 体”一整帧。
	send_all(conn.get(), bytes_of(kPayload));
	wait_bytes_available(client.get(), static_cast<int>(4 + kPayload.size()));

	// 用一个比整帧大的缓冲 peek：流式 socket 只返回“当前可读的全部”，
	// 即 4 + payload，不会为了凑满 64 字节而等待。这正是“返回实际复制字节数”。
	std::array<std::byte, 64> peeked_frame{};
	const ssize_t frame_n = peek_once(client.get(), peeked_frame);
	std::printf("整帧到齐后，peek 64 字节返回 %zd（4 字节头 + %zu 字节体）\n", frame_n,
		kPayload.size());
	check(std::memcmp(peeked_frame.data(), wire_len.data(), wire_len.size()) == 0,
		"整帧 peek 的头也正确");

	// 正式收：先收走 4 字节头。此时头才真正离开队列。
	std::array<std::byte, 4> hdr{};
	recv_exact(client.get(), hdr);
	std::printf("正式 recv 头 4 字节后 FIONREAD = %d\n", readable_bytes(client.get()));
	check(std::memcmp(hdr.data(), wire_len.data(), hdr.size()) == 0, "正式收的头与 peek 一致");

	// 按头部声明的长度分配缓冲，一次收完负载。
	std::vector<std::byte> body(kPayload.size());
	recv_exact(client.get(), body);
	check(text_of(body) == kPayload, "整帧内容一致");
	const std::string_view body_text = text_of(body);
	std::printf("整帧内容校验通过：\"%.*s\"（%zu 字节）\n",
		static_cast<int>(body_text.size()), body_text.data(), body.size());
}

/*
	建一个绑定在 127.0.0.1 临时端口上的 UDP socket，并回填它的地址。
*/
Fd make_udp_bound(sockaddr_in &addr)
{
	Fd fd{::socket(AF_INET, SOCK_DGRAM | SOCK_CLOEXEC, 0)};
	check(fd, "socket(AF_INET, SOCK_DGRAM)");

	addr = {};
	addr.sin_family = AF_INET;
	addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
	addr.sin_port = 0;
	check(::bind(fd.get(), reinterpret_cast<sockaddr *>(&addr), sizeof addr) == 0, "bind(udp)");

	socklen_t len = sizeof addr;
	check(::getsockname(fd.get(), reinterpret_cast<sockaddr *>(&addr), &len) == 0,
		"getsockname(udp)");
	return fd;
}

/*
	演示二：UDP 上 peek 一个数据报，它仍然留在队列里。
	UDP 的“队列头部”是一个完整数据报；peek 不会像 TCP 那样看到半条消息。
*/
void demo_udp()
{
	std::puts("== 2. UDP 回环：peek 一个数据报后它仍在队列里 ==");

	sockaddr_in addr_a{};
	sockaddr_in addr_b{};
	Fd a = make_udp_bound(addr_a);
	Fd b = make_udp_bound(addr_b);

	// 两端都 connect，之后就能用 send/recv，而不必每次写 sendto/recvfrom。
	check(::connect(a.get(), reinterpret_cast<sockaddr *>(&addr_b), sizeof addr_b) == 0,
		"connect(a)");
	check(::connect(b.get(), reinterpret_cast<sockaddr *>(&addr_a), sizeof addr_a) == 0,
		"connect(b)");

	constexpr std::string_view kFirst = "alpha";
	constexpr std::string_view kSecond = "beta";

	// 连发两个数据报。UDP 不合并，接收队列里就是两个独立单元。
	check(::send(b.get(), kFirst.data(), kFirst.size(), 0) == static_cast<ssize_t>(kFirst.size()),
		"send(alpha)");
	check(::send(b.get(), kSecond.data(), kSecond.size(), 0) == static_cast<ssize_t>(kSecond.size()),
		"send(beta)");

	// 第一次 peek：拿到第一个数据报，缓冲比数据报大，返回真实长度 5。
	std::array<std::byte, 64> buf{};
	const ssize_t n1 = peek_once(a.get(), buf);
	const std::string_view first_text = text_of({buf.data(), static_cast<std::size_t>(n1)});
	std::printf("第 1 次 peek：返回 %zd，内容 \"%.*s\"\n", n1,
		static_cast<int>(first_text.size()), first_text.data());

	// FIONREAD 在 UDP 上报告队列头部那个数据报的长度。因为 peek 没有取走
	// alpha，这里仍然是 5 —— 又一次证明数据还在队列里。
	std::printf("此刻 FIONREAD = %d（队头仍是 alpha）\n", readable_bytes(a.get()));

	// 再 peek 一次：还是 alpha，位置没有前进。
	const ssize_t n2 = peek_once(a.get(), buf);
	const std::string_view second_text = text_of({buf.data(), static_cast<std::size_t>(n2)});
	std::printf("第 2 次 peek：返回 %zd，内容 \"%.*s\"（仍是同一个数据报）\n", n2,
		static_cast<int>(second_text.size()), second_text.data());

	// 正式 recv 才把它摘走；下一次 recv 才是 beta。
	std::array<std::byte, 64> got{};
	ssize_t n = ::recv(a.get(), got.data(), got.size(), 0);
	check(n == n1 && text_of({got.data(), static_cast<std::size_t>(n)}) == kFirst,
		"正式 recv alpha");
	const std::string_view got_text = text_of({got.data(), static_cast<std::size_t>(n)});
	std::printf("正式 recv：\"%.*s\"（之后 FIONREAD = %d）\n",
		static_cast<int>(got_text.size()), got_text.data(), readable_bytes(a.get()));

	n = ::recv(a.get(), got.data(), got.size(), 0);
	check(n == static_cast<ssize_t>(kSecond.size()) &&
		text_of({got.data(), static_cast<std::size_t>(n)}) == kSecond, "正式 recv beta");
	const std::string_view second_got = text_of({got.data(), static_cast<std::size_t>(n)});
	std::printf("再 recv：\"%.*s\" —— 第二个数据报才被取走\n",
		static_cast<int>(second_got.size()), second_got.data());

	// 截断演示：peek 时缓冲比数据报小，只复制缓冲大小的部分。
	std::array<std::byte, 100> big{};
	for (std::size_t i = 0; i < big.size(); ++i)
		big[i] = static_cast<std::byte>('0' + i % 10);
	check(::send(b.get(), big.data(), big.size(), 0) == static_cast<ssize_t>(big.size()),
		"send(100 字节)");

	std::array<std::byte, 10> small{};
	const ssize_t trunc = peek_once(a.get(), small);
	std::printf("用 10 字节缓冲 peek 100 字节数据报：返回 %zd（被截断）\n", trunc);

	// MSG_TRUNC 与 MSG_PEEK 组合：仍然不取走数据，但返回“数据报的真实长度”。
	const ssize_t real_len = ::recv(a.get(), small.data(), small.size(), MSG_PEEK | MSG_TRUNC);
	std::printf("同一数据报再加 MSG_TRUNC：返回 %zd = 真实长度\n", real_len);
	check(real_len == static_cast<ssize_t>(big.size()), "MSG_TRUNC 返回真实长度");

	// 正式收仍然完整：peek 的截断只影响那一次复制，没有破坏队列里的数据报。
	std::array<std::byte, 128> full{};
	const ssize_t full_n = ::recv(a.get(), full.data(), full.size(), 0);
	check(full_n == static_cast<ssize_t>(big.size()), "正式 recv 完整");
	check(std::memcmp(full.data(), big.data(), big.size()) == 0, "数据报内容完整");
	std::printf("正式 recv 仍然拿到完整的 %zd 字节\n", full_n);
}

} // namespace

int main()
{
	demo_tcp();
	std::puts("");
	demo_udp();

	/*
		常见坑（不在这段演示里逐一触发，但写代码时必须知道）：
		1. peek 不是免费的：流式 socket 上有大量积压时，每次 peek 都把
		队列里能看见的部分复制一遍；拿它当“预读”会翻倍拷贝开销。
		2. peek 之后 socket 仍然可读。用 epoll/poll 驱动的程序如果只 peek
		不 recv，会一直收到可读通知，形成忙等。
		3. 对阻塞 socket，队列为空时 peek 会睡眠，和普通 recv 一样。
		需要“只看一眼就走”时用 MSG_PEEK | MSG_DONTWAIT，并对 EAGAIN
		做正常处理。
		4. TCP 上 peek 返回的字节数可能少于请求值；“peek 到 n 字节头”
		必须自己循环，或者先确认至少有 n 字节（FIONREAD）。
		5. 数据报在 peek 与真正 recv 之间并没有被锁定：同一 socket 若还有
		别的线程/进程在读，数据报可能先被别人拿走。
		6. MSG_WAITALL 与 MSG_PEEK 组合的语义各平台不一致（本机 man page
		缺失，未核对 Linux 的具体行为）；要“凑齐 n 字节再返回”时，
		自己循环比依赖这个组合安全。
	*/
	return 0;
}
