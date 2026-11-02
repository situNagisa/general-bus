/*
send / recv —— TCP/UDP 上最基础的一次一段收发

这个接口是什么、解决什么问题
	send() 把一个（或一段连续的）字节缓冲区写进已连接套接字；recv() 从已连接
	套接字里取出至多一个缓冲区大小的字节。它们是 read()/write() 的套接字版本，
	区别只在于多了一个 flags 参数（MSG_NOSIGNAL、MSG_DONTWAIT、MSG_PEEK 等）。
	所有更高层的收发接口（writev/readv、sendmsg/recvmsg、sendmmsg/recvmmsg、
	io_uring 的 SEND/RECV）最终都落到同一套套接字发送/接收队列上，send/recv
	是理解它们的起点。

	什么时候用：
	- 只需要一段连续内存，且调用者愿意在阻塞调用里等这一次收发完成；
	- 教学、控制通道、小消息。
	不适合：一次要交多段内存（用 writev）、要带地址或附加控制信息（用
	sendmsg）、要一次交多个数据报（用 sendmmsg）、要异步/批量高性能（用
	io_uring）。

形态（本接口的关键约束）
	每次调用：1 段内存（buf + len）。
	对应线上：TCP 上是字节流的一段（没有消息边界）；UDP 上是 1 个数据报。
	提交次数：1 次；完成次数：1 次（阻塞模式下返回时即完成，可能只完成一部分）。
	内存：由调用者提供，内核只读写这块内存，不在调用返回后继续引用它
	（除非用了 MSG_ZEROCOPY 这类特殊选项）。

需要的内核/硬件条件
	无特殊条件。本示例只用 IPv4 回环地址 127.0.0.1，Linux 任意版本、WSL2 均可。

怎么编译、怎么运行
	g++ -std=c++26 -O2 -Wall -Wextra -o /tmp/native_send_recv send_recv.cpp
	/tmp/native_send_recv

预期输出（端口号每次不同）
	[TCP] send 返回 9
	[TCP] recv 返回 9，内容 "hello-tcp"
	[TCP] 对端关闭后 recv 返回 0（EOF）
	[UDP] 第 1 次 recv 返回 6，内容 "ping-1"
	[UDP] 第 2 次 recv 返回 13，内容 "ping-2-longer"
	[UDP] 第 3 次 recv 返回 0（零长度数据报，不是 EOF）
	[UDP] 再收一次仍有数据：4 字节 "pong"
	全部演示结束
*/

#include <arpa/inet.h>
#include <array>
#include <cerrno>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <netinet/in.h>
#include <print>
#include <signal.h>
#include <span>
#include <string_view>
#include <sys/socket.h>
#include <unistd.h>
#include <utility>

/*
RAII 的 fd 包装：析构时 close，禁止拷贝，允许移动。
演示中所有套接字都由它持有，任何一步出错退出都不会泄漏 fd。
这是 C++ 相对裸 C 的唯一结构性差异，系统调用本身仍原样直接调用。
*/
class Fd {
public:
	explicit Fd(int fd = -1) noexcept : fd_(fd) {}

	~Fd()
	{
		reset();
	}

	Fd(const Fd &) = delete;
	Fd &operator=(const Fd &) = delete;

	Fd(Fd &&other) noexcept : fd_(std::exchange(other.fd_, -1)) {}

	Fd &operator=(Fd &&other) noexcept
	{
		if (this != &other)
			reset(std::exchange(other.fd_, -1));
		return *this;
	}

	int get() const noexcept
	{
		return fd_;
	}

	bool valid() const noexcept
	{
		return fd_ >= 0;
	}

	/* 提前关闭（例如演示里想在 client 仍活着时关闭 server 端）。 */
	void reset(int fd = -1) noexcept
	{
		if (fd_ >= 0)
			::close(fd_);
		fd_ = fd;
	}

private:
	int fd_;
};

/* 统一的错误处理：打印 errno 文本后退出。演示程序出错就直接停。 */
[[noreturn]] static void die(std::string_view what)
{
	std::fprintf(stderr, "%.*s 失败: %s\n", static_cast<int>(what.size()),
		     what.data(), std::strerror(errno));
	std::exit(EXIT_FAILURE);
}

/*
在 127.0.0.1 上建立一对已连接的 TCP 套接字，返回 client 和 server 两端。
先 listen 再 connect：connect 会由内核在 backlog 里完成握手，随后的 accept
必然能拿到连接，因此单线程、单进程就能自产自销，不需要 fork。
*/
static std::pair<Fd, Fd> make_tcp_pair()
{
	Fd listener{socket(AF_INET, SOCK_STREAM, 0)};
	if (!listener.valid())
		die("socket(TCP listen)");

	sockaddr_in addr{};
	addr.sin_family = AF_INET;			/* IPv4 */
	addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);	/* 127.0.0.1 */
	addr.sin_port = 0;				/* 0：让内核挑一个空闲端口 */

	if (bind(listener.get(), reinterpret_cast<sockaddr *>(&addr), sizeof addr) < 0)
		die("bind(TCP)");
	if (listen(listener.get(), 1) < 0)		/* backlog=1，只等一个连接 */
		die("listen");

	/* 取回内核分配的端口，connect 要用 */
	socklen_t alen = sizeof addr;
	if (getsockname(listener.get(), reinterpret_cast<sockaddr *>(&addr), &alen) < 0)
		die("getsockname");

	Fd client{socket(AF_INET, SOCK_STREAM, 0)};
	if (!client.valid())
		die("socket(TCP client)");
	if (connect(client.get(), reinterpret_cast<sockaddr *>(&addr), sizeof addr) < 0)
		die("connect");

	int sfd = accept(listener.get(), nullptr, nullptr);	/* 不关心对端地址 */
	if (sfd < 0)
		die("accept");
	Fd server{sfd};

	return {std::move(client), std::move(server)};
}

/* TCP 演示：一段内存、一次 send、一次 recv，以及 recv 返回 0 = 对端关闭。 */
static void demo_tcp()
{
	auto [cli, srv] = make_tcp_pair();

	constexpr std::string_view msg = "hello-tcp";
	/*
	send 的第三个参数是 flags。MSG_NOSIGNAL 表示：如果对端已经关闭，返回
	EPIPE 而不是给进程发 SIGPIPE（SIGPIPE 默认杀死进程，这是新手最常见的
	"程序莫名其妙没了" 的原因）。也可以全局 signal(SIGPIPE, SIG_IGN)。
	返回值：成功时是实际写入的字节数，可能小于 len（短写）；-1 表示错误。
	*/
	ssize_t n = send(srv.get(), msg.data(), msg.size(), MSG_NOSIGNAL);
	if (n < 0)
		die("send(TCP)");
	std::println("[TCP] send 返回 {}", n);
	/*
	这里只有 9 字节，远小于套接字发送缓冲区，所以一次就写完。真实程序中
	必须检查 n < len 并循环发送剩余部分；writev 的示例会完整演示续写。
	*/

	std::array<char, 64> buf{};
	/*
	recv 的返回值：
	  >0 : 实际拷贝到 buf 的字节数，可能小于 buf.size()，这不是错误；
	   0 : 有序关闭（对端发了 FIN）。TCP 上这是 EOF，此后不会再有数据；
	  -1 : 错误。EINTR 表示被信号打断，通常应重试；EAGAIN/EWOULDBLOCK
	       表示非阻塞模式下暂时没有数据。
	注意 TCP 是字节流：一次 send 的 9 字节并不保证一次 recv 就能收全，
	也可能和后续 send 的数据粘在一起。这里先 send 后 recv 且数据量小，
	才有 "一次 recv 拿到 9 字节" 的结果。
	*/
	n = recv(cli.get(), buf.data(), buf.size(), 0);
	if (n < 0)
		die("recv(TCP)");
	std::println("[TCP] recv 返回 {}，内容 \"{}\"", n,
		     std::string_view{buf.data(), static_cast<size_t>(n)});

	/*
	关闭 server 端。close 会让内核在对端方向排入 FIN；没有未发送数据时，
	对端下一次 recv 就会看到 0。这里显式 reset 以便在 client 仍存活时关闭。
	*/
	srv.reset();

	n = recv(cli.get(), buf.data(), buf.size(), 0);
	if (n < 0)
		die("recv(TCP EOF)");
	std::println("[TCP] 对端关闭后 recv 返回 {}（EOF）", n);
}

/* UDP 演示：一次 recv 恰好对应一个数据报；recv 返回 0 在 UDP 上不是 EOF。 */
static void demo_udp()
{
	/* 接收端：绑定到 127.0.0.1:0，由内核分配端口 */
	Fd rfd{socket(AF_INET, SOCK_DGRAM, 0)};
	if (!rfd.valid())
		die("socket(UDP recv)");

	sockaddr_in raddr{};
	raddr.sin_family = AF_INET;
	raddr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
	raddr.sin_port = 0;
	if (bind(rfd.get(), reinterpret_cast<sockaddr *>(&raddr), sizeof raddr) < 0)
		die("bind(UDP)");

	socklen_t alen = sizeof raddr;
	if (getsockname(rfd.get(), reinterpret_cast<sockaddr *>(&raddr), &alen) < 0)
		die("getsockname(UDP)");
	std::println("[UDP] 接收端绑定在 127.0.0.1:{}", ntohs(raddr.sin_port));

	/*
	发送端 connect 到接收端。UDP 的 connect 不发任何包，只是在内核里记下
	默认对端，之后就能用 send/recv 而不是 sendto/recvfrom，并且能收到
	ICMP 端口不可达之类的错误。本例用 connected UDP 是为了和 TCP 部分
	保持同样的 send/recv 调用形式。
	*/
	Fd sfd{socket(AF_INET, SOCK_DGRAM, 0)};
	if (!sfd.valid())
		die("socket(UDP send)");
	if (connect(sfd.get(), reinterpret_cast<sockaddr *>(&raddr), sizeof raddr) < 0)
		die("connect(UDP)");

	struct Datagram {
		std::string_view data;
	};
	const std::array dgrams{
		Datagram{"ping-1"},
		Datagram{"ping-2-longer"},
		Datagram{""},	/* 零长度数据报：合法，会单独占一个 datagram */
	};

	for (const auto &d : dgrams) {
		/*
		UDP 上一次 send 必须完整发出一个数据报：要么返回整个长度，要么
		返回 -1（如 EMSGSIZE 超过上限、ENOBUFS、EACCES 等）。
		UDP 不会出现 TCP 那种 "写了一半" 的短写；部分发送在数据报语义下
		没有意义。
		*/
		ssize_t n = send(sfd.get(), d.data.data(), d.data.size(), 0);
		if (n < 0)
			die("send(UDP)");
		std::println("[UDP] send {} 字节，返回 {}", d.data.size(), n);
	}

	std::array<char, 128> buf{};
	for (int i = 0; i < 3; i++) {
		ssize_t n = recv(rfd.get(), buf.data(), buf.size(), 0);
		if (n < 0)
			die("recv(UDP)");
		/*
		关键语义：一次 recv 只取走一个数据报。即使 buf 有 128 字节，
		也不会把两个数据报拼起来；反之 buf 太小则多出的部分被丢弃
		（sendmsg/recvmsg 的示例会用 MSG_TRUNC 观察这件事）。
		*/
		std::println("[UDP] 第 {} 次 recv 返回 {}，内容 \"{}\"", i + 1, n,
			     std::string_view{buf.data(), static_cast<size_t>(n)});
	}

	/*
	第三个数据报长度为 0，recv 返回 0。UDP 是无连接的，0 在这里表示
	"收到一个空数据报"，绝不是 EOF。判断 UDP 对端是否还活着只能靠应用层
	心跳/应答，recv 返回 0 不能作为关闭信号。
	*/
	std::println("[UDP] 上面第 3 次返回 0 是零长度数据报，不是 EOF");

	/*
	随后再发一个普通数据报，证明接收端仍然可用——如果 0 真是 EOF，
	这里就会失败或再也收不到数据。
	*/
	constexpr std::string_view pong = "pong";
	ssize_t n = send(sfd.get(), pong.data(), pong.size(), 0);
	if (n < 0)
		die("send(UDP pong)");
	n = recv(rfd.get(), buf.data(), buf.size(), 0);
	if (n < 0)
		die("recv(UDP pong)");
	std::println("[UDP] 再收一次仍有数据：{} 字节 \"{}\"", n,
		     std::string_view{buf.data(), static_cast<size_t>(n)});
}

int main()
{
	/*
	虽然本程序发送时都带了 MSG_NOSIGNAL，仍然把 SIGPIPE 设为忽略，作为
	双保险，也演示服务器程序常见的全局设置。
	*/
	signal(SIGPIPE, SIG_IGN);

	demo_tcp();
	demo_udp();

	std::println("全部演示结束");
	return 0;
}
