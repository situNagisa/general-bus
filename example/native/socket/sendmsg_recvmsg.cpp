/*
sendmsg / recvmsg —— 带对端地址与辅助数据（cmsg）的收发

这个接口是什么、解决什么问题
	sendmsg()/recvmsg() 是 send/recv 的 "完整形态"：除了多段内存（iovec，
	和 writev/readv 一样），它还接收/返回
	  - 对端地址（msg_name）：在未连接的 UDP 套接字上一次调用就能指定目的地；
	  - 辅助数据（msg_control 里的 cmsg）：随数据报一起传递的控制信息，如
	    数据报到达的本地地址/接口（IP_PKTINFO）、内核收包时间戳（SO_TIMESTAMP）、
	    显式拥塞通知、源地址选择等。
	它解决的正是 "消息之外还带有元数据" 的问题：这些信息无法塞进字节流，
	也不该被当作普通数据，cmsg 让内核和应用在同一次调用里交换它们。

	什么时候用：
	- UDP 服务器需要知道数据报从哪个地址来、落到哪个本地地址/网卡；
	- 需要内核时间戳（比用户态收包后再 clock_gettime 更接近真实到达时刻）；
	- 需要按消息选择源地址、设置 TOS/优先级、传递 SCM_RIGHTS 等。
	不需要地址和元数据时，writev/readv 更直接、参数更少。

形态（本接口的关键约束）
	sendmsg：n 段内存 + 1 个目标地址 + m 个 cmsg -> 1 次提交，
	         对应 1 个 UDP 数据报（或 TCP 字节流的一段），1 次完成。
	recvmsg：n 段内存 + 1 个地址输出缓冲 + 1 个控制信息输出缓冲 -> 1 次提交，
	         取走 1 个数据报，1 次完成。
	内存：iovec 数组、每段数据缓冲、sockaddr、控制缓冲全部由调用者提供；
	内核只在调用期间访问它们，返回后不再引用。
	返回值只报告 "数据拷贝了多少字节"；地址长度写在 msg_namelen，
	控制信息长度写在 msg_controllen，标志写在 msg_flags。

需要的内核/硬件条件
	无特殊条件，IPv4 回环即可。IP_PKTINFO 与 SO_TIMESTAMP 都是 Linux
	长期支持的选项；SO_TIMESTAMP 返回的 cmsg 里是 struct timeval。
	本机内核 6.18（WSL2）实测：一次 recvmsg 可以同时收到 SCM_TIMESTAMP
	和 IP_PKTINFO 两个 cmsg。

怎么编译、怎么运行
	g++ -std=c++26 -O2 -Wall -Wextra -o /tmp/native_sendmsg_recvmsg sendmsg_recvmsg.cpp
	/tmp/native_sendmsg_recvmsg

预期输出（端口每次不同，时间戳为当前时间）
	[setup] 接收端 127.0.0.1:XXXX，已开启 IP_PKTINFO 和 SO_TIMESTAMP
	[sendmsg] 两段共 12 字节 -> 1 个数据报，返回 12
	[recvmsg] 返回 12，来自 127.0.0.1:YYYY，msg_flags=0x0
	[recvmsg] IP_PKTINFO: ifindex=3 目的地址=127.0.0.1 本地地址=127.0.0.1
	[recvmsg] SCM_TIMESTAMP: 1793... . ...
	[MSG_TRUNC] 100 字节数据报 + 16 字节缓冲 -> 返回 16，msg_flags 含 MSG_TRUNC
	[MSG_TRUNC] 再传 MSG_TRUNC 标志 -> 返回 100（真实长度），数据仍只拷 16 字节
	[MSG_CTRUNC] 控制缓冲只给 16 字节 -> 数据完整，msg_flags 含 MSG_CTRUNC
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
#include <span>
#include <string_view>
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h>
#include <utility>

/* RAII 的 fd 包装：析构 close，禁止拷贝，允许移动。 */
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

	void reset(int fd = -1) noexcept
	{
		if (fd_ >= 0)
			::close(fd_);
		fd_ = fd;
	}

private:
	int fd_;
};

[[noreturn]] static void die(std::string_view what)
{
	std::fprintf(stderr, "%.*s 失败: %s\n", static_cast<int>(what.size()),
		     what.data(), std::strerror(errno));
	std::exit(EXIT_FAILURE);
}

/*
控制信息缓冲区的对齐要求：cmsghdr 必须按自然对齐访问，因此用一个 union
把字符数组和 cmsghdr 叠在一起，保证起始地址满足 alignof(cmsghdr)。
直接写 char cbuf[N] 再强转是常见的未对齐 bug。
*/
union ControlBuffer {
	cmsghdr align;
	char bytes[512];
};

/* 用于把 IPv4 地址安全地格式化成字符串（inet_ntoa 用静态缓冲，不能同时用两次）。 */
static const char *ip_str(const in_addr &a, char (&buf)[INET_ADDRSTRLEN])
{
	return inet_ntop(AF_INET, &a, buf, sizeof buf);
}

/*
发送一个 UDP 数据报。
	parts      ：要拼接的多段内存，内核按顺序拼成一个数据报发出；
	with_src   ：是否附带 IP_PKTINFO cmsg 指定源地址（演示 sendmsg 也能带控制信息）；
	返回实际发送的字节数。
*/
static ssize_t send_datagram(int fd, const sockaddr_in &dst,
			     std::span<const std::string_view> parts,
			     bool with_src)
{
	/* iovec 数组必须覆盖所有段；段数在运行时决定 */
	std::array<iovec, 4> iov{};
	if (parts.size() > iov.size())
		die("演示程序段数写死");
	for (size_t i = 0; i < parts.size(); i++)
		iov[i] = iovec{const_cast<char *>(parts[i].data()), parts[i].size()};

	msghdr msg{};
	/*
	msg_name/msg_namelen：目标地址。UDP 未 connect 时每次都要给；
	如果套接字已 connect，这两个字段可以留空（NULL/0）。
	*/
	msg.msg_name = const_cast<sockaddr_in *>(&dst);
	msg.msg_namelen = sizeof dst;
	msg.msg_iov = iov.data();
	msg.msg_iovlen = parts.size();

	ControlBuffer cbuf{};
	if (with_src) {
		/*
		构造一条 cmsg。cmsg 的布局是：
			cmsghdr { cmsg_len, cmsg_level, cmsg_type } + 数据（按需填充对齐）
		CMSG_SPACE(sz) 给出 "包含 sz 字节数据、且考虑对齐" 的总长度；
		CMSG_LEN(sz)   给出 cmsg_len 应填的值（头 + 数据，不含尾部填充）。
		msg_controllen 要填总空间（CMSG_SPACE），内核据此遍历。
		*/
		msg.msg_control = cbuf.bytes;
		msg.msg_controllen = CMSG_SPACE(sizeof(in_pktinfo));

		cmsghdr *cm = CMSG_FIRSTHDR(&msg);
		cm->cmsg_level = IPPROTO_IP;
		cm->cmsg_type = IP_PKTINFO;
		cm->cmsg_len = CMSG_LEN(sizeof(in_pktinfo));

		in_pktinfo pi{};
		/*
		ipi_spec_dst：希望使用的源地址（这里固定为回环地址）。
		ipi_ifindex：希望使用的出接口，0 表示由路由决定。
		注意：指定源地址需要该地址属于本机；若无权限/地址不属于本机，
		sendmsg 会失败（EINVAL/EPERM 等），错误处理见下。
		*/
		pi.ipi_spec_dst.s_addr = htonl(INADDR_LOOPBACK);
		std::memcpy(CMSG_DATA(cm), &pi, sizeof pi);
	}

	/*
	sendmsg 对 UDP：要么把整个数据报交给内核并返回总字节数，要么返回 -1，
	不存在 TCP 那种 "只写出一部分" 的短写。
	常见错误：EMSGSIZE（超过路径 MTU/套接字上限）、EACCES（广播权限等）、
	ENOBUFS、EINVAL（cmsg 内容不合法）、EPIPE（已 connect 的对端关闭）。
	*/
	ssize_t n = sendmsg(fd, &msg, 0);
	if (n < 0)
		die("sendmsg");
	return n;
}

/*
接收一个数据报并打印地址与全部 cmsg。
	control_size：控制缓冲可用长度，用来演示 MSG_CTRUNC。
	返回数据字节数；msg_flags 通过引用带出。
*/
static ssize_t recv_one(int fd, std::span<char> data,
			size_t control_size, int &flags,
			sockaddr_in &from, socklen_t &from_len,
			ControlBuffer &cbuf, msghdr &msg)
{
	std::memset(&msg, 0, sizeof msg);
	std::memset(&from, 0, sizeof from);
	from_len = sizeof from;

	/*
	msg_name 指向一个足够大的 sockaddr：recvmsg 会把发送方地址写进去，
	并把实际长度写回 msg_namelen。UDP 上这就是数据报的源地址。
	*/
	msg.msg_name = &from;
	msg.msg_namelen = from_len;
	msg.msg_iov = nullptr;
	msg.msg_iovlen = 0;
	(void)data;

	/* 数据段由调用者提供；这里只收 1 段，但 recvmsg 天然支持多段。 */
	iovec iov{data.data(), data.size()};
	msg.msg_iov = &iov;
	msg.msg_iovlen = 1;

	/*
	控制缓冲：调用者提供空间，内核把 cmsg 写进来。
	若 control_size 小于某个 cmsg 的 CMSG_SPACE，该 cmsg 被丢弃并在
	msg_flags 里置 MSG_CTRUNC（control truncated）。
	*/
	msg.msg_control = control_size ? static_cast<void *>(cbuf.bytes) : nullptr;
	msg.msg_controllen = control_size;

	/*
	recvmsg 的返回值：
	  >0 : 拷贝进 iovec 的数据字节数。对 UDP 是 "本次数据报被拷贝的部分"：
	       缓冲够大就是完整数据报长度；缓冲太小则只拷贝缓冲大小，
	       并在 msg_flags 里置 MSG_TRUNC（见后面的演示）。
	   0 : UDP 上是零长度数据报；TCP 上才是对端关闭（EOF）。
	  -1 : 错误，EINTR 应重试，EAGAIN 表示非阻塞下暂无数据。
	*/
	ssize_t n = recvmsg(fd, &msg, 0);
	if (n < 0)
		die("recvmsg");

	flags = msg.msg_flags;
	from_len = msg.msg_namelen;
	return n;
}

/* 打印 msghdr 里的全部 cmsg。遍历必须用 CMSG_FIRSTHDR/CMSG_NXTHDR。 */
static void dump_cmsgs(msghdr &msg)
{
	/*
	不能假设只有一个 cmsg，也不能用固定偏移自己算：每条 cmsg 之间有
	对齐填充，且顺序由内核决定。CMSG_NXTHDR 会正确处理这些。
	*/
	for (cmsghdr *cm = CMSG_FIRSTHDR(&msg); cm != nullptr;
	     cm = CMSG_NXTHDR(&msg, cm)) {
		if (cm->cmsg_level == IPPROTO_IP && cm->cmsg_type == IP_PKTINFO) {
			in_pktinfo pi{};
			std::memcpy(&pi, CMSG_DATA(cm), sizeof pi);
			char dst[INET_ADDRSTRLEN];
			char local[INET_ADDRSTRLEN];
			std::println("[recvmsg]   IP_PKTINFO: ifindex={} 目的地址={} 本地地址={}",
				     pi.ipi_ifindex,
				     ip_str(pi.ipi_addr, dst),
				     ip_str(pi.ipi_spec_dst, local));
			/*
			ipi_addr    ：数据报头里的目的地址（多宿主/广播时有用）；
			ipi_spec_dst：内核选出的本地地址（常用于回填源地址）；
			ipi_ifindex ：收到该包的接口索引。
			*/
		} else if (cm->cmsg_level == SOL_SOCKET && cm->cmsg_type == SCM_TIMESTAMP) {
			timeval tv{};
			/* SO_TIMESTAMP 交付的是 struct timeval；长度可用 cmsg_len 校验 */
			std::memcpy(&tv, CMSG_DATA(cm), sizeof tv);
			std::println("[recvmsg]   SCM_TIMESTAMP: {}.{:06}", tv.tv_sec,
				     static_cast<long>(tv.tv_usec));
		} else {
			std::println("[recvmsg]   未知 cmsg: level={} type={} len={}",
				     cm->cmsg_level, cm->cmsg_type,
				     static_cast<size_t>(cm->cmsg_len));
		}
	}
}

int main()
{
	/* 接收端：UDP，绑定 127.0.0.1:0 */
	Fd rfd{socket(AF_INET, SOCK_DGRAM, 0)};
	if (!rfd.valid())
		die("socket(recv)");

	sockaddr_in raddr{};
	raddr.sin_family = AF_INET;
	raddr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
	raddr.sin_port = 0;
	if (bind(rfd.get(), reinterpret_cast<sockaddr *>(&raddr), sizeof raddr) < 0)
		die("bind");
	socklen_t rlen = sizeof raddr;
	if (getsockname(rfd.get(), reinterpret_cast<sockaddr *>(&raddr), &rlen) < 0)
		die("getsockname");

	/*
	开启两个 "让内核附带 cmsg" 的选项。它们是套接字选项而不是 recvmsg 的
	参数：不开的话，内核不会生成对应 cmsg，recvmsg 自然也收不到。
	*/
	int on = 1;
	if (setsockopt(rfd.get(), IPPROTO_IP, IP_PKTINFO, &on, sizeof on) < 0)
		die("setsockopt(IP_PKTINFO)");
	if (setsockopt(rfd.get(), SOL_SOCKET, SO_TIMESTAMP, &on, sizeof on) < 0)
		die("setsockopt(SO_TIMESTAMP)");
	std::println("[setup] 接收端 127.0.0.1:{}，已开启 IP_PKTINFO 和 SO_TIMESTAMP",
		     ntohs(raddr.sin_port));

	/* 发送端：不 connect，每次 sendmsg 自己给 msg_name */
	Fd sfd{socket(AF_INET, SOCK_DGRAM, 0)};
	if (!sfd.valid())
		die("socket(send)");
	sockaddr_in laddr{};
	laddr.sin_family = AF_INET;
	laddr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
	laddr.sin_port = 0;
	if (bind(sfd.get(), reinterpret_cast<sockaddr *>(&laddr), sizeof laddr) < 0)
		die("bind(send)");
	socklen_t slen = sizeof laddr;
	if (getsockname(sfd.get(), reinterpret_cast<sockaddr *>(&laddr), &slen) < 0)
		die("getsockname(send)");

	ControlBuffer cbuf{};
	msghdr msg{};
	sockaddr_in from{};
	socklen_t from_len = 0;
	int flags = 0;
	std::array<char, 256> buf{};

	/*
	演示 1：两段内存 + 目标地址 -> 一个数据报。
	TCP 上 sendmsg 也可以这样传多段（等价于 writev），但没有 msg_name；
	这里用 UDP 是为了同时展示地址字段。
	*/
	const std::array<std::string_view, 2> parts{"sendmsg", "-demo"};
	ssize_t sent = send_datagram(sfd.get(), raddr, parts, false);
	std::println("[sendmsg] 两段共 {} 字节 -> 1 个数据报，返回 {}",
		     parts[0].size() + parts[1].size(), sent);

	ssize_t n = recv_one(rfd.get(), buf, sizeof cbuf.bytes, flags, from,
			     from_len, cbuf, msg);
	char src[INET_ADDRSTRLEN];
	std::println("[recvmsg] 返回 {}，来自 {}:{}，msg_flags=0x{:x}", n,
		     ip_str(from.sin_addr, src), ntohs(from.sin_port), flags);
	std::println("[recvmsg] 数据: \"{}\"",
		     std::string_view{buf.data(), static_cast<size_t>(n)});
	std::println("[recvmsg] 控制信息 {} 字节，共解析出 cmsg：",
		     msg.msg_controllen);
	dump_cmsgs(msg);
	/*
	msg_flags 是 "输出" 字段：recvmsg 返回后由内核填写，表示本次接收的
	状态。0 表示没有异常；MSG_TRUNC 表示数据被截断；MSG_CTRUNC 表示
	控制信息被截断。调用前不需要（也不应）把它当输入。
	*/

	/*
	演示 2：sendmsg 附带 IP_PKTINFO cmsg 指定源地址。
	这里源地址仍然是 127.0.0.1（本机唯一回环地址），所以效果是 "显式选择了
	本来就用的地址"；在多网卡机器上可以借此实现多宿主源地址选择。
	*/
	const std::array<std::string_view, 1> part2{"pktinfo-src"};
	sent = send_datagram(sfd.get(), raddr, part2, true);
	std::println("[sendmsg] 带 IP_PKTINFO cmsg 发送 {} 字节，返回 {}",
		     part2[0].size(), sent);

	n = recv_one(rfd.get(), buf, sizeof cbuf.bytes, flags, from, from_len,
		     cbuf, msg);
	std::println("[recvmsg] 返回 {}，源端口 {}（发送端绑定的是 {}）", n,
		     ntohs(from.sin_port), ntohs(laddr.sin_port));
	dump_cmsgs(msg);

	/*
	演示 3：数据缓冲小于数据报 -> MSG_TRUNC。
	发送 100 字节，接收只给 16 字节。Linux 上 recvmsg 返回 16（实际拷贝量），
	同时 msg_flags 置 MSG_TRUNC；剩余 84 字节已被内核丢弃，无法再次读到。
	*/
	std::array<char, 100> big{};
	big.fill('T');
	const std::array<std::string_view, 1> part3{
		std::string_view{big.data(), big.size()}};
	sent = send_datagram(sfd.get(), raddr, part3, false);
	std::println("[MSG_TRUNC] 发送 {} 字节数据报", sent);

	std::array<char, 16> small{};
	n = recv_one(rfd.get(), small, sizeof cbuf.bytes, flags, from, from_len,
		     cbuf, msg);
	std::println("[MSG_TRUNC] 16 字节缓冲 recvmsg 返回 {}，msg_flags=0x{:x}（MSG_TRUNC={}）",
		     n, flags, (flags & MSG_TRUNC) ? 1 : 0);

	/*
	演示 3b：在 flags 参数里传 MSG_TRUNC，返回值变成数据报的真实长度
	（100），但数据仍然只拷贝了 16 字节。这是 Linux 对 Internet 数据报
	的行为（man recv(2)：since Linux 2.4.27/2.6.8），本机实测如此。
	*/
	sent = send_datagram(sfd.get(), raddr, part3, false);
	std::println("[MSG_TRUNC] 再发 {} 字节", sent);

	std::memset(&msg, 0, sizeof msg);
	std::memset(&from, 0, sizeof from);
	from_len = sizeof from;
	iovec siov{small.data(), small.size()};
	msg.msg_name = &from;
	msg.msg_namelen = from_len;
	msg.msg_iov = &siov;
	msg.msg_iovlen = 1;
	msg.msg_control = cbuf.bytes;
	msg.msg_controllen = sizeof cbuf.bytes;
	n = recvmsg(rfd.get(), &msg, MSG_TRUNC);
	if (n < 0)
		die("recvmsg(MSG_TRUNC)");
	flags = msg.msg_flags;
	std::println("[MSG_TRUNC] 传 MSG_TRUNC 标志后返回 {}（真实长度），msg_flags=0x{:x}",
		     n, flags);
	/* 对比：返回值是 100，但缓冲只有 16，只能安全使用前 16 字节。 */

	/*
	演示 4：控制缓冲太小 -> MSG_CTRUNC。
	数据缓冲给足，但控制缓冲只给 16 字节，装不下任何一条完整 cmsg。
	recvmsg 仍成功，数据完整，msg_flags 置 MSG_CTRUNC，msg_controllen
	可能变成 0 或只包含不完整片段，不能再按 cmsg 解析。
	*/
	sent = send_datagram(sfd.get(), raddr, part2, false);
	std::println("[MSG_CTRUNC] 发送 {} 字节，控制缓冲只给 16 字节", sent);

	std::array<char, 64> databuf{};
	n = recv_one(rfd.get(), databuf, 16, flags, from, from_len, cbuf, msg);
	std::println("[MSG_CTRUNC] 数据返回 {} 字节，msg_flags=0x{:x}（MSG_CTRUNC={}），msg_controllen={}",
		     n, flags, (flags & MSG_CTRUNC) ? 1 : 0, msg.msg_controllen);
	/*
	CMSG_FIRSTHDR 在剩余空间不足一个 cmsghdr 时返回 nullptr，所以这里
	不调用 dump_cmsgs；正确做法是按需用 CMSG_SPACE 放大控制缓冲。
	*/

	std::println("全部演示结束");
	return 0;
}
