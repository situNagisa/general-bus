/*
sendmmsg / recvmmsg —— 一次系统调用收发一批数据报

这个接口是什么、解决什么问题
	sendmmsg() 一次提交 vlen 个消息描述符（struct mmsghdr），每个描述符就是
	一个完整的 msghdr（地址、iovec、控制信息）+ 一个输出字段 msg_len；
	recvmmsg() 反过来一次收取最多 vlen 个数据报，把每个数据报的长度写回
	对应的 msg_len。它们解决的是 "高包速率下每个包一次系统调用" 的开销：
	1000 个小包用 sendmmsg 只陷入内核一次，省掉 999 次系统调用和上下文切换。
	这是 UDP 服务器/转发器的标准批量接口。

	什么时候用：
	- UDP 上要成批发送或接收大量小数据报，且能接受 "批" 这种提交粒度；
	- 想用一个调用控制 "最多收 vlen 个"，并可用 MSG_WAITFORONE 只等第一个。
	不适合：TCP（字节流没有消息边界，没有 "一批消息" 的概念）；
	要真正异步/零系统调用时用 io_uring 的 SENDMSG/RECVMSG 批量提交。

形态（本接口的关键约束）
	sendmmsg：一次调用提交 k 个消息，每个消息 = 1 个 UDP 数据报（或 TCP 上
				一段字节流）；1 次提交，1 次完成（整批）。
	recvmmsg：一次调用提交最多 vlen 个接收缓冲，实际取走 k 个数据报；
				每个数据报的结果（字节数）分别写回各自 mmsghdr 的 msg_len。
	内存：每个 mmsghdr 里的 iovec、数据缓冲、地址、控制缓冲都由调用者提供，
				且整批必须在调用返回前保持有效；内核不会替你保存这些指针。
	返回值：成功时返回 k（0 <= k <= vlen），表示 "前 k 个消息有效/完成"。
				第 j 个（j > 0，从 0 数）出错时，本次调用返回 j：
				- sendmmsg 的错误码直接丢弃（本次已算成功）；
				- recvmmsg 会把该错误存进套接字的错误字段，下一次调用开始时
					直接把它报出来（本次仍返回 j）。本机 6.18 实测如此，
					内核源码 net/socket.c 中 recvmmsg 在部分成功后保存非 EAGAIN
					错误，下一次调用在收包前返回它。
				一个消息都没完成时，调用返回 -1 并置 errno。

需要的内核/硬件条件
	Linux 3.0+ 提供 sendmmsg()/recvmmsg()；MSG_WAITFORONE 是 Linux 2.6.34+
	的 GNU 扩展，glibc 在 _GNU_SOURCE 下暴露（g++ 默认定义 _GNU_SOURCE）。
	无特殊硬件，IPv4 回环即可。

怎么编译、怎么运行
	g++ -std=c++26 -O2 -Wall -Wextra -o /tmp/native_sendmmsg_recvmmsg sendmmsg_recvmmsg.cpp
	/tmp/native_sendmmsg_recvmmsg

预期输出（端口、耗时每次不同）
	[setup] 接收端 127.0.0.1:XXXX
	[sendmmsg] 提交 4 个，返回 4；msg_len = 8 12 16 20
	[recvmmsg] vlen=4，返回 4；依次收到 8 12 16 20 字节
	[MSG_WAITFORONE] 子进程 300 ms 后发 3 个，recvmmsg 在约 300 ms 后返回 3（vlen=8，不会等满）
	[sendmmsg 错误] 第 3 个消息 iovec 非法：单独 sendmsg 得到 EFAULT
	[sendmmsg 错误] 批量调用返回 2，errno=0（错误码被丢弃），msg_len = 4 4 0 0
	[sendmmsg 错误] 紧接着的正常批量返回 2（上次错误未留存）
	[recvmmsg 错误] 第一次返回 1（前 1 个有效），errno 未被设置
	[recvmmsg 错误] 下一次调用直接返回 -1，errno=EMSGSIZE（错误被存下并延后报告）
	全部演示结束
*/

#include <arpa/inet.h>
#include <array>
#include <cerrno>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <netinet/in.h>
#include <print>
#include <span>
#include <string_view>
#include <sys/socket.h>
#include <sys/time.h>
#include <sys/wait.h>
#include <time.h>
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
准备一对 UDP 套接字：接收端绑定 127.0.0.1:0，发送端 connect 到它。
返回 {接收端, 发送端}。
*/
static std::pair<Fd, Fd> make_udp_pair(sockaddr_in &raddr)
{
	Fd rfd{socket(AF_INET, SOCK_DGRAM, 0)};
	if (!rfd.valid())
		die("socket(recv)");

	raddr = sockaddr_in{};
	raddr.sin_family = AF_INET;
	raddr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
	raddr.sin_port = 0;
	if (bind(rfd.get(), reinterpret_cast<sockaddr *>(&raddr), sizeof raddr) < 0)
		die("bind");
	socklen_t len = sizeof raddr;
	if (getsockname(rfd.get(), reinterpret_cast<sockaddr *>(&raddr), &len) < 0)
		die("getsockname");

	Fd sfd{socket(AF_INET, SOCK_DGRAM, 0)};
	if (!sfd.valid())
		die("socket(send)");
	if (connect(sfd.get(), reinterpret_cast<sockaddr *>(&raddr), sizeof raddr) < 0)
		die("connect");
	return {std::move(rfd), std::move(sfd)};
}

/* 把接收队列里现成的数据报全部取走（非阻塞），用于让演示从干净状态开始。 */
static void drain_all(int fd)
{
	std::array<char, 2048> tmp{};
	for (;;) {
		ssize_t n = recv(fd, tmp.data(), tmp.size(), MSG_DONTWAIT);
		if (n > 0)
			continue;
		if (n < 0 && errno == EINTR)
			continue;
		break;		/* EAGAIN 或错误 */
	}
}

/*
用 4 个消息演示 mmsghdr 的构造与 sendmmsg 的批量返回。
	mmsghdr = msghdr（地址/iovec/控制信息） + unsigned msg_len（输出）
	每个 mmsghdr 都描述一个独立的数据报；段数、内容可以各不相同。
*/
static void demo_normal_batch(int sfd, int rfd)
{
	constexpr size_t n = 4;
	std::array<std::array<char, 64>, n> payload{};
	std::array<iovec, n> iov{};
	std::array<mmsghdr, n> mmsg{};

	for (size_t i = 0; i < n; i++) {
		/* 生成不同长度的数据，方便在 msg_len 上看到区别 */
		int len = std::snprintf(payload[i].data(), payload[i].size(),
					"batch-%zu", i);
		size_t want = 8 + i * 4;
		while (static_cast<size_t>(len) < want)
			payload[i][len++] = '#';

		iov[i].iov_base = payload[i].data();
		iov[i].iov_len = static_cast<size_t>(len);

		/*
		mmsg[i].msg_hdr 就是一个普通 msghdr：
		- 发送端已 connect，所以 msg_name/msg_namelen 留空（NULL/0）；
		若未 connect，就在这里填每个数据报自己的目标地址；
		- msg_iov/msg_iovlen 描述这个数据报的内存段；
		- msg_control/msg_controllen 可选地放 cmsg。
		msg_len 是 recvmmsg/sendmmsg 写回结果的地方，调用前不用初始化。
		*/
		mmsg[i].msg_hdr.msg_iov = &iov[i];
		mmsg[i].msg_hdr.msg_iovlen = 1;
	}

	/*
	sendmmsg 的返回值：成功发出的消息个数（这里是 4），不是字节数！
	每个消息实际发出的字节数写回各自的 msg_len。
	对 UDP，每个消息要么完整发出，要么这个位置失败；不存在 "写一半"。
	*/
	int k = sendmmsg(sfd, mmsg.data(), n, 0);
	if (k < 0)
		die("sendmmsg");
	std::print("[sendmmsg] 提交 {} 个，返回 {}；msg_len = ", n, k);
	for (int i = 0; i < k; i++)
		std::print("{}{}", i ? " " : "", mmsg[i].msg_len);
	std::println("");

	/*
	接收端：vlen 是本次最多收几个消息，不是每个消息的 iovec 段数。
	本机实测 recvmmsg 传 vlen=2000 不会报 EINVAL；而 sendmmsg 会把
	vlen 截到 UIO_MAXIOV（传 2000 只发 1024 个并返回 1024）——这是
	两个接口容易忽略的不对称。这里刚好提交 4 个，队列里有 4 个，
	一次调用收齐；若 vlen 大于当前可收数量且没有 MSG_WAITFORONE，
	recvmmsg 会继续阻塞等满 vlen 个（本机实测），所以批量收包通常配
	MSG_WAITFORONE 或刚好等于预期的包数。
	*/
	std::array<std::array<char, 64>, n> rbuf{};
	std::array<iovec, n> riov{};
	std::array<mmsghdr, n> rmmsg{};
	for (size_t i = 0; i < n; i++) {
		riov[i].iov_base = rbuf[i].data();
		riov[i].iov_len = rbuf[i].size();
		rmmsg[i].msg_hdr.msg_iov = &riov[i];
		rmmsg[i].msg_hdr.msg_iovlen = 1;
	}

	/*
	recvmmsg 的 timeout 参数：NULL 表示一直等。man 2 recvmmsg 说非 NULL 时
	到期且一个都没收到会返回 -1/EAGAIN，但本机 6.18 实测：队列为空时即使
	给了 100 ms 超时，第一次阻塞 recv 仍不会返回（未核对与 man 的差异），
	所以本示例不用 timeout 控制首次等待，而是让 vlen 与队列里的包数一致。
	*/
	k = recvmmsg(rfd, rmmsg.data(), n, 0, nullptr);
	if (k < 0)
		die("recvmmsg");
	std::print("[recvmmsg] vlen={}，返回 {}；依次收到 ", n, k);
	for (int i = 0; i < k; i++) {
		std::print("{}{}", i ? " " : "", rmmsg[i].msg_len);
	}
	std::println(" 字节");
	for (int i = 0; i < k; i++) {
		/*
		msg_len 是每个数据报实际收到的字节数；如果该消息的缓冲比数据报小，
		只有这个数据报会被截断，msg_flags 里置 MSG_TRUNC，并且不会影响
		同一批里其它消息。这里缓冲足够，长度和发送时一致。
		*/
		std::println("[recvmmsg]   #{} len={} 内容=\"{}\"", i,
					rmmsg[i].msg_len,
					std::string_view{rbuf[i].data(), rmmsg[i].msg_len});
	}
}

/*
MSG_WAITFORONE 演示：
	先清空接收队列，然后 fork 一个子进程，睡 300 ms 再连发 3 个数据报。
	父进程用 vlen=8 + MSG_WAITFORONE 调 recvmmsg：
		- 队列为空时阻塞，直到第一个数据报到达（约 300 ms）；
		- 第一个到达后，内核对本次调用剩下的消息自动加上 MSG_DONTWAIT，
		所以只会取走 "此刻已经排队" 的数据报，不会为了凑满 8 个继续等。
	这正是高吞吐服务器常用的 "睡到有包，然后尽量多收" 模式。
*/
static void demo_waitforone(int sfd, int rfd)
{
	drain_all(rfd);

	pid_t pid = fork();
	if (pid < 0)
		die("fork");

	if (pid == 0) {
		/*
		子进程：只做发送。注意 fork 后不要用 std::print/iostream
		（缓冲区是复制的，会重复输出），退出用 _exit 避免跑父进程的析构。
		*/
		timespec ts{0, 300'000'000};	/* 300 ms */
		nanosleep(&ts, nullptr);
		const char *msgs[3] = {"wait-0", "wait-1", "wait-22"};
		for (const char *m : msgs)
			(void)send(sfd, m, std::strlen(m), 0);
		_exit(0);
	}

	constexpr size_t vlen = 8;
	std::array<std::array<char, 64>, vlen> rbuf{};
	std::array<iovec, vlen> riov{};
	std::array<mmsghdr, vlen> rmmsg{};
	for (size_t i = 0; i < vlen; i++) {
		riov[i].iov_base = rbuf[i].data();
		riov[i].iov_len = rbuf[i].size();
		rmmsg[i].msg_hdr.msg_iov = &riov[i];
		rmmsg[i].msg_hdr.msg_iovlen = 1;
	}

	/*
	这里传 timeout=NULL：队列为空时一直阻塞，直到第一个数据报到达；
	子进程保证 300 ms 后会发送，因此不会永久挂住。
	MSG_WAITFORONE 的值是 0x10000（glibc bits/socket.h），属于 GNU 扩展。
	*/
	auto t0 = std::chrono::steady_clock::now();
	int k = recvmmsg(rfd, rmmsg.data(), vlen, MSG_WAITFORONE, nullptr);
	auto dt = std::chrono::duration<double, std::milli>(
		std::chrono::steady_clock::now() - t0);
	if (k < 0)
		die("recvmmsg(MSG_WAITFORONE)");

	std::print("[MSG_WAITFORONE] 子进程 300 ms 后发 3 个，recvmmsg 在 {:.0f} ms 后返回 {}（vlen={}，不等满）",
			dt.count(), k, vlen);
	if (k > 1) {
		std::print("；长度 ");
		for (int i = 0; i < k; i++)
			std::print("{}{}", i ? " " : "", rmmsg[i].msg_len);
	}
	std::println("");

	int status = 0;
	if (waitpid(pid, &status, 0) < 0)
		die("waitpid");
	if (!WIFEXITED(status) || WEXITSTATUS(status) != 0)
		std::println("[MSG_WAITFORONE] 警告：子进程发送失败");
}

/*
sendmmsg 的第 j 个消息出错：
	让第 3 个消息（下标 2）的 iovec 指向非法地址，内核在发送它时返回 EFAULT。
	由于前 2 个已经成功，sendmmsg 返回 2：前 2 个有效，第 3 个及以后不再尝试。
	错误码不会被返回，也不会在下一次调用报出——man 2 sendmmsg 明确说错误会丢失。
*/
static void demo_sendmmsg_error(int sfd)
{
	constexpr size_t n = 4;
	std::array<std::array<char, 32>, n> payload{};
	std::array<iovec, n> iov{};
	std::array<mmsghdr, n> mmsg{};

	for (size_t i = 0; i < n; i++) {
		int len = std::snprintf(payload[i].data(), payload[i].size(),
					"ok-%zu", i);
		iov[i].iov_base = payload[i].data();
		iov[i].iov_len = static_cast<size_t>(len);
		mmsg[i].msg_hdr.msg_iov = &iov[i];
		mmsg[i].msg_hdr.msg_iovlen = 1;
	}

	/* 先单独发这个坏消息，好知道它到底为什么失败 */
	iov[2].iov_base = reinterpret_cast<void *>(-1);
	iov[2].iov_len = 8;
	errno = 0;
	ssize_t one = sendmsg(sfd, &mmsg[2].msg_hdr, 0);
	std::println("[sendmmsg 错误] 单独 sendmsg 坏消息返回 {}，errno={}（{}）",
				one, errno, std::strerror(errno));

	/*
	不修复第 3 个消息：保持 iov_base 非法，这样只有它失败；
	前 2 个消息正常发出，正好演示 "前 j 个有效" 的返回语义。
	*/
	errno = 0;
	int k = sendmmsg(sfd, mmsg.data(), n, 0);
	std::println("[sendmmsg 错误] 批量调用返回 {}，errno={}（{}），msg_len = {} {} {} {}",
				k, errno, std::strerror(errno),
				mmsg[0].msg_len, mmsg[1].msg_len, mmsg[2].msg_len,
				mmsg[3].msg_len);
	/*
	观察点：
	- 返回 2 表示前 2 个消息有效，第 3 个失败了；
	- errno 仍然是调用前设置的 0：错误码被丢弃，无法得知是 EFAULT；
	- 第 3、4 个的 msg_len 为 0（内核没有处理它们）。
	如果第 0 个就失败（k=0），sendmmsg 会返回 -1 并把 errno 留给你。
	*/

	/*
	对照实验：紧接着再发一批正常消息。返回 2 说明上次的错误没有留在
	套接字上——sendmmsg 丢弃错误，而 recvmmsg 会把错误保存到下一次调用。
	*/
	std::array<std::array<char, 32>, 2> p2{};
	std::array<iovec, 2> i2{};
	std::array<mmsghdr, 2> m2{};
	for (size_t i = 0; i < 2; i++) {
		int len = std::snprintf(p2[i].data(), p2[i].size(), "again-%zu", i);
		i2[i].iov_base = p2[i].data();
		i2[i].iov_len = static_cast<size_t>(len);
		m2[i].msg_hdr.msg_iov = &i2[i];
		m2[i].msg_hdr.msg_iovlen = 1;
	}
	k = sendmmsg(sfd, m2.data(), 2, 0);
	std::println("[sendmmsg 错误] 紧接着的正常批量返回 {}（上次错误未留存）", k);
}

/*
recvmmsg 的第 j 个消息出错（j > 0）：
	第 0 个正常收一个数据报，第 1 个的 msg_iovlen 超过 IOV_MAX，内核在
	处理它时返回 EMSGSIZE。于是：
		- 第一次调用返回 1：前 1 个有效，本次不失败；
		- 内核把这个非 EAGAIN 错误存进套接字的错误字段；
		- 下一次 recvmmsg 在收包前直接返回 -1/EMSGSIZE，即使队列里还有包。
	本机 6.18 实测；man 2 recvmmsg 也写明 "错误码预期在下一次调用中返回"。
*/
static void demo_recvmmsg_error(int sfd, int rfd)
{
	drain_all(rfd);

	/* 队列里放一个数据报，保证第 0 个消息能成功 */
	constexpr std::string_view probe = "err-first";
	if (send(sfd, probe.data(), probe.size(), 0) < 0)
		die("send(err demo)");

	std::array<char, 64> buf0{};
	std::array<char, 64> buf1{};
	std::array<iovec, 2> iov{};
	std::array<mmsghdr, 2> mmsg{};

	iov[0].iov_base = buf0.data();
	iov[0].iov_len = buf0.size();
	mmsg[0].msg_hdr.msg_iov = &iov[0];
	mmsg[0].msg_hdr.msg_iovlen = 1;

	iov[1].iov_base = buf1.data();
	iov[1].iov_len = buf1.size();
	mmsg[1].msg_hdr.msg_iov = &iov[1];
	/*
	故意超过 IOV_MAX。内核在 import_iovec 阶段检查段数并返回 EMSGSIZE，
	不会去解引用 iov[1]（这里仍给了合法指针，避免演示程序自身出错）。
	*/
	mmsg[1].msg_hdr.msg_iovlen = sysconf(_SC_IOV_MAX) + 1;

	errno = 0;
	int k = recvmmsg(rfd, mmsg.data(), 2, 0, nullptr);
	std::println("[recvmmsg 错误] 第一次调用返回 {}，errno={}，msg_len[0]={}",
				k, errno, mmsg[0].msg_len);

	/*
	第二次调用：内核先检查套接字的错误字段，直接返回上次保存的 EMSGSIZE。
	注意此时第 0 个消息本来还能收到数据，但调用整体失败，收包结果不可用。
	*/
	errno = 0;
	k = recvmmsg(rfd, mmsg.data(), 2, 0, nullptr);
	std::println("[recvmmsg 错误] 下一次调用返回 {}，errno={}（{}）",
				k, errno, std::strerror(errno));
}

int main()
{
	sockaddr_in raddr{};
	auto [rfd, sfd] = make_udp_pair(raddr);
	std::println("[setup] 接收端 127.0.0.1:{}", ntohs(raddr.sin_port));

	demo_normal_batch(sfd.get(), rfd.get());
	demo_waitforone(sfd.get(), rfd.get());

	/*
	recvmmsg 的错误会 "粘" 在套接字错误字段上，所以先用一个干净的队列
	演示它；sendmmsg 的错误演示放在最后，它成功发出的 2 个数据报会留在
	接收队列里，但此后不再使用这对套接字。
	*/
	demo_recvmmsg_error(sfd.get(), rfd.get());
	demo_sendmmsg_error(sfd.get());

	std::println("全部演示结束");
	return 0;
}
