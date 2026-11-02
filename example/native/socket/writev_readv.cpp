/*
writev / readv —— 一次系统调用提交多段内存（scatter/gather）

这个接口是什么、解决什么问题
	writev() 把 iovec 数组描述的 n 段内存按顺序拼接，写到文件描述符上；
	readv() 反过来，把读到的字节依次填进 n 段内存。它们解决的是：
	应用的数据本来就分散在多个缓冲区里（协议头、消息体、尾部各一块），
	如果先 memcpy 拼成一大块再 write，就多一次完整的数据拷贝；用 writev
	可以零拷贝地让内核直接遍历这些段。

	什么时候用：
	- 一条消息由多段内存组成（header + body + trailer），想避免拼接拷贝；
	- 读的时候想把数据直接分散到结构化的多个缓冲区，避免读后再切分。
	不适合：需要带对端地址、辅助数据（cmsg）时用 sendmsg/recvmsg；
	一次要交多个独立消息（UDP 数据报）时用 sendmmsg/recvmmsg。

形态（本接口的关键约束）
	每次调用：n 段内存，n 由调用者运行时决定，1 <= n <= IOV_MAX。
	对应线上：TCP 上这 n 段被内核拼成一个连续的字节流片段（没有消息边界，
	对端无法知道你在哪里切换了 iovec）；普通文件上对应文件当前位置的一段连续区间。
	提交次数：1 次；完成次数：1 次。注意返回的只是一个总字节数，
	内核不告诉你 "每段各写了多少"，必须自己按各段长度累减推算。
	内存：iovec 数组和每段缓冲区都由调用者提供，且必须在该调用返回前保持有效
	（内核在系统调用期间直接读取 iovec 并访问缓冲区）。

需要的内核/硬件条件
	无特殊条件。Linux 的段数上限是 UIO_MAXIOV = 1024（见
	/usr/include/linux/uio.h），glibc 通过 limits.h 暴露为 IOV_MAX = 1024
	（见 /usr/include/bits/uio_lim.h），运行时可用 sysconf(_SC_IOV_MAX) 查询。
	超过上限的 writev 返回 -1 并置 EINVAL。

怎么编译、怎么运行
	g++ -std=c++26 -O2 -Wall -Wextra -o /tmp/native_writev_readv writev_readv.cpp
	/tmp/native_writev_readv [段数]      # 段数默认 5，超过 IOV_MAX 会被拒绝

预期输出（TCP 回环，数值随内核缓冲略有浮动）
	[writev] IOV_MAX = 1024，本次使用 5 段，共 204800 字节
	[writev] 第 1 次调用返回一个小于总量的值（短写，后面继续）
	... 若干次 EAGAIN 后边读边写 ...
	[writev] 完成：... 次调用，... 次短写，... 次 EAGAIN，校验 OK
	[readv] 第一次 readv 返回 1200：本次各段填充 [1000, 200, 0]
	[readv] 第二次 readv 返回 1300：本次各段填充 [0, 800, 500]
	[readv] 数据校验 OK
	[IOV_MAX] writev(iovcnt=1025) 返回 -1，EINVAL（段数超过上限）
*/

#include <arpa/inet.h>
#include <array>
#include <cerrno>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fcntl.h>
#include <netinet/in.h>
#include <poll.h>
#include <print>
#include <signal.h>
#include <span>
#include <string_view>
#include <sys/socket.h>
#include <sys/uio.h>
#include <unistd.h>
#include <utility>
#include <vector>

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
在 127.0.0.1 上建立一对已连接的 TCP 套接字。先 listen 再 connect，
accept 拿到的就是对端；单进程即可完成，不需要 fork。
*/
static std::pair<Fd, Fd> make_tcp_pair()
{
	Fd listener{socket(AF_INET, SOCK_STREAM, 0)};
	if (!listener.valid())
		die("socket(TCP listen)");

	sockaddr_in addr{};
	addr.sin_family = AF_INET;
	addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
	addr.sin_port = 0;			/* 0：内核分配端口 */

	if (bind(listener.get(), reinterpret_cast<sockaddr *>(&addr), sizeof addr) < 0)
		die("bind");
	if (listen(listener.get(), 1) < 0)
		die("listen");

	socklen_t alen = sizeof addr;
	if (getsockname(listener.get(), reinterpret_cast<sockaddr *>(&addr), &alen) < 0)
		die("getsockname");

	Fd client{socket(AF_INET, SOCK_STREAM, 0)};
	if (!client.valid())
		die("socket(client)");
	if (connect(client.get(), reinterpret_cast<sockaddr *>(&addr), sizeof addr) < 0)
		die("connect");

	int sfd = accept(listener.get(), nullptr, nullptr);
	if (sfd < 0)
		die("accept");
	Fd server{sfd};
	return {std::move(client), std::move(server)};
}

/* 计算 iovec 数组里还剩下多少字节可写/可读（未被消耗的段长度之和）。 */
static size_t total_left(std::span<const iovec> iov)
{
	size_t sum = 0;
	for (const auto &v : iov)
		sum += v.iov_len;
	return sum;
}

/*
按 "本次写/读成功 n 字节" 推进 iovec 数组：
	- 前面的段被完整消耗，iov_len 置 0；
	- 第一个没被吃完的段，iov_base 前移、iov_len 减去剩余消耗量。
这就是 writev/readv 的返回值只有总数时，唯一能确定 "每段填了多少" 的办法。
返回本次实际推进的字节数（正常情况下等于 n）。
*/
static size_t advance_iov(std::span<iovec> iov, size_t n)
{
	size_t moved = 0;

	for (auto &v : iov) {
		if (n == 0)
			break;
		size_t take = n < v.iov_len ? n : v.iov_len;
		v.iov_base = static_cast<char *>(v.iov_base) + take;
		v.iov_len -= take;
		n -= take;
		moved += take;
	}
	return moved;
}

/* 打印 "若本次成功 n 字节，各段分别填了多少"，用于观察分段关系。 */
static void print_fill(std::span<const iovec> iov, size_t n)
{
	std::print("[");
	for (size_t i = 0; i < iov.size(); i++) {
		size_t take = n < iov[i].iov_len ? n : iov[i].iov_len;
		n -= take;
		if (i)
			std::print(", ");
		std::print("{}", take);
	}
	std::print("]");
}

/* 各段的期望字节值：第 i 段全部填 'A' + i，便于校验段边界。 */
static char segment_fill(size_t index)
{
	return static_cast<char>('A' + index % 26);
}

/*
校验收到的字节流：按原始段长度把流位置映射回段号，检查每个字节是否等于
该段的期望值。TCP 没有消息边界，只能在知道各段长度的前提下这样校验。
*/
static void verify_stream(std::span<const char> data,
				const std::vector<size_t> &seg_lens,
				size_t &pos, size_t &errors)
{
	for (char c : data) {
		size_t acc = 0;
		size_t idx = 0;
		while (idx < seg_lens.size() && pos >= acc + seg_lens[idx])
			acc += seg_lens[idx++];

		char expect = idx < seg_lens.size() ? segment_fill(idx) : '?';
		if (c != expect)
			++errors;
		++pos;
	}
}

/*
writev 演示：
	1. 建立 TCP 回环连接，把发送端设为非阻塞、发送缓冲区调小；
	2. 用 n 段内存（内容各不相同）反复 writev；
	3. 观察短写（返回值小于剩余总字节数），按返回值推进 iovec 后继续；
	4. 写不进去（EAGAIN）时从对端读走数据，腾出缓冲区再续写；
	5. 最后校验对端收到的字节流。
*/
static void demo_writev(unsigned int nseg)
{
	auto [cli, srv] = make_tcp_pair();

	/*
	把发送缓冲区调小，好让 176 KB 这种数据必定一次写不完，从而稳定地
	演示 "短写"。SO_SNDBUF 的实际值由内核加倍并夹到最小值，这里只关心
	它足够小；getsockopt 可以看到实际值（未在本示例中打印）。
	*/
	int sndbuf = 4096;
	if (setsockopt(srv.get(), SOL_SOCKET, SO_SNDBUF, &sndbuf, sizeof sndbuf) < 0)
		die("setsockopt(SO_SNDBUF)");

	/*
	设为非阻塞：写满缓冲区后 writev 返回 -1/EAGAIN，而不是把线程挂住。
	本演示是单线程，发送端阻塞而没人读对端就会死锁，所以必须非阻塞，
	并在 EAGAIN 时主动去对端把数据读走。
	*/
	int fl = fcntl(srv.get(), F_GETFL, 0);
	if (fl < 0 || fcntl(srv.get(), F_SETFL, fl | O_NONBLOCK) < 0)
		die("fcntl(O_NONBLOCK)");

	/* 第 i 段长度不同，总量约 200 KB，确保远大于发送缓冲区 */
	std::vector<size_t> seg_lens;
	seg_lens.reserve(nseg);
	for (unsigned int i = 0; i < nseg; i++)
		seg_lens.push_back(32768 + static_cast<size_t>(i) * 4096);

	/* 每段一块独立内存；必须先把 vector 建好再取 data()，避免扩容失效 */
	std::vector<std::vector<char>> segs;
	segs.reserve(nseg);
	for (unsigned int i = 0; i < nseg; i++)
		segs.emplace_back(seg_lens[i], segment_fill(i));

	std::vector<iovec> iov;
	iov.reserve(nseg);
	size_t total = 0;
	for (unsigned int i = 0; i < nseg; i++) {
		iov.push_back(iovec{segs[i].data(), segs[i].size()});
		total += segs[i].size();
	}
	std::println("[writev] IOV_MAX = {}，本次使用 {} 段，共 {} 字节",
				sysconf(_SC_IOV_MAX), nseg, total);

	/*
	校验状态：pos 是已从对端读到的字节数。注意 pos 同时也是全局流位置，
	用它和 seg_lens 就能还原每个字节属于哪一段。
	*/
	size_t pos = 0;
	size_t errors = 0;
	std::array<char, 65536> rbuf{};

	/*
	从对端 "非阻塞" 地读干当前可读数据。返回读到的字节数；0 表示暂时没有
	数据（EAGAIN），不是 EOF——发送端还在写。
	*/
	auto drain_peer = [&]() {
		size_t got = 0;
		for (;;) {
			ssize_t r = recv(cli.get(), rbuf.data(), rbuf.size(), MSG_DONTWAIT);
			if (r > 0) {
				verify_stream(std::span{rbuf.data(), static_cast<size_t>(r)},
							seg_lens, pos, errors);
				got += static_cast<size_t>(r);
				continue;
			}
			if (r < 0 && errno == EINTR)
				continue;
			if (r < 0 && (errno == EAGAIN || errno == EWOULDBLOCK))
				break;
			if (r == 0)
				break;		/* 对端关闭，本演示中不会发生 */
			die("recv(drain)");
		}
		return got;
	};

	size_t calls = 0;
	size_t short_writes = 0;
	size_t eagains = 0;
	size_t written = 0;

	/*
	主循环：只要 iovec 里还有字节就继续 writev。
	每次 writev 传的是 "当前尚未写出的段"，段数始终 <= IOV_MAX。
	*/
	for (;;) {
		size_t left = total_left(iov);
		if (left == 0)
			break;
		/*
		writev 的返回值：
			>0 : 本次写出的总字节数，可能小于 left（短写），也可能正好写满；
			-1 : 错误。EAGAIN/EWOULDBLOCK 表示非阻塞发送缓冲区已满；
				EINTR 表示被信号打断，应重试；EPIPE 表示对端已关闭
				（此时若没忽略 SIGPIPE，进程会先被信号杀死）。
		注意它绝不会告诉你每段各写了多少——下面用 print_fill 自己算。
		*/
		ssize_t r = writev(srv.get(), iov.data(), static_cast<int>(iov.size()));
		++calls;

		if (r < 0) {
			if (errno == EINTR)
				continue;
			if (errno == EAGAIN || errno == EWOULDBLOCK) {
				++eagains;
				/*
				缓冲区满：先读走对端数据（同时校验），再等发送端
				重新可写。只忙轮询会空转很多次，poll(POLLOUT) 更贴近
				真实的非阻塞写法。100 ms 只是演示用的上限。
				*/
				drain_peer();
				pollfd pfd{srv.get(), POLLOUT, 0};
				if (poll(&pfd, 1, 100) < 0 && errno != EINTR)
					die("poll(POLLOUT)");
				continue;
			}
			die("writev");
		}
		if (r == 0) {
			/* 所有剩余段长度都为 0 时才会出现；正常推进后不会发生 */
			break;
		}

		size_t n = static_cast<size_t>(r);
		if (calls <= 3 || n < left) {
			std::print("[writev] 第 {} 次调用返回 {}：本次各段填充 ", calls, n);
			print_fill(iov, n);
			std::println("，剩余 {} 字节", left - n);
		}
		/*
		关键步骤：writev 只返回总数，用 advance_iov 按各段长度推进。
		如果忘了推进，下一轮会把同一批数据从头再写一遍。
		*/
		advance_iov(iov, n);
		written += n;

		if (n < left) {
			++short_writes;
			/*
			短写不是错误：TCP 发送缓冲区就那么大，内核先写进去一部分。
			正确做法就是像这里一样更新 iovec 后继续写剩余部分。
			*/
		}
	}

	/* 发送端已全部写入内核；现在把剩余在途数据读完，直到收齐 total 字节 */
	while (pos < total) {
		ssize_t r = recv(cli.get(), rbuf.data(), rbuf.size(), 0);
		if (r < 0) {
			if (errno == EINTR)
				continue;
			die("recv(final drain)");
		}
		if (r == 0)
			die("对端提前关闭");
		verify_stream(std::span{rbuf.data(), static_cast<size_t>(r)},
					seg_lens, pos, errors);
	}

	std::println("[writev] 完成：写出 {} 字节，{} 次调用，{} 次短写，{} 次 EAGAIN，收到 {} 字节，校验 {}",
				written, calls, short_writes, eagains, pos,
				errors == 0 ? "OK" : "失败");

	/*
	边界演示：段数超过 IOV_MAX 时内核直接拒绝。这里用 IOV_MAX+1 个
	零长度 iovec（内核在检查段数时不会解引用它们）验证 EINVAL。
	*/
	long iov_max = sysconf(_SC_IOV_MAX);
	std::vector<iovec> too_many(static_cast<size_t>(iov_max) + 1, iovec{nullptr, 0});
	errno = 0;
	ssize_t r = writev(srv.get(), too_many.data(),
				static_cast<int>(too_many.size()));
	std::println("[IOV_MAX] writev(iovcnt={}) 返回 {}，errno={}（{}）",
				too_many.size(), r, errno, std::strerror(errno));
}

/*
readv 演示：两次读取，观察 readv 返回值如何分摊到各段。
	第一次：对端已有 1200 字节，但请求 3 段共 3000 字节；
			返回 1200 -> 段 0 填满 1000，段 1 填 200，段 2 为 0。
	第二次：对端再发 1300 字节；
			返回 1300 -> 段 1 再填 800（填满），段 2 填 500。
TCP 是字节流，readv 只保证 "有多少给多少"，不保证填满所有段，
所以必须按返回值推进 iovec 并循环读。
*/
static void demo_readv()
{
	auto [cli, srv] = make_tcp_pair();

	std::array<std::array<char, 1000>, 3> bufs{};
	std::array<iovec, 3> iov{};
	for (size_t i = 0; i < iov.size(); i++)
		iov[i] = iovec{bufs[i].data(), bufs[i].size()};

	/* 按全局流位置生成可校验的内容：'0' + 位置 % 10 */
	auto chunk_with_pattern = [](size_t offset, size_t len) {
		std::vector<char> v(len);
		for (size_t i = 0; i < len; i++)
			v[i] = static_cast<char>('0' + (offset + i) % 10);
		return v;
	};

	size_t received = 0;
	size_t errors = 0;
	auto recv_stage = [&](size_t want, std::string_view label) {
		while (want > 0) {
			/*
			readv 的返回值同样只是一个总字节数：
				>0 : 本次读进各段的总字节数，可能没有填满所有段；
				0 : 对端有序关闭（EOF），TCP 上表示不会再有数据；
				-1 : 错误，EINTR 重试，EAGAIN 表示暂时无数据。
			*/
			ssize_t r = readv(cli.get(), iov.data(), static_cast<int>(iov.size()));
			if (r < 0) {
				if (errno == EINTR)
					continue;
				die("readv");
			}
			if (r == 0)
				die("readv 收到 EOF");

			size_t n = static_cast<size_t>(r);
			std::print("[readv] {} readv 返回 {}：本次各段填充 ", label, n);
			print_fill(iov, n);
			std::println("，累计 {} 字节", received + n);

			/*
			校验并按返回值推进 iovec。数据是按段顺序落进各段的：
			第 i 段拿到 min(剩余字节数, 该段长度) 个字节，位置从
			iov[i].iov_base 开始（这个指针已经随着之前的部分读前移过）。
			*/
			{
				size_t off = received;
				size_t left = n;
				for (const auto &v : iov) {
					size_t take = left < v.iov_len ? left : v.iov_len;
					const char *p = static_cast<const char *>(v.iov_base);
					for (size_t i = 0; i < take; i++) {
						if (p[i] != static_cast<char>('0' + (off + i) % 10))
							++errors;
					}
					off += take;
					left -= take;
					if (left == 0)
						break;
				}
			}

			advance_iov(iov, n);
			received += n;
			want -= n;
		}
	};

	/* 第一次：只发 1200 字节，远小于 readv 请求的 3000 字节 */
	auto first = chunk_with_pattern(0, 1200);
	ssize_t s = send(srv.get(), first.data(), first.size(), MSG_NOSIGNAL);
	if (s != static_cast<ssize_t>(first.size()))
		die("send(stage 1)");
	recv_stage(first.size(), "第一次");

	/* 第二次：再发 1300 字节，把段 1 剩余部分和段 2 一起填上 */
	auto second = chunk_with_pattern(1200, 1300);
	s = send(srv.get(), second.data(), second.size(), MSG_NOSIGNAL);
	if (s != static_cast<ssize_t>(second.size()))
		die("send(stage 2)");
	recv_stage(second.size(), "第二次");

	std::println("[readv] 共收到 {} 字节，数据校验 {}", received,
				errors == 0 ? "OK" : "失败");
}

int main(int argc, char **argv)
{
	/*
	writev/readv 没有 flags 参数，无法像 send 那样用 MSG_NOSIGNAL 避免
	SIGPIPE，因此写套接字前通常全局忽略 SIGPIPE，改为检查 EPIPE。
	*/
	signal(SIGPIPE, SIG_IGN);

	/*
	段数由运行时决定，默认 5。上限是 IOV_MAX；超出就直接报错，
	不把非法参数送进内核。
	*/
	unsigned long nseg = 5;
	long iov_max = sysconf(_SC_IOV_MAX);
	if (argc > 1) {
		char *end = nullptr;
		nseg = std::strtoul(argv[1], &end, 10);
		if (end == argv[1] || *end != '\0' || nseg == 0) {
			std::fprintf(stderr, "用法: %s [段数 1..%ld]\n", argv[0], iov_max);
			return EXIT_FAILURE;
		}
	}
	if (nseg > static_cast<unsigned long>(iov_max)) {
		std::fprintf(stderr, "段数 %lu 超过 IOV_MAX=%ld\n", nseg, iov_max);
		return EXIT_FAILURE;
	}

	demo_writev(static_cast<unsigned int>(nseg));
	demo_readv();

	std::println("全部演示结束");
	return 0;
}
