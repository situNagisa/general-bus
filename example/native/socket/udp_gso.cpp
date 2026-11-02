/*
* udp_gso.cpp —— UDP GSO（Generic Segmentation Offload，UDP_SEGMENT）
*
* 一、这个接口是什么
*   UDP GSO 让用户态一次 send()/sendmsg() 交一整块 UDP 载荷给内核，由内核在
*   发送路径上按指定大小（gso_size，也叫“段大小”）切成多个独立的 UDP 数据报。
*   每个切出来的数据报都有自己完整的 UDP 头和 IP 头，在网络上就是普通的 UDP 包；
*   对端不需要知道发送端用过 GSO。
*
* 二、解决什么问题
*   原来要发 N 个 UDP 包，用户态必须调 N 次 send，每次都要陷入内核、查路由、
*   组装 skb。GSO 把 N 次系统调用变成 1 次，并让网卡（支持 NETIF_F_GSO_UDP_L4 时）
*   直接对一个“大 skb”做硬件分段，显著降低大流量 UDP（QUIC、视频、日志转发等）的
*   每包开销。
*
* 三、形态（一次调用交几段内存、对应几个线上单元、提交几次、完成几次）
*   - 内存：用户提供一块连续缓冲区；内核在发送时复制/引用它，send 返回后用户即可复用。
*   - 一次 send 提交 1 个“GSO skb”，内核按 gso_size 切成 N 个 UDP 数据报（N 个线上单元）。
*   - 提交 1 次系统调用；UDP 无连接，没有“完成”通知，send 返回即表示已交给协议栈。
*   - 接收端仍然是普通 recv，每个数据报单独返回，收 N 次。
*
* 四、需要的内核/硬件条件
*   - 本机头文件 /usr/include/linux/udp.h 定义了 UDP_SEGMENT=103（这就是本机依据）。
*   - 出口设备需要支持 UDP GSO（NETIF_F_GSO_UDP_L4）；不支持时内核在软件里分段，
*     功能仍然正确，只是省不到驱动的分段开销。回环设备支持。
*   - 本机实测：WSL2，内核 6.18.33.2，IPv4 回环可用。
*
* 五、限制（本机内核 6.18 源码 + 实测，均以 IPv4 为例）
*   - payload 总长 <= 65507（65535 - 20 字节 IP 头 - 8 字节 UDP 头），65508 返回 EMSGSIZE。
*   - 段数 <= UDP_MAX_SEGMENTS：6.18 是 128（1<<7，本机实测 128 段成功、129 段 EINVAL）。
*     2024 年以前的历史值是 64（未核对），旧内核上超过 64 段会返回 EINVAL。
*     不要凭“最多 64 段”写死。
*   - 单段 IP 包必须放得进出口 MTU：gso_size + 28 <= MTU，否则 send 返回 EMSGSIZE。
*     本机 127.0.0.1 实际路由到 loopback0（MTU 1500），所以 gso_size 最大 1472。
*     普通机器上 lo 的 MTU 通常是 65536（未核对），这个限制几乎不会碰到。
*   - 最后一段可以更短；载荷 <= gso_size 时整块就是一个数据报，不切。
*   - gso_size 通过 setsockopt 传的是 int；通过 cmsg 传的却是 16 位整数（__u16），
*     且 cmsg_len 必须精确等于 CMSG_LEN(sizeof(__u16))，否则 EINVAL（本机实测）。
*
* 六、编译与运行
*   g++ -std=c++26 -O2 -Wall -Wextra -o /tmp/native_udp_gso udp_gso.cpp
*   /tmp/native_udp_gso
*   预期输出：依次打印 setsockopt 分段、cmsg 分段、128/129 段边界、65507/65508 字节
*   边界、以及按出口 MTU 推出的 gso_size 上限；每个用例给出 recv 到的包个数和每包大小。
*/

#include <arpa/inet.h>
#include <netinet/in.h>
#include <netinet/udp.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <cerrno>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <print>
#include <span>
#include <string_view>
#include <utility>
#include <vector>

// glibc 的头文件没有定义 SOL_UDP（内核文档里常用它）。它和 IPPROTO_UDP 一样是 17，
// 作为 setsockopt 的 level 用。这里显式定义，避免依赖某个 libc 的扩展。
constexpr int kSolUdp = IPPROTO_UDP;

// IPv4 头 20 字节 + UDP 头 8 字节；GSO 的单段长度要加上它才算一个完整的 IP 包。
constexpr std::size_t kIpUdpHeaderLen = 20 + 8;

// ---------------------------------------------------------------------------
// RAII：文件描述符包装。析构自动 close，禁止拷贝，允许移动。
// 示例里所有 socket 都用它管理，异常/提前 return 都不会泄漏 fd。
// ---------------------------------------------------------------------------
class UniqueFd {
public:
	UniqueFd() noexcept = default;
	explicit UniqueFd(int fd) noexcept : fd_(fd) {}

	UniqueFd(const UniqueFd &) = delete;
	UniqueFd &operator=(const UniqueFd &) = delete;

	UniqueFd(UniqueFd &&other) noexcept : fd_(std::exchange(other.fd_, -1)) {}

	UniqueFd &operator=(UniqueFd &&other) noexcept
	{
		if (this != &other)
			reset(std::exchange(other.fd_, -1));
		return *this;
	}

	~UniqueFd() { reset(); }

	[[nodiscard]] int get() const noexcept { return fd_; }

	void reset(int fd = -1) noexcept
	{
		if (fd_ >= 0)
			::close(fd_);
		fd_ = fd;
	}

private:
	int fd_ = -1;
};

// 出错就打印 errno 文本并退出。示例程序不 recovery，直接退出最清楚。
[[noreturn]] void die(std::string_view what)
{
	std::println(stderr, "错误：{}：{}", what, std::strerror(errno));
	std::exit(EXIT_FAILURE);
}

// 设置一个 int 类型的 socket 选项；失败直接退出。
void set_int_option(int fd, int level, int name, int value, std::string_view what)
{
	if (::setsockopt(fd, level, name, &value, sizeof(value)) < 0)
		die(what);
}

// 创建一对连通的 UDP socket：rx 先 bind 到回环的临时端口，tx 再 connect 过去。
// 返回的 tx 已 connect，可以用 send()；rx 已设置 1 秒接收超时，避免演示卡死。
struct UdpPair {
	UniqueFd tx;
	UniqueFd rx;
};

UdpPair make_loopback_pair()
{
	UdpPair pair;

	pair.rx.reset(::socket(AF_INET, SOCK_DGRAM, 0));
	if (pair.rx.get() < 0)
		die("socket(rx)");

	// 只监听回环：sin_addr = 127.0.0.1，端口填 0 让内核选一个空闲端口。
	sockaddr_in addr{};
	addr.sin_family = AF_INET;
	addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
	addr.sin_port = 0;
	if (::bind(pair.rx.get(), reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) < 0)
		die("bind(rx)");

	// 把内核选中的端口读回来，发给 tx 用。
	socklen_t addr_len = sizeof(addr);
	if (::getsockname(pair.rx.get(), reinterpret_cast<sockaddr *>(&addr), &addr_len) < 0)
		die("getsockname(rx)");

	// 接收超时：所有预期数据收完后，用一次 EAGAIN 判断“没有多余数据”。
	timeval timeout{};
	timeout.tv_sec = 1;
	if (::setsockopt(pair.rx.get(), SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout)) < 0)
		die("setsockopt(SO_RCVTIMEO)");

	pair.tx.reset(::socket(AF_INET, SOCK_DGRAM, 0));
	if (pair.tx.get() < 0)
		die("socket(tx)");

	// tx 也 bind 一个回环临时端口，保证源地址明确是 127.0.0.1。
	sockaddr_in local{};
	local.sin_family = AF_INET;
	local.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
	local.sin_port = 0;
	if (::bind(pair.tx.get(), reinterpret_cast<sockaddr *>(&local), sizeof(local)) < 0)
		die("bind(tx)");

	// connect 之后可以 send()，不用每次写目标地址；UDP 的 connect 只是记下默认对端。
	if (::connect(pair.tx.get(), reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) < 0)
		die("connect(tx)");

	return pair;
}

// 填一个可校验的字节图案：接收端按顺序拼起来后可以逐字节比对。
void fill_pattern(std::span<std::byte> buf)
{
	for (std::size_t i = 0; i < buf.size(); ++i)
		buf[i] = static_cast<std::byte>((i * 131u + 7u) & 0xffu);
}

// 计算“总长 total、段大小 seg”时内核应该切出的各段长度。
// 规则：前面每段都是 seg，最后一段是余数（可以为 1..seg 字节）。
std::vector<std::size_t> expected_sizes(std::size_t total, std::size_t seg)
{
	std::vector<std::size_t> sizes;
	while (total > seg) {
		sizes.push_back(seg);
		total -= seg;
	}
	sizes.push_back(total);
	return sizes;
}

// 收包结果：每个数据报的长度，以及按顺序拼起来的内容。
struct RecvResult {
	std::vector<std::size_t> sizes;
	std::vector<std::byte> bytes;
	bool timed_out = false;
};

// 收包，直到收到 expected_total 字节或接收超时。
// 注意：GSO 只在发送端生效，接收端看到的就是 N 个普通 UDP 数据报，必须 recv N 次。
RecvResult recv_all(int fd, std::size_t expected_total)
{
	RecvResult result;
	result.bytes.reserve(expected_total);

	// 单次 recv 的临时缓冲区；每次最多收 64KB（UDP 数据报上限 65507）。
	std::array<std::byte, 65536> scratch{};

	while (result.bytes.size() < expected_total) {
		ssize_t n = ::recv(fd, scratch.data(), scratch.size(), 0);
		if (n < 0) {
			if (errno == EAGAIN || errno == EWOULDBLOCK) {
				result.timed_out = true;
				break;
			}
			die("recv");
		}
		if (n == 0)
			break;

		// recv 返回的是这一个数据报的字节数（不是累计值）。
		result.sizes.push_back(static_cast<std::size_t>(n));
		result.bytes.insert(result.bytes.end(), scratch.begin(), scratch.begin() + n);
	}
	return result;
}

// 打印每个包的大小；包多时只打印首尾若干项，避免刷屏。
void print_sizes(const RecvResult &r, std::string_view title)
{
	std::println("  {}：共 {} 个包，总 {} 字节", title, r.sizes.size(), r.bytes.size());
	constexpr std::size_t kHead = 5;
	constexpr std::size_t kTail = 2;
	if (r.sizes.size() <= kHead + kTail + 1) {
		for (std::size_t i = 0; i < r.sizes.size(); ++i)
			std::println("    [{}] {} 字节", i, r.sizes[i]);
		return;
	}
	for (std::size_t i = 0; i < kHead; ++i)
		std::println("    [{}] {} 字节", i, r.sizes[i]);
	std::println("    ... 省略 {} 个 ...", r.sizes.size() - kHead - kTail);
	for (std::size_t i = r.sizes.size() - kTail; i < r.sizes.size(); ++i)
		std::println("    [{}] {} 字节", i, r.sizes[i]);
}

// 校验接收到的字节与发送图案完全一致。
bool contents_match(const RecvResult &r, std::span<const std::byte> sent)
{
	return r.bytes.size() == sent.size() &&
		std::equal(r.bytes.begin(), r.bytes.end(), sent.begin());
}

// 打印一个用例的结论：段数/每段大小是否符合预期，内容是否一致。
void report_case(std::string_view name, const RecvResult &r, std::span<const std::byte> sent,
			const std::vector<std::size_t> &expected)
{
	bool sizes_ok = r.sizes == expected;
	bool bytes_ok = contents_match(r, sent);
	std::println("  [{}] 预期 {} 个包（{}）", name, expected.size(),
			sizes_ok ? "段大小符合" : "段大小不符");
	if (!sizes_ok) {
		std::println("  实际段大小序列：");
		for (std::size_t s : r.sizes)
			std::print("{} ", s);
		std::println("");
	}
	std::println("  结论：{}，内容{}", sizes_ok ? "分段符合预期" : "分段不符合预期",
			bytes_ok ? "逐字节一致" : "不一致");
}

// 用例 1：setsockopt(SOL_UDP, UDP_SEGMENT) 设置“套接字级”段大小。
// 之后这个 socket 上所有 send 都按这个大小切，直到重新设置为止。
void case_setsockopt()
{
	std::println("\n== 用例 1：setsockopt 设置 UDP_SEGMENT = 1000，一次 send 3500 字节 ==");
	constexpr std::size_t kSeg = 1000;
	constexpr std::size_t kTotal = 3500;

	UdpPair pair = make_loopback_pair();
	int seg = static_cast<int>(kSeg);
	// level 用 SOL_UDP(=17)，optname 用头文件里的 UDP_SEGMENT(=103)，optval 是 int 段大小。
	set_int_option(pair.tx.get(), kSolUdp, UDP_SEGMENT, seg, "setsockopt(UDP_SEGMENT)");

	std::vector<std::byte> payload(kTotal);
	fill_pattern(payload);

	// send 的返回值是“提交的字节数”（3500），不是包数；包数是内核按 seg 算出来的。
	ssize_t n = ::send(pair.tx.get(), payload.data(), payload.size(), 0);
	if (n < 0)
		die("send");
	std::println("  send 返回 {}（整块已交给内核，内核负责切分）", n);

	RecvResult r = recv_all(pair.rx.get(), kTotal);
	print_sizes(r, "接收结果");
	report_case("setsockopt", r, payload, expected_sizes(kTotal, kSeg));
}

// 用例 2：不改 socket 选项，用 cmsg 给“这一次 sendmsg”单独指定段大小。
// 适合偶尔才用 GSO、或者每批段大小不同的场景。
void case_cmsg()
{
	std::println("\n== 用例 2：sendmsg 的 cmsg 指定 UDP_SEGMENT = 1200，一次发 3000 字节 ==");
	constexpr std::uint16_t kSeg = 1200;
	constexpr std::size_t kTotal = 3000;

	UdpPair pair = make_loopback_pair();

	// 先把套接字级 GSO 清零，证明这次分段完全来自 cmsg，而不是残留的 socket 选项。
	set_int_option(pair.tx.get(), kSolUdp, UDP_SEGMENT, 0, "setsockopt(UDP_SEGMENT=0)");

	std::vector<std::byte> payload(kTotal);
	fill_pattern(payload);

	// 控制消息缓冲区必须用 CMSG_SPACE 计算大小，保证对齐。
	std::array<char, CMSG_SPACE(sizeof(std::uint16_t))> control{};
	iovec iov{payload.data(), payload.size()};
	msghdr msg{};
	msg.msg_iov = &iov;
	msg.msg_iovlen = 1;
	msg.msg_control = control.data();
	msg.msg_controllen = control.size();

	cmsghdr *cmsg = CMSG_FIRSTHDR(&msg);
	cmsg->cmsg_level = kSolUdp;
	cmsg->cmsg_type = UDP_SEGMENT;
	// 关键坑：cmsg 里的段大小是 16 位整数，cmsg_len 必须正好是 CMSG_LEN(2)。
	// 传 sizeof(int)（4 字节）时本机内核返回 EINVAL。
	cmsg->cmsg_len = CMSG_LEN(sizeof(std::uint16_t));
	std::memcpy(CMSG_DATA(cmsg), &kSeg, sizeof(kSeg));

	ssize_t n = ::sendmsg(pair.tx.get(), &msg, 0);
	if (n < 0)
		die("sendmsg");
	std::println("  sendmsg 返回 {}", n);

	RecvResult r = recv_all(pair.rx.get(), kTotal);
	print_sizes(r, "接收结果");
	report_case("cmsg", r, payload, expected_sizes(kTotal, kSeg));
}

// 用例 3：段数上限。6.18 的 UDP_MAX_SEGMENTS = 128：
//   datalen > gso_size * UDP_MAX_SEGMENTS 时 send 返回 EINVAL。
void case_segment_limit()
{
	std::println("\n== 用例 3：段数上限（本机 6.18 为 128 段）==");
	constexpr std::uint16_t kSeg = 100;
	constexpr std::size_t kMaxSegs = 128;

	{
		// 128 段：100 * 128 = 12800，刚好在上限内，应当成功。
		UdpPair pair = make_loopback_pair();
		set_int_option(pair.tx.get(), kSolUdp, UDP_SEGMENT, kSeg, "setsockopt(UDP_SEGMENT=100)");

		std::vector<std::byte> payload(kSeg * kMaxSegs);
		fill_pattern(payload);
		ssize_t n = ::send(pair.tx.get(), payload.data(), payload.size(), 0);
		if (n < 0)
			die("send(128 段)");
		RecvResult r = recv_all(pair.rx.get(), payload.size());
		std::println("  128 段 / {} 字节：send 成功，收到 {} 个包", payload.size(), r.sizes.size());
	}

	{
		// 129 段：超过上限，内核在 udp_send_skb 里直接返回 EINVAL，一个包都不会发出。
		UdpPair pair = make_loopback_pair();
		set_int_option(pair.tx.get(), kSolUdp, UDP_SEGMENT, kSeg, "setsockopt(UDP_SEGMENT=100)");

		std::vector<std::byte> payload(kSeg * (kMaxSegs + 1));
		fill_pattern(payload);
		errno = 0;
		ssize_t n = ::send(pair.tx.get(), payload.data(), payload.size(), 0);
		std::println("  129 段 / {} 字节：send = {}，errno = {}（{}）", payload.size(), n, errno,
				n < 0 ? std::strerror(errno) : "无错误");
	}
}

// 用例 4：总长度上限。IPv4 下 UDP 载荷最大 65507 字节；65508 在进入 GSO 分段前
// 就被长度检查拒绝，返回 EMSGSIZE。
void case_total_limit()
{
	std::println("\n== 用例 4：总长度上限（IPv4 UDP 载荷最多 65507 字节）==");
	constexpr std::uint16_t kSeg = 1000;

	{
		UdpPair pair = make_loopback_pair();
		set_int_option(pair.tx.get(), kSolUdp, UDP_SEGMENT, kSeg, "setsockopt(UDP_SEGMENT=1000)");

		std::vector<std::byte> payload(65507);
		fill_pattern(payload);
		ssize_t n = ::send(pair.tx.get(), payload.data(), payload.size(), 0);
		if (n < 0)
			die("send(65507)");
		RecvResult r = recv_all(pair.rx.get(), payload.size());
		std::println("  65507 字节：send 成功，收到 {} 个包，最后一段 {} 字节",
				r.sizes.size(), r.sizes.empty() ? 0 : r.sizes.back());
	}

	{
		UdpPair pair = make_loopback_pair();
		set_int_option(pair.tx.get(), kSolUdp, UDP_SEGMENT, kSeg, "setsockopt(UDP_SEGMENT=1000)");

		std::vector<std::byte> payload(65508);
		fill_pattern(payload);
		errno = 0;
		ssize_t n = ::send(pair.tx.get(), payload.data(), payload.size(), 0);
		std::println("  65508 字节：send = {}，errno = {}（{}）", n, errno,
				n < 0 ? std::strerror(errno) : "无错误");
	}
}

// 用例 5：gso_size 受出口 MTU 限制。
// 内核检查 hlen + min(datalen, gso_size) > fragsize 就返回 EMSGSIZE。
// 对已 connect 的 UDP socket，可用 getsockopt(IP_MTU) 问出路径 MTU。
void case_mtu_limit()
{
	std::println("\n== 用例 5：gso_size 受出口 MTU 限制 ==");

	UdpPair pair = make_loopback_pair();

	int mtu = 0;
	socklen_t mtu_len = sizeof(mtu);
	if (::getsockopt(pair.tx.get(), IPPROTO_IP, IP_MTU, &mtu, &mtu_len) < 0)
		die("getsockopt(IP_MTU)");

	std::println("  到 127.0.0.1 的路径 MTU = {} 字节（IP+UDP 头 {} 字节）", mtu, kIpUdpHeaderLen);
	if (mtu <= static_cast<int>(kIpUdpHeaderLen)) {
		std::println("  MTU 太小，跳过");
		return;
	}
	std::size_t max_seg = static_cast<std::size_t>(mtu) - kIpUdpHeaderLen;
	std::println("  因此单段 gso_size 最大 = {}（再大就装不下一个 IP 包）", max_seg);

	// 只有两段总量不超过 UDP 上限时才做“边界 +1”的验证。
	if (max_seg * 2 > 65507) {
		std::println("  本机 MTU 很大（两段会超过 65507），跳过 +1 边界实测");
		return;
	}

	{
		// 用允许的最大段大小发两段，应当成功。
		UdpPair p = make_loopback_pair();
		set_int_option(p.tx.get(), kSolUdp, UDP_SEGMENT, static_cast<int>(max_seg),
				"setsockopt(UDP_SEGMENT=max)");
		std::vector<std::byte> payload(max_seg * 2);
		fill_pattern(payload);
		ssize_t n = ::send(p.tx.get(), payload.data(), payload.size(), 0);
		if (n < 0)
			die("send(max_seg)");
		RecvResult r = recv_all(p.rx.get(), payload.size());
		std::println("  gso_size = {}：send 成功，收到 {} 个 {} 字节的包", max_seg,
				r.sizes.size(), max_seg);
	}

	{
		// 段大小再大 1 字节：单段 IP 包超过 MTU，send 返回 EMSGSIZE。
		UdpPair p = make_loopback_pair();
		set_int_option(p.tx.get(), kSolUdp, UDP_SEGMENT, static_cast<int>(max_seg + 1),
				"setsockopt(UDP_SEGMENT=max+1)");
		std::vector<std::byte> payload(max_seg * 2);
		fill_pattern(payload);
		errno = 0;
		ssize_t n = ::send(p.tx.get(), payload.data(), payload.size(), 0);
		std::println("  gso_size = {}（+1）：send = {}，errno = {}（{}）", max_seg + 1, n, errno,
				n < 0 ? std::strerror(errno) : "无错误");
	}
}

int main()
{
	std::println("UDP GSO 演示：一次 send 交给内核，内核按 UDP_SEGMENT 切成多个 UDP 包");
	std::println("注意：接收端不需要任何特殊设置，用普通 recv 逐个收。");

	case_setsockopt();
	case_cmsg();
	case_segment_limit();
	case_total_limit();
	case_mtu_limit();

	std::println("\n全部用例结束。");
	return 0;
}
