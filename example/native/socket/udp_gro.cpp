/*
* udp_gro.cpp —— UDP GRO（Generic Receive Offload，UDP_GRO）
*
* 一、这个接口是什么
*   接收端调用 setsockopt(SOL_UDP, UDP_GRO, &on) 打开后，内核可以把同一对端、
*   同一目的地址、长度相同（最后一段可短）的多个 UDP 数据报合并成一个“大块”
*   交给用户态。用户态一次 recvmsg 就能拿到多个原始数据报的拼接结果，同时通过
*   控制消息（cmsg）UDP_GRO 得到 gso_size，按它就能把大块切回原来的各个数据报。
*
* 二、解决什么问题
*   高包速率场景下，每个 UDP 包一次 recv 的系统调用和协议栈开销很大。GRO 把
*   多个包在接收路径上合并，用户态按批处理，减少 recv 次数和每包开销。
*   注意 GRO 只影响“本地怎么收”，线路上仍然是普通 UDP 包，发送端不需要配合；
*   对端也不知道接收端开了 GRO。
*
* 三、形态
*   - 一次 recvmsg 返回一块连续内存，里面是 K 个原始 UDP 载荷首尾相接。
*   - K 个原始数据报 = 线上 K 个 UDP 包（K 个传统意义上的接收单元）。
*   - 用户态提交 1 次 recvmsg，完成 1 次；内核可能在一次调用里交付多个包。
*   - 内存由用户提供（recvmsg 的 iovec），内核把数据拷进来。
*   - cmsg UDP_GRO 的载荷是一个 32 位 int 的 gso_size；没有合并时不会带这个 cmsg。
*
* 四、需要的内核/硬件条件
*   - 本机头文件 /usr/include/linux/udp.h 定义 UDP_GRO=104。
*   - 接收 socket 主动 setsockopt 打开；发送端无需设置。
*   - 回环上能否合并取决于内核的 GRO 批处理：
*       * 发送端用 UDP GSO 发大块时，回环驱动会把带 gso_size 的 skb 直接交给
*         接收端，UDP_GRO 能稳定拿到合并结果（机制描述未核对，行为已实测）；
*       * 发送端逐个 send 小包时，只有这些包落在同一个 NAPI 轮询批次里才会被
*         合并，属于“尽力而为”，不同内核/负载下结果可能不同。
*     本程序会如实打印本次运行到底合并了没有，不假定一定合并。
*   - 本机实测：WSL2，内核 6.18.33.2，IPv4 回环两种方式都能复现合并。
*
* 五、常见坑
*   - recvmsg 的缓冲区要足够大：合并后的块可能接近 64KB（GRO 的合并上限未核对）。
*     缓冲区不够时内核会截断并置 MSG_TRUNC，数据就丢了（本程序检查 msg_flags）。
*   - cmsg 的 level 是 SOL_UDP（glibc 未定义，等于 IPPROTO_UDP=17），type 是 UDP_GRO。
*   - 发送侧 cmsg 的 UDP_SEGMENT 是 16 位，接收侧 cmsg 的 UDP_GRO 是 32 位 int，
*     两者不一样，别混用。
*   - 只有 gso_size > 0 时才需要切分；普通单个数据报没有 cmsg，整块就是 1 个包。
*   - 切分时最后一段允许短于 gso_size；不能假设总长一定是 gso_size 的整数倍。
*
* 六、编译与运行
*   g++ -std=c++26 -O2 -Wall -Wextra -o /tmp/native_udp_gro udp_gro.cpp
*   /tmp/native_udp_gro
*   预期输出：三个用例分别展示“GSO 发送 + GRO 接收”“普通逐包发送 + GRO 接收”
*   “GSO 发送但不打开 GRO”时，recvmsg 每次拿到多少字节、是否带 gso_size、
*   按 gso_size 切回几个原始数据报，以及内容校验结果。
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

// glibc 没有 SOL_UDP；它就是 IPPROTO_UDP（17），作为 setsockopt/cmsg 的 level。
constexpr int kSolUdp = IPPROTO_UDP;

// GRO 合并后的块可能很大；给 256KB 的接收缓冲区，远大于单个 UDP 数据报上限。
constexpr std::size_t kRecvBufferSize = 256 * 1024;

// ---------------------------------------------------------------------------
// RAII 文件描述符：析构 close，禁止拷贝，允许移动。
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

[[noreturn]] void die(std::string_view what)
{
	std::println(stderr, "错误：{}：{}", what, std::strerror(errno));
	std::exit(EXIT_FAILURE);
}

void set_int_option(int fd, int level, int name, int value, std::string_view what)
{
	if (::setsockopt(fd, level, name, &value, sizeof(value)) < 0)
		die(what);
}

struct UdpPair {
	UniqueFd tx;
	UniqueFd rx;
};

// 回环上的一对 UDP socket：rx 绑定临时端口，tx connect 到它；rx 设 1 秒接收超时。
UdpPair make_loopback_pair()
{
	UdpPair pair;

	pair.rx.reset(::socket(AF_INET, SOCK_DGRAM, 0));
	if (pair.rx.get() < 0)
		die("socket(rx)");

	sockaddr_in addr{};
	addr.sin_family = AF_INET;
	addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
	addr.sin_port = 0;
	if (::bind(pair.rx.get(), reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) < 0)
		die("bind(rx)");

	socklen_t addr_len = sizeof(addr);
	if (::getsockname(pair.rx.get(), reinterpret_cast<sockaddr *>(&addr), &addr_len) < 0)
		die("getsockname(rx)");

	timeval timeout{};
	timeout.tv_sec = 1;
	if (::setsockopt(pair.rx.get(), SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout)) < 0)
		die("setsockopt(SO_RCVTIMEO)");

	pair.tx.reset(::socket(AF_INET, SOCK_DGRAM, 0));
	if (pair.tx.get() < 0)
		die("socket(tx)");

	sockaddr_in local{};
	local.sin_family = AF_INET;
	local.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
	local.sin_port = 0;
	if (::bind(pair.tx.get(), reinterpret_cast<sockaddr *>(&local), sizeof(local)) < 0)
		die("bind(tx)");

	if (::connect(pair.tx.get(), reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) < 0)
		die("connect(tx)");

	return pair;
}

void fill_pattern(std::span<std::byte> buf)
{
	for (std::size_t i = 0; i < buf.size(); ++i)
		buf[i] = static_cast<std::byte>((i * 131u + 7u) & 0xffu);
}

// 从 recvmsg 的 msghdr 里取出 UDP_GRO 的 gso_size。
// 返回 -1 表示没有这个 cmsg（即这一块没有合并，或者只有一个数据报）。
int get_gso_size(msghdr &msg)
{
	for (cmsghdr *cmsg = CMSG_FIRSTHDR(&msg); cmsg != nullptr;
		cmsg = CMSG_NXTHDR(&msg, cmsg)) {
		if (cmsg->cmsg_level != kSolUdp || cmsg->cmsg_type != UDP_GRO)
			continue;
		// 接收侧 UDP_GRO 的载荷是 int（CMSG_LEN(sizeof(int))），与发送侧
		// UDP_SEGMENT 的 16 位不同。先检查长度再 memcpy，避免越界。
		if (cmsg->cmsg_len < CMSG_LEN(sizeof(int)))
			continue;
		int gso_size = 0;
		std::memcpy(&gso_size, CMSG_DATA(cmsg), sizeof(gso_size));
		return gso_size;
	}
	return -1;
}

// 按 gso_size 把一块合并数据切回各原始数据报的长度。
// gso_size <= 0：整块就是一个数据报。
// 否则前面都是 gso_size，最后一块是余数（可以为 0，表示正好整除）。
std::vector<std::size_t> split_segments(std::size_t len, int gso_size)
{
	std::vector<std::size_t> segments;
	if (gso_size <= 0) {
		segments.push_back(len);
		return segments;
	}

	const std::size_t seg = static_cast<std::size_t>(gso_size);
	std::size_t offset = 0;
	while (offset < len) {
		std::size_t n = std::min(seg, len - offset);
		segments.push_back(n);
		offset += n;
	}
	return segments;
}

// 一次 recvmsg 的结果。
struct Batch {
	std::size_t len = 0;	  // 本次 recvmsg 返回的总字节数
	int gso_size = -1;	  // cmsg 里的 gso_size，-1 表示没有
	bool truncated = false;	  // 缓冲区不够，数据被截断
	std::vector<std::size_t> segments; // 按 gso_size 切出的各原始数据报长度
};

// 连续 recvmsg，直到收满 expected_total 字节或超时。
// 同时把所有字节按顺序拼进 bytes，便于最后做内容校验。
std::vector<Batch> recv_batches(int fd, std::size_t expected_total, std::vector<std::byte> &bytes)
{
	std::vector<Batch> batches;
	std::array<std::byte, kRecvBufferSize> buffer{};
	std::array<char, CMSG_SPACE(sizeof(int))> control{};

	while (bytes.size() < expected_total) {
		iovec iov{buffer.data(), buffer.size()};
		msghdr msg{};
		msg.msg_iov = &iov;
		msg.msg_iovlen = 1;
		msg.msg_control = control.data();
		msg.msg_controllen = control.size();

		ssize_t n = ::recvmsg(fd, &msg, 0);
		if (n < 0) {
			if (errno == EAGAIN || errno == EWOULDBLOCK)
				break;
			die("recvmsg");
		}
		if (n == 0)
			break;

		Batch batch;
		batch.len = static_cast<std::size_t>(n);
		batch.gso_size = get_gso_size(msg);
		batch.truncated = (msg.msg_flags & MSG_TRUNC) != 0;
		batch.segments = split_segments(batch.len, batch.gso_size);

		// MSG_TRUNC 表示内核其实有更多数据但缓冲区放不下，n 可能仍是“完整长度”。
		// 这里只拷贝缓冲区实际容纳的部分，并在打印时提示。
		std::size_t copy_len = std::min(batch.len, buffer.size());
		bytes.insert(bytes.end(), buffer.begin(), buffer.begin() + copy_len);
		batches.push_back(std::move(batch));
	}
	return batches;
}

// 打印每个 recvmsg 的结果，并汇总“原始数据报个数”。
struct Summary {
	std::size_t recv_calls = 0;
	std::size_t datagrams = 0;
	bool merged = false;
	bool any_truncated = false;
};

Summary print_batches(const std::vector<Batch> &batches)
{
	Summary summary;
	summary.recv_calls = batches.size();

	for (std::size_t i = 0; i < batches.size(); ++i) {
		const Batch &b = batches[i];
		summary.datagrams += b.segments.size();
		summary.any_truncated = summary.any_truncated || b.truncated;
		// 一次拿到的块里包含多于一个原始数据报，才算真正发生了合并。
		if (b.segments.size() > 1 || (b.gso_size > 0 && b.len > static_cast<std::size_t>(b.gso_size)))
			summary.merged = true;

		std::print("  recvmsg[{}]：{} 字节", i, b.len);
		if (b.gso_size > 0)
			std::print("，cmsg gso_size={}，切回 {} 个包：", b.gso_size, b.segments.size());
		else
			std::print("，无 UDP_GRO cmsg，视为 1 个包：");

		constexpr std::size_t kShow = 8;
		for (std::size_t j = 0; j < std::min(b.segments.size(), kShow); ++j)
			std::print("{}{}", j ? " + " : "", b.segments[j]);
		if (b.segments.size() > kShow)
			std::print(" + ...（共 {} 个）", b.segments.size());
		std::println("");

		if (b.truncated)
			std::println("    警告：MSG_TRUNC，接收缓冲区不足，数据被截断");
	}
	return summary;
}

// 用例 A：发送端用 UDP GSO 发一大块（4500 字节 / 1000 一段），接收端开 UDP_GRO。
// 这是最容易观察到合并的组合：回环直接传递带 gso_size 的 skb。
void case_gso_sender()
{
	std::println("\n== 用例 A：GSO 发送（4500 字节，段 1000）+ GRO 接收 ==");
	constexpr std::uint16_t kSeg = 1000;
	constexpr std::size_t kTotal = 4500;

	UdpPair pair = make_loopback_pair();
	int on = 1;
	// 接收端打开 UDP_GRO。optval 是 int，非 0 即开启。
	set_int_option(pair.rx.get(), kSolUdp, UDP_GRO, on, "setsockopt(UDP_GRO)");

	// 发送端的 GSO 设置与 udp_gso.cpp 相同；GRO 与 GSO 互不依赖。
	set_int_option(pair.tx.get(), kSolUdp, UDP_SEGMENT, kSeg, "setsockopt(UDP_SEGMENT)");

	std::vector<std::byte> payload(kTotal);
	fill_pattern(payload);
	ssize_t sent = ::send(pair.tx.get(), payload.data(), payload.size(), 0);
	if (sent < 0)
		die("send");
	std::println("  发送端一次 send 了 {} 字节（线上应切成 5 个 UDP 包）", sent);

	std::vector<std::byte> received;
	std::vector<Batch> batches = recv_batches(pair.rx.get(), kTotal, received);
	Summary summary = print_batches(batches);

	std::println("  汇总：recvmsg 调用 {} 次，还原出 {} 个原始数据报；是否合并：{}", summary.recv_calls,
			summary.datagrams, summary.merged ? "是" : "否");
	std::println("  内容校验：{}", received == payload ? "逐字节一致" : "不一致");
	if (summary.datagrams != 5)
		std::println("  注意：期望 5 个原始数据报，实际切出 {} 个", summary.datagrams);
}

// 用例 B：发送端不用 GSO，普通地逐个 send 小包；接收端仍然开 UDP_GRO。
// 回环上这些包可能落在同一个 GRO 批次里而被合并，也可能不合并。
// 本程序不假设结果，只如实打印。
void case_plain_sender()
{
	std::println("\n== 用例 B：普通逐包发送（4 × 800 字节）+ GRO 接收 ==");
	constexpr std::size_t kEach = 800;
	constexpr int kCount = 4;

	UdpPair pair = make_loopback_pair();
	int on = 1;
	set_int_option(pair.rx.get(), kSolUdp, UDP_GRO, on, "setsockopt(UDP_GRO)");

	std::vector<std::byte> payload(kEach * kCount);
	fill_pattern(payload);

	// 关键：这里故意每次只 send 800 字节，不用 GSO。内核是否合并取决于
	// 这些包是否在同一轮 NAPI 接收里被 GRO 处理。
	for (int i = 0; i < kCount; ++i) {
		ssize_t n = ::send(pair.tx.get(), payload.data() + i * kEach, kEach, 0);
		if (n < 0)
			die("send");
	}
	std::println("  发送端调用了 {} 次 send，每次 {} 字节", kCount, kEach);

	std::vector<std::byte> received;
	std::vector<Batch> batches = recv_batches(pair.rx.get(), payload.size(), received);
	Summary summary = print_batches(batches);

	std::println("  汇总：recvmsg 调用 {} 次，还原出 {} 个原始数据报；是否合并：{}", summary.recv_calls,
			summary.datagrams, summary.merged ? "是" : "否");
	if (!summary.merged)
		std::println("  说明：本次运行没有发生合并。这是正常的——逐包发送的 GRO 合并是尽力而为，"
				"取决于内核批处理时机。");
	std::println("  内容校验：{}", received == payload ? "逐字节一致" : "不一致");
}

// 用例 C：对照组。发送端同样用 GSO，但接收端不打开 UDP_GRO。
// 此时每次 recvmsg 只拿到一个原始数据报，也没有 UDP_GRO cmsg。
void case_gro_off()
{
	std::println("\n== 用例 C：GSO 发送（4000 字节，段 1000）+ 接收端不开 GRO ==");
	constexpr std::uint16_t kSeg = 1000;
	constexpr std::size_t kTotal = 4000;

	UdpPair pair = make_loopback_pair();
	set_int_option(pair.tx.get(), kSolUdp, UDP_SEGMENT, kSeg, "setsockopt(UDP_SEGMENT)");

	std::vector<std::byte> payload(kTotal);
	fill_pattern(payload);
	if (::send(pair.tx.get(), payload.data(), payload.size(), 0) < 0)
		die("send");

	std::vector<std::byte> received;
	std::vector<Batch> batches = recv_batches(pair.rx.get(), kTotal, received);
	Summary summary = print_batches(batches);

	std::println("  汇总：recvmsg 调用 {} 次，每次 1 个包；是否合并：{}", summary.recv_calls,
			summary.merged ? "是" : "否");
	std::println("  内容校验：{}", received == payload ? "逐字节一致" : "不一致");
}

int main()
{
	std::println("UDP GRO 演示：接收端一次 recvmsg 拿到多个合并的 UDP 包，再按 gso_size 切回");
	std::println("注意：GRO 只影响接收端；发送端不需要为接收端的 GRO 做任何设置。");

	case_gso_sender();
	case_plain_sender();
	case_gro_off();

	std::println("\n全部用例结束。");
	return 0;
}
