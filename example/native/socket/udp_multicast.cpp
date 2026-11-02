/*
* udp_multicast.cpp —— UDP 组播（IP_ADD_MEMBERSHIP / IP_MULTICAST_IF / IP_MULTICAST_LOOP）
*
* 一、这个接口是什么
*   组播（multicast）让一个发送端只发一次数据，由网络把这份数据复制给加入同一
*   组播组（group）的所有接收端。UDP 组播用 IPv4 的 D 类地址（224.0.0.0/4），
*   接收端通过 setsockopt(IP_ADD_MEMBERSHIP) 告诉内核“我要收这个组”。
*
* 二、解决什么问题
*   一对多分发时，如果用单播，发送端必须知道每个接收者地址并逐个发；组播把
*   “发给谁”交给组地址，发送端只发一次，接收端自行加入/退出，适合行情分发、
*   服务发现、局域网内的一对多通知等。
*
* 三、形态
*   - 发送端：一次 sendto 提交 1 个 UDP 数据报，目标是组地址:端口；网络负责复制。
*   - 接收端：每个接收 socket 需要 bind 本地端口 + 用 IP_ADD_MEMBERSHIP 加入组；
*     每个成员各自完成一次 recvfrom。
*   - 内存由用户提供；UDP 无连接、无确认，发送成功只代表交给了协议栈。
*   - 本例 1 个发送端、2 个接收 socket，一次 sendto，两个接收端都应收到同一份数据。
*
* 四、需要的内核/接口条件（本机 WSL2 的实际情况很重要）
*   - 承载组播的接口必须有 IFF_MULTICAST 标志。`ip link show lo` 里应出现 MULTICAST。
*   - 本机 WSL2（内核 6.18.33.2）的 lo 只有 <LOOPBACK,UP,LOWER_UP>，没有 MULTICAST。
*     在 lo 上 IP_ADD_MEMBERSHIP 虽然返回 0，但内核不会把组播送到 lo，收不到包。
*     因此本程序先检测 lo 的标志：
*       * lo 有 MULTICAST：按题目要求用 127.0.0.1/lo 做 IP_MULTICAST_IF 和加入接口；
*       * lo 没有 MULTICAST：如实打印原因，退回到系统默认组播路由所在的接口
*         （本机是 eth0）完成演示。组播包 TTL=1，只在本链路内，用于演示。
*     要在 lo 上真正跑通，需要内核/驱动支持给 lo 打开 MULTICAST，例如：
*       ip link set lo multicast on
*     但 WSL2 的 lo 驱动通常仍不会真正回环组播（未核对），这时应改用带 MULTICAST
*     的 veth/dummy/以太网接口。
*   - 需要有到 239.0.0.0/8 的路由；IP_MULTICAST_LOOP=1 才会把本机发出的组播
*     再送给本机的组成员。
*
* 五、常见坑
*   - 接收端 bind 的是本地地址（本例 INADDR_ANY）和端口，不是组地址；组地址只用于
*     IP_ADD_MEMBERSHIP 和发送时的目的地址。
*   - struct ip_mreq 的 imr_interface 要填“本地接口地址”（如 127.0.0.1 或 eth0 的
*     地址），不是组地址，也不是接口名。
*   - IP_MULTICAST_IF 的 optval 是 struct in_addr（本地接口地址），不是 ip_mreq。
*   - 多个接收 socket 绑定同一端口时必须都设 SO_REUSEADDR，否则第二个 bind 报
*     EADDRINUSE。
*   - 发送 socket 不需要加入组；只有接收方才需要 IP_ADD_MEMBERSHIP。
*   - UDP 组播不保证可靠、不保证顺序，也不做拥塞控制。
*
* 六、编译与运行
*   g++ -std=c++26 -O2 -Wall -Wextra -o /tmp/native_udp_multicast udp_multicast.cpp
*   /tmp/native_udp_multicast
*   预期输出：先打印 lo 是否有 MULTICAST；然后两个接收 socket 加入组，发送端
*   一次 sendto，两个接收端各打印收到的字节数和内容；最后给出“两个都收到”的结论。
*   在 lo 没有 MULTICAST 的 WSL 上会看到退回默认接口的说明。
*/

#include <arpa/inet.h>
#include <net/if.h>
#include <netinet/in.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <array>
#include <cerrno>
#include <cstddef>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <print>
#include <string_view>
#include <utility>

// 组播组：239.0.0.0/8 是管理范围（administratively scoped）的组播地址，
// 适合在本地/组织内做演示，不会跑到公网。端口在运行时选空闲端口。
constexpr const char *kGroupText = "239.255.42.99";

// TTL=1：组播包只在本链路内传播，不跨路由器。演示足够，也避免影响外部网络。
constexpr int kMulticastTtl = 1;

// ---------------------------------------------------------------------------
// RAII 文件描述符。
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

// 查询接口（如 "lo"）是否有 IFF_MULTICAST 标志。
// SIOCGIFFLAGS 填充 struct ifreq 的 ifr_flags；失败则返回 false 并打印原因。
bool interface_has_multicast(std::string_view name)
{
	UniqueFd ctl(::socket(AF_INET, SOCK_DGRAM, 0));
	if (ctl.get() < 0)
		die("socket(SIOCGIFFLAGS)");

	ifreq ifr{};
	// ifr_name 最多 IFNAMSIZ-1 个字符，末尾要留 '\0'。
	std::strncpy(ifr.ifr_name, name.data(), IFNAMSIZ - 1);
	if (::ioctl(ctl.get(), SIOCGIFFLAGS, &ifr) < 0) {
		std::println("  查询接口 {} 的标志失败：{}", name, std::strerror(errno));
		return false;
	}
	return (ifr.ifr_flags & IFF_MULTICAST) != 0;
}

// 找出内核会把“发往该组播组”的包从哪个本地地址送出去。
// 做法：建一个临时 UDP socket，connect 到组地址（UDP connect 不发包，只选路由），
// 再用 getsockname 读回内核选中的本地地址。失败返回 false。
bool discover_multicast_source(const in_addr &group, in_addr &source)
{
	UniqueFd probe(::socket(AF_INET, SOCK_DGRAM, 0));
	if (probe.get() < 0)
		return false;

	sockaddr_in dst{};
	dst.sin_family = AF_INET;
	dst.sin_addr = group;
	dst.sin_port = htons(9); // 端口只用于选路，不会真的发送
	if (::connect(probe.get(), reinterpret_cast<sockaddr *>(&dst), sizeof(dst)) < 0)
		return false;

	sockaddr_in local{};
	socklen_t len = sizeof(local);
	if (::getsockname(probe.get(), reinterpret_cast<sockaddr *>(&local), &len) < 0)
		return false;

	source = local.sin_addr;
	return true;
}

std::string_view ip_text(const in_addr &addr, std::array<char, INET_ADDRSTRLEN> &storage)
{
	if (::inet_ntop(AF_INET, &addr, storage.data(), storage.size()) == nullptr)
		return "?";
	return std::string_view(storage.data());
}

int main()
{
	std::println("UDP 组播演示：1 个发送端一次 sendto，2 个接收 socket 加入同组");

	// 组地址：inet_pton 把点分十进制转成网络字节序的 32 位地址。
	in_addr group{};
	if (::inet_pton(AF_INET, kGroupText, &group) != 1)
		die("inet_pton(组地址)");

	std::array<char, INET_ADDRSTRLEN> group_text{};
	std::println("组播组：{}", ip_text(group, group_text));

	// ---- 1. 选接口 -------------------------------------------------------
	// 优先按题目要求用 lo/127.0.0.1；只有 lo 带 IFF_MULTICAST 才有意义。
	in_addr iface{};
	iface.s_addr = htonl(INADDR_LOOPBACK);
	bool using_lo = interface_has_multicast("lo");

	if (using_lo) {
		std::println("  lo 具备 IFF_MULTICAST：使用 127.0.0.1/lo 作为组播接口");
	} else {
		std::println("  lo 没有 IFF_MULTICAST：内核不会在 lo 上回环组播，加入组也收不到。");
		std::println("  原因：WSL2 的 lo 只有 LOOPBACK/UP/LOWER_UP（可用 `ip link show lo` 确认）。");
		std::println("  本程序改用系统默认组播路由所在接口完成演示；TTL=1，包不会离开本链路。");

		if (!discover_multicast_source(group, iface)) {
			// 连默认路由都没有：退回 INADDR_ANY，由内核在加入组时自行选择。
			iface.s_addr = htonl(INADDR_ANY);
			std::println("  无法确定默认组播接口，改为 INADDR_ANY（由内核选择）。");
		} else {
			std::array<char, INET_ADDRSTRLEN> text{};
			std::println("  实际使用接口地址：{}", ip_text(iface, text));
		}
	}

	// ---- 2. 两个接收 socket ---------------------------------------------
	// 两个 socket 绑定同一端口，才能都收到发往该端口的组播；SO_REUSEADDR 是必须的。
	UniqueFd rx1(::socket(AF_INET, SOCK_DGRAM, 0));
	if (rx1.get() < 0)
		die("socket(rx1)");
	set_int_option(rx1.get(), SOL_SOCKET, SO_REUSEADDR, 1, "setsockopt(SO_REUSEADDR rx1)");

	// bind 到 INADDR_ANY:0，让内核先选一个空闲端口；组播接收端不能 bind 组地址。
	sockaddr_in bind_addr{};
	bind_addr.sin_family = AF_INET;
	bind_addr.sin_addr.s_addr = htonl(INADDR_ANY);
	bind_addr.sin_port = 0;
	if (::bind(rx1.get(), reinterpret_cast<sockaddr *>(&bind_addr), sizeof(bind_addr)) < 0)
		die("bind(rx1)");

	socklen_t bind_len = sizeof(bind_addr);
	if (::getsockname(rx1.get(), reinterpret_cast<sockaddr *>(&bind_addr), &bind_len) < 0)
		die("getsockname(rx1)");
	std::println("  接收端口：{}", ntohs(bind_addr.sin_port));

	UniqueFd rx2(::socket(AF_INET, SOCK_DGRAM, 0));
	if (rx2.get() < 0)
		die("socket(rx2)");
	set_int_option(rx2.get(), SOL_SOCKET, SO_REUSEADDR, 1, "setsockopt(SO_REUSEADDR rx2)");
	if (::bind(rx2.get(), reinterpret_cast<sockaddr *>(&bind_addr), sizeof(bind_addr)) < 0)
		die("bind(rx2)");

	// 接收超时，避免收不到时无限等待。
	timeval timeout{};
	timeout.tv_sec = 1;
	for (int fd : {rx1.get(), rx2.get()}) {
		if (::setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout)) < 0)
			die("setsockopt(SO_RCVTIMEO)");
	}

	// 加入组播组。ip_mreq 两个字段都是网络字节序：
	//   imr_multiaddr = 组地址
	//   imr_interface = 本地接口地址（不是组地址）；INADDR_ANY 表示让内核选。
	ip_mreq membership{};
	membership.imr_multiaddr = group;
	membership.imr_interface = iface;
	for (int fd : {rx1.get(), rx2.get()}) {
		if (::setsockopt(fd, IPPROTO_IP, IP_ADD_MEMBERSHIP, &membership, sizeof(membership)) < 0)
			die("setsockopt(IP_ADD_MEMBERSHIP)");
	}
	std::println("  两个接收 socket 都已加入组 {}", kGroupText);

	// ---- 3. 发送端 -------------------------------------------------------
	UniqueFd tx(::socket(AF_INET, SOCK_DGRAM, 0));
	if (tx.get() < 0)
		die("socket(tx)");

	// IP_MULTICAST_IF 指定从哪个本地地址发出组播；optval 是 struct in_addr。
	// INADDR_ANY 表示不指定，交给路由表。发送 socket 不需要加入组。
	if (iface.s_addr != htonl(INADDR_ANY)) {
		if (::setsockopt(tx.get(), IPPROTO_IP, IP_MULTICAST_IF, &iface, sizeof(iface)) < 0)
			die("setsockopt(IP_MULTICAST_IF)");
	}

	// IP_MULTICAST_LOOP=1：把本机发出的组播复制一份给本机的组成员。
	// 没有它，同一台机器上的接收端收不到自己发的组播。
	set_int_option(tx.get(), IPPROTO_IP, IP_MULTICAST_LOOP, 1, "setsockopt(IP_MULTICAST_LOOP)");
	set_int_option(tx.get(), IPPROTO_IP, IP_MULTICAST_TTL, kMulticastTtl, "setsockopt(IP_MULTICAST_TTL)");

	// 目的地址 = 组地址 + 接收端口。
	sockaddr_in dst{};
	dst.sin_family = AF_INET;
	dst.sin_addr = group;
	dst.sin_port = bind_addr.sin_port;

	std::array<std::byte, 64> payload{};
	constexpr std::string_view kMessage = "hello multicast, one sendto";
	std::memcpy(payload.data(), kMessage.data(), kMessage.size());

	// 只调用一次 sendto：两个接收端都应该收到。
	ssize_t sent = ::sendto(tx.get(), payload.data(), kMessage.size(), 0,
				reinterpret_cast<sockaddr *>(&dst), sizeof(dst));
	if (sent < 0)
		die("sendto");
	std::println("  发送端一次 sendto 了 {} 字节到 {}:{}", sent, kGroupText,
			ntohs(dst.sin_port));

	// ---- 4. 两个接收端分别收 --------------------------------------------
	int received_count = 0;
	bool content_ok = true;
	for (int i = 0; i < 2; ++i) {
		int fd = i == 0 ? rx1.get() : rx2.get();
		std::array<std::byte, 256> buf{};
		sockaddr_in from{};
		socklen_t from_len = sizeof(from);

		ssize_t n = ::recvfrom(fd, buf.data(), buf.size(), 0,
					reinterpret_cast<sockaddr *>(&from), &from_len);
		if (n < 0) {
			if (errno == EAGAIN || errno == EWOULDBLOCK) {
				std::println("  接收端 {}：1 秒内没收到组播包", i + 1);
				continue;
			}
			die("recvfrom");
		}

		++received_count;
		std::array<char, INET_ADDRSTRLEN> from_text{};
		bool same = static_cast<std::size_t>(n) == kMessage.size() &&
				std::memcmp(buf.data(), kMessage.data(), kMessage.size()) == 0;
		content_ok = content_ok && same;
		std::println("  接收端 {}：收到 {} 字节，来自 {}:{}{}", i + 1, n,
				ip_text(from.sin_addr, from_text), ntohs(from.sin_port),
				same ? "，内容一致" : "，内容不一致");
	}

	// ---- 5. 结论 ---------------------------------------------------------
	std::println("");
	if (received_count == 2 && content_ok) {
		std::println("结论：一次 sendto，两个接收 socket 都收到了同一份数据。");
	} else {
		std::println("结论：只有 {}/2 个接收端收到数据。", received_count);
		std::println("排查：1) 接口是否有 IFF_MULTICAST；2) 是否存在到 239.0.0.0/8 的路由；");
		std::println("      3) IP_MULTICAST_LOOP 是否为 1；4) 防火墙/组播 snooping 是否丢弃了包。");
		std::println("      WSL2 的 lo 没有 MULTICAST 时，组播无法只在本机回环内完成。");
		return EXIT_FAILURE;
	}
	return 0;
}
