/*
	* af_xdp.cpp —— AF_XDP socket（Linux 原生 UAPI，不依赖 libbpf / libxdp）
	*
	* 一、它是什么
	* AF_XDP 是 Linux 4.18 起提供的套接字地址族（AF_XDP = 44，SOL_XDP = 283），
	* 把一个网卡的某个接收队列“旁路”给用户态：驱动（或内核的通用 XDP 路径）
	* 直接通过共享内存环把收到的帧递给用户进程，用户进程也通过共享内存环交包给驱动发送。
	* 它由三块组成：
	*   1. UMEM：一块由“用户”分配的连续内存，被切成固定大小的 chunk（frame），
	*      所有收发数据都放在这里面，内核不会替你再分配包缓冲区；
	*   2. 四张单生产者/单消费者环（fill / completion / rx / tx），由内核分配、
	*      用户 mmap 到自己的地址空间，双方通过 read/write 索引交换描述符；
	*   3. XDP 程序 + XSKMAP：只有网卡入口的 XDP 程序对包执行 XDP_REDIRECT 并把它
	*      redirect 到本 socket 的 XSKMAP 表项，包才会进入 rx 环（见下面的说明）。
	*
	* 二、解决什么问题、什么时候用
	* 普通 socket 每个包都要经过 skb 分配、协议栈、系统调用；AF_XDP 把这些都绕开，
	* 用户态批量地看/填共享环即可，单核可以做到每秒千万级包的收发，同时网卡仍由
	* 内核管理（路由、中断、ethtool 都还在），不像 DPDK 那样独占设备。
	* 适合：用户态包处理/转发、负载均衡、DNS 之类的高 PPS 服务、抓包加速。
	* 不适合：吞吐大但包数少的大块传输（这时普通 socket / io_uring 更省事）。
	*
	* 三、它的“形态”（重点，和 read/write 式接口完全不同）
	*   - 数据面没有 read/write/recvfrom/sendto 传数据：数据全部在 UMEM 里，
	*     双方只交换 16 字节的描述符（xdp_desc：addr 是相对 UMEM 起点的偏移、
	*     len 是包长、options 是标志）。一次“提交”是往 tx 环里放 N 个描述符，
	*     对应线上 N 个以太网帧；内核发完后往 completion 环放 N 个 addr 归还缓冲区。
	*   - 一次 sendto() 不携带数据，只是“踢”内核：告诉它“环里有新东西了，来看一眼”。
	*   - 一个 UMEM 的 chunk 可以是 2048/4096 字节；默认一个包占一个 chunk，
	*     巨型帧要开 XDP_USE_SG 才允许多 chunk 拼一个包（本示例不用）。
	*   - 内存归属：UMEM 由用户提供并负责生命周期；四张环的内核部分由内核分配，
	*     用户只 mmap 映射。描述符里的 addr 永远是“UMEM 内的偏移”，不是指针。
	*
	* 四、四张环的生产者/消费者（本例代码严格按这个方向读写）
	*   环名          生产者        消费者        放的东西
	*   fill   (fr)   用户          内核          空闲 frame 的 UMEM 偏移（__u64）
	*   rx            内核          用户          收到的包（struct xdp_desc）
	*   tx            用户          内核          要发的包（struct xdp_desc）
	*   completion(cr)内核          用户          已发完的 frame 偏移（__u64）
	*   用户侧对 fill/tx 只写 producer，对 rx/completion 只写 consumer；另外各自
	*   缓存一份对方的索引（cached_*），只有跨过缓存线时才真正读对方的内存。
	*   环大小是 2 的幂，索引自由回绕，用“无符号减法”算可用槽位。
	*
	* 五、前置条件
	*   - 需要 root 或 CAP_NET_RAW（创建 AF_XDP socket 本身就要）。没有权限时
	*     socket() 返回 EPERM，本程序会如实打印出来。
	*   - 网卡驱动要支持 XDP（至少 copy 模式）。lo 这种没有 ndo_bpf 的设备
	*     bind 可能返回 EOPNOTSUPP，这是设备限制而不是代码问题，注释里已说明。
	*   - 真正“收”到包还要求接口上挂了 XDP 程序并 redirect 到本 socket 的 XSKMAP；
	*     本示例只做到 bind（以及 TX 方向发一个包），重定向部分只给说明，见
	*     setup 末尾的注释块。
	*
	* 六、编译与运行
	*   g++ -std=c++26 -O2 -Wall -Wextra -o /tmp/native_af_xdp af_xdp.cpp
	*   sudo /tmp/native_af_xdp [网卡名]     # 默认 lo
	*   预期输出：逐条打印 socket/UMEM/四个环/bind 的结果；root 下在 TX 环提交
	*   一个 UDP 包（Ethernet + IPv4 + UDP，目标是 127.0.0.1:9999），等 completion
	*   环把它还回来，最后打印 XDP_STATISTICS。非 root 时在第一步打印 EPERM 退出。
	*
	*   实测（WSL2，内核 6.18，lo）：以 root 运行时 bind/提交/TX completion 全部正常；
	*   但这个注入帧并没有被本机 UDP 栈接收（另开 UDP socket 监听 9999 收不到）。
	*   帧本身用 AF_PACKET 抓包核对过，格式与校验和正确，丢在哪一段未核对。
	*   本示例要演示的是 TX 环与 completion 环，不依赖数据真的被应用层收到。
	*
	* 七、参考
	*   /usr/include/linux/if_xdp.h（本机的 UAPI 头，字段/常量以此为准）、
	*   内核 Documentation/networking/af_xdp.rst。
	*/

#include <arpa/inet.h>
#include <linux/if_ether.h>
#include <linux/if_xdp.h>
#include <linux/ip.h>
#include <linux/udp.h>
#include <net/if.h>
#include <sys/mman.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <format>
#include <optional>
#include <print>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <thread>
#include <utility>

namespace {

/* UMEM 里每个 frame 的大小。必须是 2 的幂。
	* 2048 是 copy 模式的常用值（普通以太网帧最大 1518，留有余量）。
	* 注意：内核还要求 chunk_size 不小于某个下限（我记得是 2048）且大于
	* headroom + XDP_PACKET_HEADROOM(256)，此处未在本机核对内核源码。 */
constexpr std::uint32_t kFrameSize = 2048;
/* UMEM 中 frame 总数，也是 fill 环的槽位数。必须 <= 环大小。 */
constexpr std::uint32_t kFrameCount = 128;
/* 四个环的槽位数，必须是 2 的幂。内核一般要求 64..4096，
	* 具体上下限未在本机核对（本机没有内核源码）。 */
constexpr std::uint32_t kRingEntries = 128;
/* 前若干个 frame 专门留给 TX，其余投入 fill 环供 RX 使用，避免同一个
	* frame 同时被内核（RX）和用户（TX）写。 */
constexpr std::uint32_t kTxFrames = 8;

static_assert((kFrameCount & (kFrameCount - 1)) == 0);
static_assert((kRingEntries & (kRingEntries - 1)) == 0);
static_assert(kFrameCount <= kRingEntries);
static_assert(kTxFrames < kFrameCount);

/* 演示包的以太网类型/端口等常量。 */
constexpr std::uint16_t kDemoDstPort = 9999;
constexpr std::string_view kDemoText = "hello from AF_XDP TX ring";
constexpr std::size_t kDemoFrameLen =
	sizeof(ethhdr) + sizeof(iphdr) + sizeof(udphdr) + kDemoText.size();

static_assert(sizeof(ethhdr) == 14, "以太网头应恰为 14 字节");
static_assert(sizeof(iphdr) == 20, "无选项 IPv4 头应恰为 20 字节");
static_assert(sizeof(udphdr) == 8, "UDP 头应恰为 8 字节");

[[noreturn]] void throw_errno(std::string_view what, int err)
{
	throw std::runtime_error(std::format("{}: {}", what, std::strerror(err)));
}

/* ------------------------------------------------------------------ */
/* RAII：文件描述符。构造即接管，析构 close；禁止拷贝，允许移动。      */
/* ------------------------------------------------------------------ */
class fd_handle {
public:
	explicit fd_handle(int fd = -1) noexcept : fd_(fd) {}
	~fd_handle() { reset(); }

	fd_handle(const fd_handle &) = delete;
	fd_handle &operator=(const fd_handle &) = delete;

	fd_handle(fd_handle &&other) noexcept : fd_(std::exchange(other.fd_, -1)) {}

	fd_handle &operator=(fd_handle &&other) noexcept
	{
		if (this != &other)
			reset(std::exchange(other.fd_, -1));
		return *this;
	}

	int get() const noexcept { return fd_; }

	/* 交还所有权，调用者负责 close。 */
	int release() noexcept { return std::exchange(fd_, -1); }

	void reset(int fd = -1) noexcept
	{
		if (fd_ >= 0)
			::close(fd_);
		fd_ = fd;
	}

private:
	int fd_;
};

/* ------------------------------------------------------------------ */
/* RAII：mmap 出来的内存。记录长度，析构时 munmap。                     */
/* ------------------------------------------------------------------ */
class mapping {
public:
	mapping() = default;
	mapping(void *base, std::size_t len) noexcept : base_(base), len_(len) {}
	~mapping() { reset(); }

	mapping(const mapping &) = delete;
	mapping &operator=(const mapping &) = delete;

	mapping(mapping &&other) noexcept
		: base_(std::exchange(other.base_, nullptr)), len_(std::exchange(other.len_, 0))
	{
	}

	mapping &operator=(mapping &&other) noexcept
	{
		if (this != &other) {
			reset();
			base_ = std::exchange(other.base_, nullptr);
			len_ = std::exchange(other.len_, 0);
		}
		return *this;
	}

	void reset() noexcept
	{
		if (base_ != nullptr)
			::munmap(base_, len_);
		base_ = nullptr;
		len_ = 0;
	}

	std::byte *data() const noexcept { return static_cast<std::byte *>(base_); }
	std::size_t size() const noexcept { return len_; }

	/* 从基址偏移 off 处取一个 T*。环的 producer/consumer/desc 都是
	* getsockopt(XDP_MMAP_OFFSETS) 返回的“页内偏移”，用这个函数定位。 */
	template <class T>
	T *at(std::uint64_t off) const noexcept
	{
		return reinterpret_cast<T *>(data() + static_cast<std::size_t>(off));
	}

private:
	void *base_ = nullptr;
	std::size_t len_ = 0;
};

/* ------------------------------------------------------------------ */
/* 一张环的四个关键位置。                                              */
/* ------------------------------------------------------------------ */
struct ring {
	/* producer/consumer 是内核与用户共享的两个 u32，只增不减、自由回绕；
	* 判断“空/满”永远用无符号减法，不要直接比较大小。 */
	std::uint32_t *producer = nullptr;
	std::uint32_t *consumer = nullptr;
	/* flags：内核往这里写 XDP_RING_NEED_WAKEUP，表示驱动已经睡了，
	* 用户提交后必须显式 wakeup（sendto）。仅 bind 时带
	* XDP_USE_NEED_WAKEUP 才有意义。 */
	std::uint32_t *flags = nullptr;
	/* 每个槽位的描述符。fill/completion 环的槽位是 __u64（frame 偏移），
	* rx/tx 环的槽位是 struct xdp_desc。 */
	std::uint64_t *addrs = nullptr;
	xdp_desc *descs = nullptr;
	/* 槽位数 - 1，用于 idx & mask 的回绕。 */
	std::uint32_t mask = 0;
	/* 用户侧缓存的对方索引：批量操作期间不必反复读共享缓存线。 */
	std::uint32_t cached_prod = 0;
	std::uint32_t cached_cons = 0;
};

/* 共享内存里的索引由内核在 C++ 内存模型之外读写。用 atomic_ref 做
	* acquire/release 访问，生成的就是 x86 上的普通 load/store（x86 本身是
	* TSO，release store 不需要额外指令），但语义上把顺序讲清楚了。
	* 严格说，对一个非 _Atomic 对象做 atomic_ref 属于实现相关的用法，
	* 这里是 AF_XDP 用户的通行做法（libbpf 也一样用 __atomic builtin）。 */
std::uint32_t load_acquire(const std::uint32_t *p) noexcept
{
	return std::atomic_ref<std::uint32_t>(*const_cast<std::uint32_t *>(p))
		.load(std::memory_order_acquire);
}

std::uint32_t load_relaxed(const std::uint32_t *p) noexcept
{
	return std::atomic_ref<std::uint32_t>(*const_cast<std::uint32_t *>(p))
		.load(std::memory_order_relaxed);
}

void store_release(std::uint32_t *p, std::uint32_t v) noexcept
{
	std::atomic_ref<std::uint32_t>(*p).store(v, std::memory_order_release);
}

/* 生产方向（fill/tx）：预留 n 个槽位，返回起始下标。
	* 可用槽位 = 环容量 - (本地 producer - 共享 consumer)。 */
std::optional<std::uint32_t> reserve_prod(ring &r, std::uint32_t n)
{
	const std::uint32_t used = r.cached_prod - load_acquire(r.consumer);
	const std::uint32_t free_slots = (r.mask + 1) - used;
	if (free_slots < n)
		return std::nullopt;
	const std::uint32_t idx = r.cached_prod & r.mask;
	r.cached_prod += n;
	return idx;
}

/* 生产方向提交：把本地 producer 发布给内核。release 保证描述符先于索引可见。 */
void commit_prod(ring &r) noexcept
{
	store_release(r.producer, r.cached_prod);
}

struct cons_batch {
	std::uint32_t start;
	std::uint32_t count;
};

/* 消费方向（rx/completion）：一次拿走最多 max 个已完成项。
	* 返回的 start 是本次批次的起始槽位，count 是数量。 */
std::optional<cons_batch> peek_cons(ring &r, std::uint32_t max)
{
	const std::uint32_t avail = load_acquire(r.producer) - r.cached_cons;
	const std::uint32_t n = std::min(avail, max);
	if (n == 0)
		return std::nullopt;
	const std::uint32_t start = r.cached_cons & r.mask;
	r.cached_cons += n;
	return cons_batch{start, n};
}

/* 消费方向归还：告诉内核这些槽位可以复用。 */
void release_cons(ring &r) noexcept
{
	store_release(r.consumer, r.cached_cons);
}

bool needs_wakeup(const ring &r) noexcept
{
	return (load_relaxed(r.flags) & XDP_RING_NEED_WAKEUP) != 0;
}

/* “踢”内核。参数全为 0/NULL：内核只关心有人调用了 sendto，
	* 并不会把这段参数当数据发出去。环不忙时调用也是安全的。 */
void kick(int fd)
{
	// libbpf 的 af_xdp 文档里的标准写法；man 2 sendto 中 length=0 不发送数据。
	if (::sendto(fd, nullptr, 0, MSG_DONTWAIT, nullptr, 0) < 0) {
		const int e = errno;
		std::println(stderr, "警告：sendto() 唤醒内核失败：{}", std::strerror(e));
	}
}

/* 标准的 IPv4 头部校验和：16 位反码和再取反。
	* 只算头部（20 字节，偶数长度）；UDP 校验和此处填 0，IPv4 允许。 */
std::uint16_t ip_checksum(const void *data, std::size_t len)
{
	const auto *p = static_cast<const std::uint16_t *>(data);
	std::uint32_t sum = 0;
	while (len > 1) {
		sum += *p++;
		len -= 2;
	}
	if (len != 0)
		sum += *reinterpret_cast<const std::uint8_t *>(p);
	while ((sum >> 16) != 0)
		sum = (sum & 0xffff) + (sum >> 16);
	return static_cast<std::uint16_t>(~sum);
}

/* 构造一个合法的 Ethernet + IPv4 + UDP 帧。
	* 目的地址是全零 MAC（lo 的 MAC）：AF_XDP 的 TX 走 dev_direct_xmit，
	* 帧头必须完整。发往 127.0.0.1:9999。帧格式和校验和都正确，但在 WSL2 的
	* lo 上实测这个注入帧没有进入本机 UDP 栈（详见文件头“实测”一段），
	* 所以本示例只用它来观察 TX 环 -> completion 环，不断言应用层能收到。 */
std::array<std::byte, kDemoFrameLen> build_demo_frame()
{
	ethhdr eth{};
	iphdr ip{};
	udphdr udp{};

	// 以太网头：lo 的地址是 00:00:00:00:00:00；0x0800 表示后面是 IPv4。
	std::memset(eth.h_dest, 0, ETH_ALEN);
	std::memset(eth.h_source, 0, ETH_ALEN);
	eth.h_proto = htons(ETH_P_IP);

	// IPv4 头：20 字节，无选项；TTL 64；UDP 协议号 17。
	ip.version = 4;
	ip.ihl = 5;
	ip.tos = 0;
	ip.tot_len = htons(static_cast<std::uint16_t>(sizeof(iphdr) +
		sizeof(udphdr) + kDemoText.size()));
	ip.id = htons(0x1234);
	ip.frag_off = 0;
	ip.ttl = 64;
	ip.protocol = IPPROTO_UDP;
	ip.saddr = htonl(INADDR_LOOPBACK);
	ip.daddr = htonl(INADDR_LOOPBACK);
	ip.check = 0;
	ip.check = ip_checksum(&ip, sizeof ip);

	// UDP 头：源端口随便取；IPv4 下校验和填 0 表示“未计算”，是合法的。
	udp.source = htons(40000);
	udp.dest = htons(kDemoDstPort);
	udp.len = htons(static_cast<std::uint16_t>(sizeof(udphdr) + kDemoText.size()));
	udp.check = 0;

	std::array<std::byte, kDemoFrameLen> buf{};
	std::memcpy(buf.data(), &eth, sizeof eth);
	std::memcpy(buf.data() + sizeof eth, &ip, sizeof ip);
	std::memcpy(buf.data() + sizeof eth + sizeof ip, &udp, sizeof udp);
	std::memcpy(buf.data() + sizeof eth + sizeof ip + sizeof udp,
		kDemoText.data(), kDemoText.size());
	return buf;
}

/* AF_XDP 的一个 socket，连同它的 UMEM 与四张环。
	* 构造即完成“建 socket → 注册 UMEM → 设四个环大小 → mmap 环 → bind”，
	* 任何一步失败抛异常，已取得的资源由 RAII 成员自动释放。 */
class xsk_socket {
public:
	explicit xsk_socket(std::string_view ifname);

	xsk_socket(const xsk_socket &) = delete;
	xsk_socket &operator=(const xsk_socket &) = delete;

	/* 把除 TX 保留区以外的 frame 全部放进 fill 环：
	* 这是 RX 的前提——内核收到包时要从 fill 环取空闲缓冲区。 */
	void fill_all_frames();

	/* 在 tx 环提交一个包；返回 false 表示环满（本演示只发一个，正常不会满）。 */
	bool send_frame(std::span<const std::byte> frame);

	/* 等 TX completion，返回被归还的 frame 偏移；超时返回 nullopt。
	* 每取到一个就把它对应的完成项从 completion 环释放掉。 */
	std::optional<std::uint64_t> wait_tx_completion(std::chrono::milliseconds timeout);

	xdp_statistics statistics() const;

private:
	mapping map_ring(ring &r, std::uint32_t entries, bool has_desc,
		std::uint64_t base_off, const xdp_ring_offset &off,
		std::string_view name);

	fd_handle fd_;
	mapping umem_;
	mapping fill_map_;
	mapping comp_map_;
	mapping rx_map_;
	mapping tx_map_;
	ring fill_{};
	ring comp_{};
	ring rx_{};
	ring tx_{};
	std::uint32_t next_tx_frame_ = 0;
};

mapping xsk_socket::map_ring(ring &r, std::uint32_t entries, bool has_desc,
	std::uint64_t base_off, const xdp_ring_offset &off, std::string_view name)
{
	// 一张环的共享内存布局（同一页起）：producer(u32)、consumer(u32)，
	// 然后是描述符数组；具体偏移以内核给的 off 为准，不要硬编码。
	// 描述符数组之后可能还有 flags 等字段，所以映射长度要覆盖到 flags。
	const std::size_t page = static_cast<std::size_t>(::sysconf(_SC_PAGESIZE));
	std::size_t len = static_cast<std::size_t>(off.flags) + sizeof(std::uint32_t);
	// fill/completion 环的槽位是 __u64 地址，rx/tx 环是 16 字节的 xdp_desc；
	// 两种都要把描述符数组映射进来，只是指针类型不同。
	const std::size_t elem_size = has_desc ? sizeof(xdp_desc) : sizeof(std::uint64_t);
	const std::size_t desc_end = static_cast<std::size_t>(off.desc) +
		static_cast<std::size_t>(entries) * elem_size;
	len = std::max(len, desc_end);
	// mmap 的 offset 必须是页对齐的；XDP_PGOFF_* 都满足。
	len = (len + page - 1) & ~(page - 1);

	void *base = ::mmap(nullptr, len, PROT_READ | PROT_WRITE, MAP_SHARED,
		fd_.get(), static_cast<off_t>(base_off));
	if (base == MAP_FAILED) {
		const int e = errno;
		throw std::runtime_error(std::format("mmap({} ring) 失败：{}", name,
			std::strerror(e)));
	}

	auto *bytes = static_cast<std::byte *>(base);
	r.producer = reinterpret_cast<std::uint32_t *>(bytes + off.producer);
	r.consumer = reinterpret_cast<std::uint32_t *>(bytes + off.consumer);
	r.flags = reinterpret_cast<std::uint32_t *>(bytes + off.flags);
	r.addrs = reinterpret_cast<std::uint64_t *>(bytes + off.desc);
	if (has_desc)
		r.descs = reinterpret_cast<xdp_desc *>(bytes + off.desc);
	r.mask = entries - 1;
	r.cached_prod = 0;
	r.cached_cons = 0;
	return mapping(base, len);
}

xsk_socket::xsk_socket(std::string_view ifname)
{
	// ---- 第 1 步：创建 AF_XDP socket ----
	// SOCK_RAW 是 AF_XDP 唯一支持的类型；protocol 必须为 0。
	// 这一步就会检查 CAP_NET_RAW，普通用户直接 EPERM。
	int raw = ::socket(AF_XDP, SOCK_RAW, 0);
	if (raw < 0) {
		const int e = errno;
		if (e == EPERM || e == EACCES) {
			throw std::runtime_error(std::format(
				"socket(AF_XDP, SOCK_RAW, 0) 失败：{} —— 需要 root 或 CAP_NET_RAW"
				"（本机非 root，这是预期结果，用 sudo 重跑）", std::strerror(e)));
		}
		throw_errno("socket(AF_XDP, SOCK_RAW, 0)", e);
	}
	fd_.reset(raw);
	std::println("[1] socket(AF_XDP, SOCK_RAW, 0) = {}", fd_.get());

	// ---- 第 2 步：分配并注册 UMEM ----
	// UMEM 是用户内存，内核会 pin 住这些页，整段必须一直有效到 socket 关闭。
	// 用匿名 mmap 而不是 new：页对齐、生命周期清晰。
	const std::size_t umem_len = static_cast<std::size_t>(kFrameSize) * kFrameCount;
	void *umem = ::mmap(nullptr, umem_len, PROT_READ | PROT_WRITE,
		MAP_PRIVATE | MAP_ANONYMOUS, -1, 0);
	if (umem == MAP_FAILED) {
		const int e = errno;
		throw_errno("mmap(UMEM)", e);
	}
	umem_ = mapping(umem, umem_len);

	// xdp_umem_reg 各字段：
	//   addr            UMEM 的绝对用户地址
	//   len             UMEM 总长度
	//   chunk_size      每个 frame 的大小（2 的幂）
	//   headroom        每个 frame 头部预留给 XDP 元数据的字节数，0 表示数据从 chunk 头开始
	//   flags           XDP_UMEM_UNALIGNED_CHUNK_FLAG / XDP_UMEM_TX_SW_CSUM 等，这里不用
	//   tx_metadata_len 每个描述符预留的 TX 元数据长度，0 表示不使用
	xdp_umem_reg reg{};
	reg.addr = reinterpret_cast<std::uint64_t>(umem);
	reg.len = umem_len;
	reg.chunk_size = kFrameSize;
	reg.headroom = 0;
	reg.flags = 0;
	reg.tx_metadata_len = 0;
	if (::setsockopt(fd_.get(), SOL_XDP, XDP_UMEM_REG, &reg, sizeof reg) < 0) {
		const int e = errno;
		throw_errno("setsockopt(XDP_UMEM_REG)", e);
	}
	std::println("[2] UMEM: {} 个 frame × {} 字节 = {} KiB，已注册",
		kFrameCount, kFrameSize, umem_len / 1024);

	// ---- 第 3 步：设置四个环的大小 ----
	// 值就是槽位数（u32），内核会向上取整到 2 的幂。必须在 bind 之前设置。
	auto set_ring = [this](int opt, std::uint32_t entries, std::string_view name) {
		if (::setsockopt(fd_.get(), SOL_XDP, opt, &entries, sizeof entries) < 0) {
			const int e = errno;
			throw std::runtime_error(std::format("setsockopt({} ring, {}): {}",
				name, entries, std::strerror(e)));
		}
	};
	set_ring(XDP_UMEM_FILL_RING, kRingEntries, "fill");
	set_ring(XDP_UMEM_COMPLETION_RING, kRingEntries, "completion");
	set_ring(XDP_RX_RING, kRingEntries, "rx");
	set_ring(XDP_TX_RING, kRingEntries, "tx");
	std::println("[3] fill/completion/rx/tx 四张环各 {} 个槽位", kRingEntries);

	// ---- 第 4 步：问内核要各环内部的字段偏移 ----
	// 结构是四个 xdp_ring_offset（rx/tx/fr/cr），每个含 producer/consumer/desc/flags
	// 的字节偏移。不同内核版本布局可能变，所以必须 getsockopt，不能写死。
	xdp_mmap_offsets offs{};
	socklen_t optlen = sizeof offs;
	if (::getsockopt(fd_.get(), SOL_XDP, XDP_MMAP_OFFSETS, &offs, &optlen) < 0) {
		const int e = errno;
		throw_errno("getsockopt(XDP_MMAP_OFFSETS)", e);
	}
	std::println("[4] 环内偏移：rx(prod={}, cons={}, desc={}) tx(prod={}, cons={}, desc={})",
		offs.rx.producer, offs.rx.consumer, offs.rx.desc,
		offs.tx.producer, offs.tx.consumer, offs.tx.desc);

	// ---- 第 5 步：mmap 四张环 ----
	// pgoff：RX 从 0 开始，TX 用 0x80000000，fill 用 0x100000000，
	// completion 用 0x180000000（见 linux/if_xdp.h 的 XDP_PGOFF_*）。
	// 这些“偏移”是 mmap 的文件偏移，同一 socket 的不同窗口。
	fill_map_ = map_ring(fill_, kRingEntries, false, XDP_UMEM_PGOFF_FILL_RING,
		offs.fr, "fill");
	comp_map_ = map_ring(comp_, kRingEntries, false, XDP_UMEM_PGOFF_COMPLETION_RING,
		offs.cr, "completion");
	rx_map_ = map_ring(rx_, kRingEntries, true, XDP_PGOFF_RX_RING, offs.rx, "rx");
	tx_map_ = map_ring(tx_, kRingEntries, true, XDP_PGOFF_TX_RING, offs.tx, "tx");
	std::println("[5] 四张环已 mmap（fill {} B / completion {} B / rx {} B / tx {} B）",
		fill_map_.size(), comp_map_.size(), rx_map_.size(), tx_map_.size());

	// ---- 第 6 步：bind 到接口的某个队列 ----
	// sxdp_family  必须 AF_XDP
	// sxdp_flags   XDP_COPY 强制拷贝模式（驱动把包 memcpy 进 UMEM，兼容性最好；
	//              零拷贝用 XDP_ZEROCOPY）；XDP_USE_NEED_WAKEUP 让驱动在
	//              “我要睡了”时置位环的 flags，用户看到后要 sendto 踢一下，
	//              避免用户态自旋等包时空耗 CPU；
	//              还有 XDP_SHARED_UMEM / XDP_USE_SG，本例不用。
	// sxdp_ifindex 接口索引（if_nametoindex）
	// sxdp_queue_id 接收队列编号，0 表示 0 号队列
	// sxdp_shared_umem_fd 共享 UMEM 时指向创建者的 fd，这里 0
	const std::string name(ifname);
	const unsigned int ifindex = ::if_nametoindex(name.c_str());
	if (ifindex == 0)
		throw std::runtime_error(std::format("找不到网络接口 \"{}\"", name));

	sockaddr_xdp addr{};
	addr.sxdp_family = AF_XDP;
	addr.sxdp_flags = XDP_COPY | XDP_USE_NEED_WAKEUP;
	addr.sxdp_ifindex = ifindex;
	addr.sxdp_queue_id = 0;
	addr.sxdp_shared_umem_fd = 0;
	if (::bind(fd_.get(), reinterpret_cast<sockaddr *>(&addr), sizeof addr) < 0) {
		const int e = errno;
		if (e == EOPNOTSUPP || e == ENODEV) {
			throw std::runtime_error(std::format(
				"bind 到 {} 失败：{} —— 该设备不支持 AF_XDP/XDP（lo 往往没有 ndo_bpf，"
				"换真实网卡或 veth 试试），或接口未 up", name, std::strerror(e)));
		}
		throw_errno(std::format("bind(sockaddr_xdp, {})", name), e);
	}
	std::println("[6] bind: 接口 {} (ifindex {}), queue 0, flags XDP_COPY|XDP_USE_NEED_WAKEUP",
		name, ifindex);

	// ---- 关于“收包”：还需要 XDP 程序，本示例不加载 ----
	// 仅仅 bind 不会让任何包进入 rx 环。收包链路上必须有一个 XDP 程序挂在
	// 该接口（attach 到入口），它对每个包返回 XDP_REDIRECT，并把目标指定为
	// 一个 XSKMAP（BPF map of type BPF_MAP_TYPE_XSKMAP，键是 queue id，
	// 值是 AF_XDP socket 的 fd）。内核在 redirect 时把包放进对应 socket 的
	// rx 环，同时从 fill 环取走一个 frame 作为包的存放位置。
	// 典型 BPF 代码（伪代码，未在本示例中加载）：
	//   int xdp_prog(struct xdp_md *ctx) {
	//       return bpf_redirect_map(&xsks_map, ctx->rx_queue_index, XDP_DROP);
	//   }
	// 它由 libbpf 编译、加载，并用 XDP_FLAGS_* + bpf_set_link_xdp_fd 挂到网卡。
	// 本示例为了只依赖 UAPI，不做这一步；因此 rx 环在本程序里始终为空。
	//
	// 想实际跑通收发，可以在外面用 iproute2 的 xdp-loader 或 libbpf 的
	// xsk 示例把程序挂上，再把对应的 fd 塞进 XSKMAP。
}

void xsk_socket::fill_all_frames()
{
	// fill 环里的内容是“内核可以拿去收包的 frame 偏移”。TX 保留区（前
	// kTxFrames 个）不放进去，避免发送和接收同时用同一块内存。
	const std::uint32_t n = kFrameCount - kTxFrames;
	auto idx = reserve_prod(fill_, n);
	if (!idx) {
		throw std::runtime_error(std::format(
			"fill 环预留 {} 个槽位失败（环太小？）", n));
	}
	for (std::uint32_t i = 0; i < n; ++i) {
		// 注意：描述符存的是“相对 UMEM 起点的字节偏移”，不是指针。
		fill_.addrs[*idx + i] =
			static_cast<std::uint64_t>(i + kTxFrames) * kFrameSize;
	}
	commit_prod(fill_);
	// 如果驱动已经睡了，要踢一下它才会来取这些 frame。
	if (needs_wakeup(fill_))
		kick(fd_.get());
	std::println("[7] fill 环投入 {} 个空闲 frame（frame {}..{}），producer={}",
		n, kTxFrames, kFrameCount - 1, load_relaxed(fill_.producer));
}

bool xsk_socket::send_frame(std::span<const std::byte> frame)
{
	if (frame.size() > kFrameSize)
		throw std::runtime_error("待发包大于一个 frame");

	const std::uint32_t slot = next_tx_frame_++ % kTxFrames;
	auto idx = reserve_prod(tx_, 1);
	if (!idx)
		return false;

	xdp_desc &d = tx_.descs[*idx];
	d.addr = static_cast<std::uint64_t>(slot) * kFrameSize; // UMEM 内偏移
	d.len = static_cast<std::uint32_t>(frame.size());       // 线上帧长
	d.options = 0; // 单缓冲区、无 TX 元数据；多缓冲要置 XDP_PKT_CONTD

	// 把帧内容拷进 UMEM 对应 frame。TX 描述符发布前完成即可。
	std::memcpy(umem_.data() + d.addr, frame.data(), frame.size());

	// 发布描述符：一条 release store 让内核看到完整的 desc 和数据。
	commit_prod(tx_);
	const std::uint64_t addr = d.addr;
	std::println("[8] tx 环提交 1 个包: frame addr={}, len={}", addr, d.len);

	// XDP_USE_NEED_WAKEUP 模式下，如果驱动的 TX 已经休眠，仅在环里放东西
	// 不会触发发送，必须 sendto 踢一下。复制模式下这个调用也会顺带
	// 促成内核立刻处理环内容。
	if (needs_wakeup(tx_))
		kick(fd_.get());
	return true;
}

std::optional<std::uint64_t> xsk_socket::wait_tx_completion(
	std::chrono::milliseconds timeout)
{
	const auto deadline = std::chrono::steady_clock::now() + timeout;
	while (std::chrono::steady_clock::now() < deadline) {
		// 消费方向：一次最多取 8 个完成项。completion 环里是 __u64 偏移，
		// 没有长度，长度要自己从提交时记录（本演示只有一个包）。
		if (auto batch = peek_cons(comp_, 8)) {
			std::uint64_t addr = 0;
			for (std::uint32_t i = 0; i < batch->count; ++i) {
				const std::uint32_t slot = (batch->start + i) & comp_.mask;
				addr = comp_.addrs[slot];
				std::println("[9] TX completion: 内核归还 frame addr={}", addr);
			}
			release_cons(comp_);
			return addr;
		}
		// 没有完成项就睡 1ms，避免空转把 CPU 烧掉。
		std::this_thread::sleep_for(std::chrono::milliseconds(1));
	}
	return std::nullopt;
}

xdp_statistics xsk_socket::statistics() const
{
	xdp_statistics st{};
	socklen_t len = sizeof st;
	if (::getsockopt(fd_.get(), SOL_XDP, XDP_STATISTICS, &st, &len) < 0) {
		const int e = errno;
		throw_errno("getsockopt(XDP_STATISTICS)", e);
	}
	return st;
}

} // namespace

int main(int argc, char **argv)
{
	const std::string ifname = argc > 1 ? argv[1] : "lo";
	std::println("AF_XDP 演示：接口 \"{}\"，UMEM {} frame × {} B，环 {} 槽位",
		ifname, kFrameCount, kFrameSize, kRingEntries);

	try {
		xsk_socket xs(ifname);
		xs.fill_all_frames();

		const auto frame = build_demo_frame();
		std::println("[*] 构造 Ethernet+IPv4+UDP 帧 {} 字节，目的 127.0.0.1:{}",
			frame.size(), kDemoDstPort);
		if (!xs.send_frame(frame)) {
			std::println(stderr, "tx 环满，提交失败");
			return 1;
		}

		const auto done = xs.wait_tx_completion(std::chrono::seconds(2));
		if (!done) {
			std::println(stderr,
				"2 秒内没有收到 TX completion（设备/驱动可能不支持 copy 模式 TX）");
			return 1;
		}

		// 统计项可以帮助判断包为什么没收到：fill 环空、rx 环满、描述符非法等。
		const auto st = xs.statistics();
		std::println("[10] XDP_STATISTICS: rx_dropped={} rx_invalid_descs={}"
			" tx_invalid_descs={} rx_ring_full={} rx_fill_ring_empty_descs={}"
			" tx_ring_empty_descs={}",
			st.rx_dropped, st.rx_invalid_descs, st.tx_invalid_descs,
			st.rx_ring_full, st.rx_fill_ring_empty_descs,
			st.tx_ring_empty_descs);
	} catch (const std::exception &e) {
		// 所有资源已在栈展开时由 RAII 释放：环 munmap、UMEM munmap、fd close。
		std::fflush(stdout); // 先刷出前面打印的步骤，避免与 stderr 乱序
		std::println(stderr, "失败：{}", e.what());
		return 1;
	}

	std::println("演示结束。");
	return 0;
}
