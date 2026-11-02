/*
* bulk_urb.cpp —— usbfs 批量端点的异步 URB：提交、收割、取消、流水线
*
* 一、批量(bulk)传输
*     批量传输用于“数据量大、一点都不能丢、但不关心何时送到”的场合：U 盘、
*     打印机、扫描仪、各种数据采集。USB 协议给它的是：
*       · 可靠：每个包都有握手(ACK/NAK/STALL)，出错自动重传；
*       · 不保证带宽：主机控制器把每帧剩余时间给批量传输，忙时排队；
*       · 没有固定周期：想发就发，设备用 NAK 表示“现在没空”。
*     一次批量传输在线上会被拆成若干个 maxpacket 大小的包（高速批量端点
*     wMaxPacketSize 通常是 512），最后一个包一般短于 maxpacket，接收方
*     看到短包就知道“这次传输结束了”。如果长度正好是 maxpacket 的整数倍，
*     要靠一个零长度包(ZLP)收尾，见 USBDEVFS_URB_ZERO_PACKET。
*
* 二、为什么不用 USBDEVFS_BULK
*     USBDEVFS_BULK 是同步版本：一次 ioctl 只发一个批量传输、线程阻塞到它
*     完成，不能同时挂多个，也不能取消。USBDEVFS_SUBMITURB 把“提交”和
*     “完成”拆开：
*       · 可以同时提交多个 URB（流水线），让总线始终有活干；
*       · 可以在传输途中用 DISCARDURB 取消；
*       · 可以和别的 fd 一起放进事件循环（signr 信号 或 REAPURBNDELAY）。
*     代价是：URB 和它的 buffer 由你自己分配、自己保活、自己收割。
*
* 三、形态
*     · 一次 SUBMITURB：交 1 个 URB，指向 1 段用户内存、1 个端点；
*       线上是 1 次批量传输，被拆成多个包。
*     · 一次 REAPURB：收割 1 个已完成的 URB，对应 1 次提交。
*     · 提交 N 次才完成 N 次；可以有 N 个同时在途（本文件 4 个）。
*     · URB 结构体和数据 buffer 都由用户提供；内核只保存指针，在被收割
*       之前不能释放、不能改大小、不能挪动；内核不拷贝数据。
*     · actual_length/status 是每个 URB 各自的结果，不是一条全局流。
*     · 进程退出/关闭 fd 时，内核会替你把在途 URB 全部结束掉，但那之后
*       你的 buffer 才能释放；正常做法是主动 cancel + reap。
*
* 四、struct usbdevfs_urb 各字段（批量用得到的）
*     type         必须 USBDEVFS_URB_TYPE_BULK(3)。
*     endpoint     端点地址，含方向位：0x81 = IN 端点 1，0x02 = OUT 端点 2。
*                  控制端点 0 不能做批量；方向位决定内核用收/发管道。
*     status       输出：0 成功；负数 = -errno（EPIPE=STALL、ESHUTDOWN=
*                  设备断开、ECONNRESET=被 DISCARDURB 取消、EREMOTEIO=
*                  短包且设了 SHORT_NOT_OK、ETIMEDOUT=控制器超时）。
*     flags        见第五节。
*     buffer       用户数据缓冲；必须一直有效到 URB 被收割。
*     buffer_length 本次传输要发/收多少字节。放在 int 里，别超过 INT_MAX；
*                  老内核可能还有更小的上限，具体值未核对。
*     actual_length 输出：真正传了多少。IN 时小于 buffer_length 就是短包。
*     start_frame  批量不用，填 0。
*     number_of_packets / stream_id
*                  普通批量填 0；用 USBDEVFS_ALLOC_STREAMS 开批量流后，
*                  这个联合体里放 stream_id。
*     error_count  批量一般 0。
*     signr        0 = 完成不发信号；非 0 = 每次完成给进程发该信号，之后
*                  仍要 REAPURB 取结果。
*     usercontext  原样回传的指针，收割时用来找回自己的请求结构。
*     iso_frame_desc[]  柔性数组，只有等时 URB 用（见 iso_urb.cpp）。
*
* 五、flags
*     USBDEVFS_URB_SHORT_NOT_OK (0x01)：IN 传输遇到短包时把 status 变成
*         -EREMOTEIO，而不是当正常结束。需要“没读满就是错”的协议用；
*         默认（不设）短包是合法的“这轮数据结束”信号。
*     USBDEVFS_URB_ZERO_PACKET (0x40)：只对 OUT 有意义。当 buffer_length
*         正好是 maxpacket 整数倍时，多发一个零长度包，让设备明确知道传输
*         结束（大容量存储常用）。需要主机控制器支持，先查
*         USBDEVFS_CAP_ZERO_PACKET。
*     USBDEVFS_URB_BULK_CONTINUATION (0x04)：内核内部拆超大传输时用，
*         用户态不要设。
*     USBDEVFS_URB_NO_INTERRUPT (0x80)：不产生完成中断，减少中断开销，
*         代价是通知变晚（内核内部用得多，语义未核对）。
*
* 六、怎么把完成的 URB 对应回请求
*     内核原样保存你传的 urb 指针，REAPURB 也原样还给你，所以拿指针查表
*     是最直接的办法。但当 URB 是单独 new 出来、请求结构在别处时，用
*     usercontext 更方便：提交前 urb.usercontext = &req；收割时
*     req = static_cast<...>(u->usercontext)。本文件两种都做：取
*     usercontext，再核对 req->urb == u，任何一方错位都能立刻发现。
*
* 七、超时
*     usbdevfs_urb 没有超时字段！提交的批量 URB 会一直等下去（设备拔掉时
*     内核会以 ESHUTDOWN 结束它）。想要超时只能：
*       · REAPURBNDELAY 非阻塞收割 + 自己定时轮询（本文件 IN 路径）；
*       · 或设 signr，用 sigtimedwait/ppoll 等信号。
*     另外 REAPURB 本身没有超时参数，阻塞版可能永远不返回。
*
* 八、取消
*     DISCARDURB 的参数是 urb 指针。它只是“请求取消”：立即返回，URB 之后
*     仍会以 -ECONNRESET（或被正常完成）出现在完成队列里，必须继续
*     REAPURB 才能拿回内存。忘了收割 = 内存泄漏 + 指针悬空。
*
* 九、需要什么
*     /dev/bus/usb/BBB/DDD 可读写（root 或 udev 规则）；端点必须是批量
*     类型；必须先用 CLAIMINTERFACE 把端点所属接口 claim 下来（否则内核
*     返回 EPERM）；接口不能被内核驱动占着（usb-storage/hid 等会返回
*     EBUSY），要么解绑驱动，要么用 USBDEVFS_DISCONNECT_CLAIM（会断开
*     驱动，慎用）。接口号必须和端点所属接口一致，本程序从命令行取。
*
* 十、编译 / 运行
*     g++ -std=c++26 -O2 -Wall -Wextra -o /tmp/bulk_urb bulk_urb.cpp
*     sudo /tmp/bulk_urb /dev/bus/usb/001/003 0x81       # IN 端点 1，接口 0
*     sudo /tmp/bulk_urb /dev/bus/usb/001/003 0x02 1     # OUT 端点 2，接口 1
*     IN：提交 4 个 4096 字节读请求，边收边补，直到短包或 8 个完成；每个
*         完成打印 actual_length/status，短包后取消剩余在途 URB。
*     OUT：发 8 个 512 字节请求，始终保持 4 个在途，打印每个的完成情况。
*     打不开设备节点时打印原因退出。WSL2 默认没有 USB 设备，本机只验证
*     编译和报错路径。
*/

#include <algorithm>
#include <array>
#include <cerrno>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <format>
#include <memory>
#include <optional>
#include <print>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <thread>
#include <utility>

#include <fcntl.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <endian.h>
#include <linux/usb/ch9.h>
#include <linux/usbdevice_fs.h>

// 同时在途的 URB 数：流水线越深越能掩盖单次往返延迟，但也更吃内存、
// 更难处理顺序。批量传输按提交顺序在总线上执行，完成顺序一般也是 FIFO，
// 但不保证；所以下面不依赖完成顺序。
constexpr int kPipeDepth = 4;

// OUT 演示：8 个请求 × 512 字节（512 是高速批量端点的典型 maxpacket）。
constexpr int kOutTotal = 8;
constexpr std::size_t kOutChunk = 512;

// IN 演示：8 个请求 × 4096 字节。
constexpr int kInTotal = 8;
constexpr std::size_t kInChunk = 4096;

// usbfs 的 URB 没有超时字段，IN 路径用 NDELAY 轮询自己实现一个上限，
// 避免设备一直不给数据时程序永远卡住。
constexpr std::chrono::milliseconds kReapTimeout{2000};

// 带 errno 的异常：main 统一捕获，打印人话后以非 0 退出。
class SysError : public std::runtime_error {
public:
	explicit SysError(std::string msg) : std::runtime_error(std::move(msg)) {}
};

static SysError errno_error(std::string_view what, int err)
{
	return SysError(std::format("{}: {} (errno={})", what,
		std::string_view{std::strerror(err)}, err));
}

// RAII 管 fd：析构自动 close；禁止拷贝，允许移动。
// usbfs 的 claim 和所有在途 URB 都绑定在这个 fd 上，close 时内核会一并
// 释放——它是“忘记清理”的最后一道保险，但正常路径仍应显式 release/cancel。
class UniqueFd {
public:
	UniqueFd() = default;
	explicit UniqueFd(int fd) noexcept : fd_(fd) {}
	~UniqueFd() { reset(); }

	UniqueFd(const UniqueFd &) = delete;
	UniqueFd &operator=(const UniqueFd &) = delete;
	UniqueFd(UniqueFd &&other) noexcept : fd_(std::exchange(other.fd_, -1)) {}
	UniqueFd &operator=(UniqueFd &&other) noexcept
	{
		if (this != &other)
			reset(std::exchange(other.fd_, -1));
		return *this;
	}

	void reset(int fd = -1) noexcept
	{
		if (fd_ >= 0)
			::close(fd_);
		fd_ = fd;
	}
	int get() const noexcept { return fd_; }

private:
	int fd_ = -1;
};

// 打开 /dev/bus/usb/BBB/DDD。路径里的 BUG/DDD 是十进制补零三位，和 lsusb
// 的 Bus/Device 一致。打不开时把 errno 翻成人话再抛出。
static UniqueFd open_device(std::string_view path)
{
	std::string p{path};
	int fd = ::open(p.c_str(), O_RDWR | O_CLOEXEC);

	if (fd < 0) {
		int err = errno;
		std::string msg =
			errno_error(std::format("打不开 {}", path), err).what();

		switch (err) {
		case ENOENT:
			msg += "\n  没有这个节点：确认设备已插上；WSL2 默认不把 USB 设备直通进来。";
			msg += "\n  用 lsusb 查 Bus/Device 号，节点是 /dev/bus/usb/<Bus>/<Device>。";
			break;
		case EACCES:
		case EPERM:
			msg += "\n  权限不够：用 sudo 跑，或装 udev 规则把节点给当前用户。";
			break;
		default:
			break;
		}
		throw SysError(std::move(msg));
	}
	return UniqueFd{fd};
}

// 解析命令行里的数字，支持 0x 前缀（端点一般写成 0x81 这种十六进制）。
static unsigned long parse_ulong(std::string_view text, std::string_view what)
{
	std::string tmp{text};
	char *end = nullptr;

	errno = 0;
	unsigned long v = std::strtoul(tmp.c_str(), &end, 0);
	if (errno != 0 || end == tmp.c_str() || *end != '\0')
		throw SysError(std::format("{} 不是合法数字: {}", what, text));
	return v;
}

// 十六进制打印一段字节，用于看 IN 收到的数据。
static void hexdump(std::string_view tag, std::span<const std::byte> bytes)
{
	std::print("{}: ", tag);
	for (std::byte b : bytes)
		std::print("{:02x} ", std::to_integer<unsigned>(b));
	std::println("");
}

// RAII 管“接口已 claim”这件事：构造时 CLAIMINTERFACE，析构时
// RELEASEINTERFACE。批量/中断/等时端点都属于某个接口，不 claim 就提交
// 内核会拒绝（EPERM）。claim 也是独占的：同一个接口同时只能被一个 fd 用。
class ClaimedInterface {
public:
	ClaimedInterface(int fd, unsigned int ifno) : fd_(fd), ifno_(ifno)
	{
		// CLAIMINTERFACE 的参数是指向 unsigned int 的指针，不是值本身。
		unsigned int n = ifno_;

		if (::ioctl(fd_, USBDEVFS_CLAIMINTERFACE, &n) < 0) {
			int err = errno;
			std::string msg =
				errno_error(std::format("claim 接口 {}", ifno_), err).what();

			if (err == EBUSY) {
				msg += "\n  EBUSY：有内核驱动（usb-storage/hid/...）正占着这个接口。";
				msg += "\n  先解绑驱动（往 /sys/bus/usb/drivers/<drv>/unbind 写设备号），";
				msg += "\n  或用 USBDEVFS_DISCONNECT_CLAIM（会断开驱动，慎用）。";
			} else if (err == ENODEV || err == ENOENT) {
				msg += "\n  接口号可能不对：接口号见配置描述符的 bInterfaceNumber。";
			}
			throw SysError(std::move(msg));
		}
	}

	~ClaimedInterface()
	{
		if (fd_ >= 0) {
			unsigned int n = ifno_;
			::ioctl(fd_, USBDEVFS_RELEASEINTERFACE, &n);
		}
	}

	ClaimedInterface(const ClaimedInterface &) = delete;
	ClaimedInterface &operator=(const ClaimedInterface &) = delete;

private:
	int fd_;
	unsigned int ifno_;
};

// 一个批量请求：数据缓冲 + 在途时持有的 URB。
// buffer 用 std::array，地址固定；如果换成 std::vector，提交后千万不能再
// push_back/resize，否则内核手里还是旧地址，数据会写飞。
template <std::size_t Size>
struct IoRequest {
	int id = 0;
	std::array<std::byte, Size> buf{};
	std::unique_ptr<usbdevfs_urb> urb; // 非空 = 该请求正在途中
};

// 提交一个批量 URB。成功返回 true（只代表排队成功），失败返回 false 并保留
// errno。URB 单独 new，因为它的生命周期由“收割”决定，和请求结构解耦。
template <std::size_t Size>
static bool submit_bulk(int fd, IoRequest<Size> &req, unsigned char endpoint,
	unsigned int flags)
{
	auto urb = std::make_unique<usbdevfs_urb>();
	std::memset(urb.get(), 0, sizeof(*urb));

	urb->type = USBDEVFS_URB_TYPE_BULK;
	urb->endpoint = endpoint; // 含 0x80 方向位
	urb->flags = flags; // 短包策略 / ZLP 等，见文件头第五节
	urb->buffer = req.buf.data(); // 用户内存，内核直接往里读写
	urb->buffer_length = static_cast<int>(req.buf.size());
	urb->start_frame = 0; // 批量不用
	urb->number_of_packets = 0; // 普通批量不用（stream_id 也在这个联合体里）
	urb->signr = 0; // 不发信号，靠 REAPURB
	// 关键：收割时靠它找回请求。内核不理解这个指针，只是原样保存。
	urb->usercontext = &req;

	// SUBMITURB 的参数是 urb 指针。内核会把 URB 挂到端点的队列上，
	// 立刻返回；传输结果（status/actual_length）稍后才填。
	if (::ioctl(fd, USBDEVFS_SUBMITURB, urb.get()) < 0)
		return false; // errno 由调用者读

	// 从这一刻起内核引用这块内存，unique_ptr 只是替我们记住它，
	// 收割之前绝不能 reset。
	req.urb = std::move(urb);
	return true;
}

// 收割一个已完成的 URB。
//   timeout 为空：用阻塞版 USBDEVFS_REAPURB，没有超时，可能永远不返回；
//   timeout 有值：用 USBDEVFS_REAPURBNDELAY 轮询，1 ms 一次。
// 成功返回 URB 指针；失败返回 nullptr（errno 说明原因，超时则置 ETIMEDOUT）。
static usbdevfs_urb *reap_one(int fd,
	std::optional<std::chrono::milliseconds> timeout)
{
	if (!timeout) {
		usbdevfs_urb *urb = nullptr;

		// 参数是“指向指针的指针”：内核把完成的 URB 指针写进 urb。
		// 返回的是你提交的那个指针，内核不做拷贝。
		if (::ioctl(fd, USBDEVFS_REAPURB, &urb) < 0)
			return nullptr;
		return urb;
	}

	const auto deadline = std::chrono::steady_clock::now() + *timeout;

	for (;;) {
		usbdevfs_urb *urb = nullptr;

		// NDELAY：完成队列空时立刻返回 -1/EAGAIN，不阻塞。
		if (::ioctl(fd, USBDEVFS_REAPURBNDELAY, &urb) == 0)
			return urb;
		if (errno != EAGAIN)
			return nullptr;
		if (std::chrono::steady_clock::now() >= deadline) {
			errno = ETIMEDOUT;
			return nullptr;
		}
		std::this_thread::sleep_for(std::chrono::milliseconds{1});
	}
}

// 批量端点 STALL 之后，端点会停在 halt 状态，后续请求全部 EPIPE。
// CLEAR_HALT 清掉它；参数同样是指向端点地址的指针（含方向位）。
static void clear_halt(int fd, unsigned char endpoint)
{
	unsigned int ep = endpoint;

	if (::ioctl(fd, USBDEVFS_CLEAR_HALT, &ep) < 0)
		std::println(stderr, "CLEAR_HALT(0x{:02x}) 失败: {}",
			static_cast<unsigned>(endpoint),
			std::string_view{std::strerror(errno)});
}

// 一个 URB 的完成结果。
struct Completion {
	int status;
	int actual;
	int cap;
	bool short_packet; // IN 且 status==0 且 actual < cap
};

// 处理一个刚收割到的 URB：核对对应关系、释放 URB 结构体、STALL 时清 halt。
// 注意：调用后 req->urb 一定为空，URB 已归还给用户态。
template <std::size_t Size>
static Completion finish_urb(int fd, unsigned char endpoint, usbdevfs_urb *urb,
	IoRequest<Size> *req)
{
	Completion c{urb->status, urb->actual_length, urb->buffer_length, false};

	// 一致性检查：内核还回来的指针应当就是提交的那个，usercontext 也应
	// 指回同一个请求。对不上说明生命周期管理有 bug，尽早暴露。
	if (!req || !req->urb || req->urb.get() != urb) {
		std::println(stderr, "警告: URB {} 与请求记录对不上（usercontext={}）",
			static_cast<const void *>(urb),
			static_cast<const void *>(req));

		if (req && req->urb)
			req->urb.reset();
		else
			delete urb; // 兜底，URB 是 new 出来的
		return c;
	}

	// 收割之后内核不再引用它，可以释放 URB 结构体；数据 buffer 在 req 里，
	// 由 req 自己的生命周期管理。
	req->urb.reset();

	if (c.status == 0 && c.actual < c.cap)
		c.short_packet = true;
	if (c.status == -EPIPE)
		clear_halt(fd, endpoint);

	return c;
}

// 收尾：把所有仍在途的请求逐一取消，再把它们收割回来。
// DISCARDURB 只是“请求取消”，不会立刻把 URB 还给你；取消后的 URB 会以
// -ECONNRESET 完成（也可能在取消前已经正常完成），必须继续 REAPURB。
template <std::size_t Size, std::size_t Count>
static int cancel_and_reap(int fd, std::array<IoRequest<Size>, Count> &reqs)
{
	int pending = 0;

	for (auto &req : reqs) {
		if (!req.urb)
			continue;

		if (::ioctl(fd, USBDEVFS_DISCARDURB, req.urb.get()) < 0) {
			// EINVAL = 内核已找不到这个 URB（通常它刚好完成了，正躺在
			// 完成队列里等收割）。这种情况照样会被下面的 REAPURB 收回。
			if (errno != EINVAL)
				std::println(stderr, "DISCARDURB 请求 #{} 失败: {}", req.id,
					std::string_view{std::strerror(errno)});
		}
		++pending;
	}

	int reaped = 0;

	while (pending > 0) {
		usbdevfs_urb *urb = reap_one(fd, std::chrono::milliseconds{1000});

		if (!urb) {
			std::println(stderr, "收尾时还有 {} 个 URB 没收回: {}", pending,
				std::string_view{std::strerror(errno)});
			break;
		}

		auto *req = static_cast<IoRequest<Size> *>(urb->usercontext);

		if (req && req->urb && req->urb.get() == urb) {
			if (urb->status != 0)
				std::println("  被取消的请求 #{}: status={} ({})", req->id,
					urb->status,
					std::string_view{std::strerror(-urb->status)});
			req->urb.reset();
		} else {
			delete urb;
		}
		--pending;
		++reaped;
	}
	return reaped;
}

// OUT 演示：8 个请求，保持 4 个在途，边完成边补，展示流水线。
static void run_out(int fd, unsigned char endpoint)
{
	std::array<IoRequest<kOutChunk>, kOutTotal> reqs;

	// 造可辨认的数据：开头 4 字节小端序号，其余按请求编号填不同字节。
	// 设备固件如果回显，就能看出顺序（批量按提交顺序上线）。
	for (int i = 0; i < kOutTotal; ++i) {
		reqs[i].id = i;

		uint32_t tag = htole32(static_cast<uint32_t>(i));

		std::memcpy(reqs[i].buf.data(), &tag, sizeof(tag));
		std::fill(reqs[i].buf.begin() + static_cast<std::ptrdiff_t>(sizeof(tag)),
			reqs[i].buf.end(), std::byte{static_cast<unsigned char>(0xa0 + i)});
	}

	std::println("批量 OUT：端点 0x{:02x}，{} 个请求 × {} 字节，流水深度 {}",
		static_cast<unsigned>(endpoint), kOutTotal, kOutChunk, kPipeDepth);

	int next = 0; // 下一个要提交的请求
	int inflight = 0; // 已提交未收割
	int completed = 0;

	// 先把流水线灌满：SUBMITURB 异步返回，此时总线上已经有多个请求在跑，
	// 而不是“发一个等一个”地把往返延迟叠起来。
	while (next < kOutTotal && inflight < kPipeDepth) {
		// flags = 0：短包算正常、不追加 ZLP。若协议要求“整数倍长度也要
		// 以零长度包结尾”，这里改成 USBDEVFS_URB_ZERO_PACKET，并先用
		// USBDEVFS_GET_CAPABILITIES 确认主机控制器支持。
		if (!submit_bulk(fd, reqs[next], endpoint, 0)) {
			std::println(stderr, "提交请求 #{} 失败: {}（后续不再提交）", next,
				std::string_view{std::strerror(errno)});
			break;
		}
		++next;
		++inflight;
	}

	// 收割循环：每完成一个就补一个，维持深度。批量 URB 没有超时，
	// 这里用阻塞 REAPURB；真实程序若怕设备失联，可改用下面的定时版本。
	while (inflight > 0) {
		usbdevfs_urb *urb = reap_one(fd, std::nullopt);

		if (!urb) {
			std::println(stderr, "收割失败: {}",
				std::string_view{std::strerror(errno)});
			break;
		}
		--inflight;
		++completed;

		auto *req = static_cast<IoRequest<kOutChunk> *>(urb->usercontext);
		Completion c = finish_urb(fd, endpoint, urb, req);

		if (c.status < 0) {
			std::println(stderr, "请求 #{} 失败: status={} ({})",
				req ? req->id : -1, c.status,
				std::string_view{std::strerror(-c.status)});
		} else {
			std::println("请求 #{} 完成: actual_length={}/{}", req ? req->id : -1,
				c.actual, c.cap);
			if (c.actual != c.cap)
				std::println("  注意：OUT 实际只发了 {} 字节（短写），"
					"但 status 仍是 0", c.actual);
		}

		// 补位：只要还有没提交的，就保持流水线深度。
		if (next < kOutTotal) {
			if (!submit_bulk(fd, reqs[next], endpoint, 0)) {
				std::println(stderr, "提交请求 #{} 失败: {}", next,
					std::string_view{std::strerror(errno)});
				break;
			}
			++next;
			++inflight;
		}
	}

	int reaped = cancel_and_reap(fd, reqs);

	std::println("OUT 结束：完成 {} 个，收尾取消/收回 {} 个", completed, reaped);
}

// IN 演示：同样保持 4 个在途，直到收到短包（这轮数据结束）或用满 8 个。
static void run_in(int fd, unsigned char endpoint)
{
	std::array<IoRequest<kInChunk>, kInTotal> reqs;

	for (int i = 0; i < kInTotal; ++i)
		reqs[i].id = i;

	std::println("批量 IN：端点 0x{:02x}，{} 个请求 × {} 字节，流水深度 {}",
		static_cast<unsigned>(endpoint), kInTotal, kInChunk, kPipeDepth);

	int next = 0;
	int inflight = 0;
	int completed = 0;
	bool stop = false;

	while (next < kInTotal && inflight < kPipeDepth) {
		if (!submit_bulk(fd, reqs[next], endpoint, 0)) {
			std::println(stderr, "提交请求 #{} 失败: {}", next,
				std::string_view{std::strerror(errno)});
			break;
		}
		++next;
		++inflight;
	}

	while (inflight > 0) {
		// 用带超时的收割：设备可能很久不给数据，阻塞版会永远等下去。
		usbdevfs_urb *urb = reap_one(fd, kReapTimeout);

		if (!urb) {
			std::println(stderr, "等完成超时/出错: {}（取消剩余 {} 个）",
				std::string_view{std::strerror(errno)}, inflight);
			break;
		}
		--inflight;

		auto *req = static_cast<IoRequest<kInChunk> *>(urb->usercontext);
		Completion c = finish_urb(fd, endpoint, urb, req);

		++completed;

		if (c.status < 0) {
			std::println(stderr, "请求 #{} 失败: status={} ({})",
				req ? req->id : -1, c.status,
				std::string_view{std::strerror(-c.status)});
		} else {
			std::println("请求 #{} 完成: actual_length={}/{}",
				req ? req->id : -1, c.actual, c.cap);

			// 只打印有效部分：actual_length 之后的内容未定义。
			if (req && c.actual > 0)
				hexdump(std::format("  前 {} 字节", std::min<int>(c.actual, 16)),
					std::span<const std::byte>{req->buf}.first(
						static_cast<std::size_t>(std::min<int>(c.actual, 16))));

			if (c.short_packet) {
				// 短包 = 设备这次没把 buffer 填满。批量 IN 默认把它当成
				// “这一轮数据结束”的正常信号（status 仍是 0）。如果提交时
				// 设了 USBDEVFS_URB_SHORT_NOT_OK，这里会变成 -EREMOTEIO。
				std::println("  短包（{} < {}）：批量 IN 默认视为正常收尾，"
					"status=0", c.actual, c.cap);
				if (c.actual == 0)
					std::println("  actual=0 即零长度包(ZLP)");
				stop = true;
			}
		}

		if (stop) {
			std::println("收到短包，停止补请求，取消剩余在途 URB");
			break;
		}

		// 只有真正收到数据才算“还有更多”，所以短包后再补请求没意义。
		if (next < kInTotal) {
			if (!submit_bulk(fd, reqs[next], endpoint, 0)) {
				std::println(stderr, "提交请求 #{} 失败: {}", next,
					std::string_view{std::strerror(errno)});
				break;
			}
			++next;
			++inflight;
		}
	}

	int reaped = cancel_and_reap(fd, reqs);

	std::println("IN 结束：完成 {} 个，收尾取消/收回 {} 个", completed, reaped);
}

static void run(int argc, char **argv)
{
	if (argc < 3 || argc > 4)
		throw SysError(std::format(
			"用法: {} <设备节点> <端点地址，如 0x81> [接口号，默认 0]", argv[0]));

	UniqueFd fd = open_device(argv[1]);

	// 端点地址支持 0x 前缀；高 4 位里只有 bit7 是方向，其余保留。
	unsigned long ep = parse_ulong(argv[2], "端点地址");

	if (ep > 0xff)
		throw SysError("端点地址超过 8 位（形如 0x81 / 0x02）");

	unsigned char endpoint = static_cast<unsigned char>(ep);
	unsigned int ifno = 0;

	if (argc == 4) {
		unsigned long v = parse_ulong(argv[3], "接口号");

		if (v > 0xffff)
			throw SysError("接口号过大");
		ifno = static_cast<unsigned int>(v);
	}

	// 端点 0 是控制端点，不能做批量传输。
	if ((endpoint & USB_ENDPOINT_NUMBER_MASK) == 0)
		throw SysError("端点 0 是控制端点，批量端点的编号必须 >= 1");

	// 顺便看看 usbfs/主机控制器的能力位，决定能不能用 ZERO_PACKET 等。
	uint32_t caps = 0;

	if (::ioctl(fd.get(), USBDEVFS_GET_CAPABILITIES, &caps) == 0) {
		std::println("usbfs 能力: ZERO_PACKET={} BULK_CONTINUATION={} BULK_SG={}",
			(caps & USBDEVFS_CAP_ZERO_PACKET) ? "yes" : "no",
			(caps & USBDEVFS_CAP_BULK_CONTINUATION) ? "yes" : "no",
			(caps & USBDEVFS_CAP_BULK_SCATTER_GATHER) ? "yes" : "no");
	}

	// claim 必须在 submit 之前；RAII 保证函数退出时 release。
	ClaimedInterface claim(fd.get(), ifno);

	if (endpoint & USB_ENDPOINT_DIR_MASK) {
		std::println("方向: IN（设备→主机），接口 {}", ifno);
		run_in(fd.get(), endpoint);
	} else {
		std::println("方向: OUT（主机→设备），接口 {}", ifno);
		run_out(fd.get(), endpoint);
	}
}

int main(int argc, char **argv)
{
	try {
		run(argc, argv);
		return 0;
	} catch (const SysError &e) {
		std::println(stderr, "{}", e.what());
		return 2;
	}
}
