/*
* iso_urb.cpp —— usbfs 等时(isochronous) URB：一个 URB 带多个包，每包各自结果
*
* 一、等时传输
*     等时传输是为“实时流”设计的：音频采样、视频帧、传感器高速采样。
*     它和批量传输正好相反：
*       · 有带宽保证：提交 URB 时主机控制器会预留带宽，不够就报错；
*       · 没有重传、没有握手：设备没发、发坏了、发晚了，就是丢了；
*       · 固定节奏：每个（微）帧传一个包。全速/低速 1 帧 = 1 ms，
*         高速/超速 1 微帧 = 125 us。包与包之间不会合并、不会重排。
*     所以等时传输只适合“丢一点能忍”的数据；要可靠就得在应用层补
*     （前向纠错、重传、丢帧隐藏）。USB 音频/视频类设备就是典型用户。
*
* 二、一个 URB 里装多个包
*     等时端点一次提交的不是“一段字节”，而是“一串按帧排好的包”：
*       · number_of_packets 告诉内核这个 URB 覆盖几个（微）帧；
*       · iso_frame_desc[i].length 是第 i 个包的容量（输入），
*         actual_length / status 是第 i 个包的结果（输出）；
*       · 内核在一帧里发/收第 0 个包，下一帧第 1 个……直到全部完成，
*         然后整个 URB 才“完成”一次，REAPURB 归还它。
*     所以：1 次 SUBMITURB 提交 1 个 URB = N 个包 = N 个（微）帧；
*           1 次 REAPURB 收割 1 个 URB，然后读 N 组每包结果。
*     提交/完成是 URB 粒度，不是包粒度；包粒度只体现在 iso_frame_desc 里。
*
* 三、数据在 buffer 里怎么摆（有洞！）
*     数据 buffer 是一段连续内存，但每个包的位置是“按容量”固定切好的：
*       buffer + i * length[i] 开始，长 length[i] 字节，是第 i 个包的位置。
*     不是“上一个包实际收了多少，下一个就从哪开始”。所以：
*       · 如果包 1 只收到 100 字节（容量 1024），包 2 的数据仍然从
*         buffer + 2048 开始，buffer + 1024..2047 这一段是垃圾/旧数据；
*       · 包可能整个是空的（actual_length = 0），对应位置就整个是洞；
*       · 读数据必须按每包的 actual_length 读，不能按整块 buffer 读，
*         更不能假设“数据是连续紧凑的”。
*     示意（length 都相等时）：
*       +--------+--------+--------+--------+
*       | 包 0   | 包 1   | 包 2   | 包 3   |  每格 length 字节
*       +--------+--------+--------+--------+
*       ^        ^        ^        ^
*       偏移 0   length   2*len    3*len     （与前面包实际收多少无关）
*
* 四、尾部柔性数组怎么分配
*     struct usbdevfs_urb 的最后是：
*         struct usbdevfs_iso_packet_desc iso_frame_desc[];
*     这是 C 的“柔性数组成员”(flexible array member)：sizeof(struct
*     usbdevfs_urb) 不包含任何包描述，只算到 usercontext 为止。内核提交
*     时只从用户空间拷这一固定大小的头部，然后按 number_of_packets 和
*     “arg + offsetof(struct usbdevfs_urb, iso_frame_desc)”去读包描述。
*     所以必须一次性分配一整块连续内存：
*         sizeof(struct usbdevfs_urb)
*         + number_of_packets * sizeof(struct usbdevfs_iso_packet_desc)
*     再把它当成 usbdevfs_urb* 用，iso_frame_desc[i] 就落在后面。
*     不能把头部和包描述分成两块内存（内核只认这一块）；也不能只分配
*     sizeof(struct usbdevfs_urb) 就写 iso_frame_desc——那会越界。
*     本文件用 new std::byte[总长]{}：new[] 保证满足最大基础对齐，
*     {} 顺便清零（status/actual_length 由内核覆盖，length 由我们填）。
*
* 五、调度：USBDEVFS_URB_ISO_ASAP 与 start_frame
*     · 置 USBDEVFS_URB_ISO_ASAP：让主机控制器挑最近的可用帧开始，
*       不用你算时间；完成时 start_frame 会被改写成实际起始帧。
*       （usbdevice_fs.h 里它的值是 0x02。）
*     · 不置：start_frame 是输入，必须是一个“还没到、又不太远”的帧号，
*       否则提交会被拒绝（具体返回码未核对）。适合音视频同步、固定
*       时间轴。
*     · 持续采集：拿上一个 URB 完成后的 start_frame，下一个设成
*       start_frame + number_of_packets，就可以无缝接上；要更稳就保持
*       2~4 个 URB 在途。跟不上时会出现空包/短包（-ENOENT/-EXDEV）。
*     · 丢了就是丢了：等时端点不会重传，也不会因为一个包坏了把整个
*       URB 判失败（整体 status 可能仍是 0，要看每包 status）。
*
* 六、每包状态（status 是负数 errno）
*     0         正常；
*     -EXDEV    包不完整/短包，前 actual_length 字节有效（数据可用）；
*     -ENOENT   这一帧没有数据（错过了微帧、设备没给），数据不可用；
*     -EOVERFLOW 设备发得比 length 多（babble），多余部分被丢弃；
*     -EPROTO   协议错误；
*     -ESHUTDOWN 端点关闭/设备断开；
*     -ECONNRESET 被 DISCARDURB 取消。
*     具体哪个 HCD 给哪个码可能略有差异（未逐一核对）；应用应只信任
*     status==0 或 -EXDEV 时的前 actual_length 字节。
*     urb->actual_length 是各包 actual_length 之和；urb->error_count 是
*     出错包数（由 HCD 填，是否严格等于非 0 status 的包数未核对）；
*     urb->status 只给整体结果，别拿它替代每包状态。
*
* 七、需要什么
*     /dev/bus/usb/BBB/DDD 可读写；端点是等时类型（bEndpointAddress 的
*     方向位 + bmAttributes bit1..0 = 01）；接口要先 CLAIMINTERFACE。
*     等时端点常常只在非 0 的 altsetting 里（音频设备典型：0 号设置没有
*     带宽，切到 1 号才有等时端点），那种情况要先：
*         usbdevfs_setinterface si{ .interface = ifno, .altsetting = 1 };
*         ioctl(fd, USBDEVFS_SETINTERFACE, &si);
*     再 claim。切换 altsetting 会重新分配带宽，可能失败（ENOSPC，未核对）。
*     每包 length 不能超过端点 wMaxPacketSize（高速高带宽端点还要乘上
*     wMaxPacketSize bit12:11 指定的额外事务数），否则提交返回 EINVAL。
*
* 八、形态 / 编译 / 运行
*     形态：1 个 URB 交 1 段连续内存（按包切槽）、1 个端点；
*           线上 = number_of_packets 个包 = 同样多个（微）帧；
*           提交 1 次、完成 1 次，完成里带 N 组每包结果；
*           头部和包描述、数据 buffer 全部由用户提供并保活。
*     g++ -std=c++26 -O2 -Wall -Wextra -o /tmp/iso_urb iso_urb.cpp
*     sudo /tmp/iso_urb /dev/bus/usb/001/004 0x81           # 8 包 × 1024 字节
*     sudo /tmp/iso_urb /dev/bus/usb/001/004 0x81 1 16 192 # 接口 1，16 包 × 192
*     参数：<设备节点> <端点> [接口号=0] [包数=8] [每包字节=1024]
*     预期输出：URB 总量、起始帧、每包的偏移/容量/实际长度/状态，以及
*     “洞里不是数据”的提示。打不开设备节点时打印原因退出。
*     WSL2 默认没有 USB 设备，本机只能验证编译和报错路径。
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
#include <print>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>

#include <fcntl.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <linux/usb/ch9.h>
#include <linux/usbdevice_fs.h>

// 一个 URB 最多带多少个包：本示例自己限 64，避免栈/堆开销失控；
// 内核允许的上限与 HCD 有关（未核对）。
constexpr int kMaxPackets = 64;

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

// 打开 /dev/bus/usb/BBB/DDD。打不开时把 errno 翻成人话再抛出。
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

// 解析命令行数字，支持 0x 前缀。
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

// 十六进制打印，只看有效字节，避免把“洞”里的旧数据当设备数据。
static void hexdump(std::string_view tag, std::span<const std::byte> bytes)
{
	std::print("{}: ", tag);
	for (std::byte b : bytes)
		std::print("{:02x} ", std::to_integer<unsigned>(b));
	std::println("");
}

// RAII 管 claim：等时端点也必须先 claim 它所属的接口。
class ClaimedInterface {
public:
	ClaimedInterface(int fd, unsigned int ifno) : fd_(fd), ifno_(ifno)
	{
		unsigned int n = ifno_;

		// 参数是指向 unsigned int 的指针。
		if (::ioctl(fd_, USBDEVFS_CLAIMINTERFACE, &n) < 0) {
			int err = errno;
			std::string msg =
				errno_error(std::format("claim 接口 {}", ifno_), err).what();

			if (err == EBUSY) {
				msg += "\n  EBUSY：内核驱动正占着这个接口，先解绑，或用";
				msg += "\n  USBDEVFS_DISCONNECT_CLAIM（会断开驱动，慎用）。";
			} else if (err == ENODEV || err == ENOENT) {
				msg += "\n  接口号可能不对；等时端点常在某个非 0 altsetting 里，";
				msg += "\n  必要时先用 USBDEVFS_SETINTERFACE 切过去。";
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

// 设备速度：等时传输的“帧”长度和它直接相关。
static std::string_view speed_name(int speed)
{
	switch (speed) {
	case USB_SPEED_LOW: return "low speed（1.5 Mbps，帧 1 ms）";
	case USB_SPEED_FULL: return "full speed（12 Mbps，帧 1 ms）";
	case USB_SPEED_HIGH: return "high speed（480 Mbps，微帧 125 us）";
	case USB_SPEED_WIRELESS: return "wireless";
	case USB_SPEED_SUPER: return "super speed（5 Gbps，微帧 125 us）";
	case USB_SPEED_SUPER_PLUS: return "super speed+";
	default: return "unknown";
	}
}

// 每包 status 的人话解释。注意等时 URB 里 status 是负 errno，
// 0 才是正常；一个包坏掉不会让整个 URB 的 ioctl 失败。
static std::string_view iso_status_text(int status)
{
	switch (status) {
	case 0: return "OK";
	case -EXDEV: return "包不完整/短包，前 actual_length 字节有效";
	case -ENOENT: return "这一帧没有数据（丢帧），数据不可用";
	case -EOVERFLOW: return "设备发得比 length 多（babble），已截断";
	case -EPROTO: return "协议错误";
	case -ESHUTDOWN: return "端点关闭/设备断开";
	case -ECONNRESET: return "被取消";
	case -ETIME: return "超时";
	default: return "其它错误";
	}
}

static void run(int argc, char **argv)
{
	if (argc < 3 || argc > 6)
		throw SysError(std::format(
			"用法: {} <设备节点> <端点，如 0x81> [接口号=0] [包数=8] [每包字节=1024]",
			argv[0]));

	UniqueFd fd = open_device(argv[1]);

	unsigned long ep = parse_ulong(argv[2], "端点地址");

	if (ep > 0xff)
		throw SysError("端点地址超过 8 位（形如 0x81）");

	unsigned char endpoint = static_cast<unsigned char>(ep);
	unsigned int ifno = 0;
	int npkt = 8;
	unsigned long pkt_bytes = 1024;

	if (argc >= 4) {
		unsigned long v = parse_ulong(argv[3], "接口号");

		if (v > 0xffff)
			throw SysError("接口号过大");
		ifno = static_cast<unsigned int>(v);
	}
	if (argc >= 5) {
		unsigned long v = parse_ulong(argv[4], "包数");

		if (v == 0 || v > static_cast<unsigned long>(kMaxPackets))
			throw SysError(std::format("包数必须在 1..{} 之间", kMaxPackets));
		npkt = static_cast<int>(v);
	}
	if (argc >= 6) {
		pkt_bytes = parse_ulong(argv[5], "每包字节");

		if (pkt_bytes == 0 || pkt_bytes > (1UL << 20))
			throw SysError("每包字节数必须在 1..1048576 之间");
	}

	// 端点 0 是控制端点，不可能做等时传输。
	if ((endpoint & USB_ENDPOINT_NUMBER_MASK) == 0)
		throw SysError("端点 0 是控制端点，等时端点的编号必须 >= 1");

	const bool in = (endpoint & USB_ENDPOINT_DIR_MASK) != 0;
	const std::size_t per_packet = static_cast<std::size_t>(pkt_bytes);
	const std::size_t data_bytes = per_packet * static_cast<std::size_t>(npkt);

	if (data_bytes > static_cast<std::size_t>(INT32_MAX))
		throw SysError("buffer_length 超过 int 能表示的范围");

	// 打印速度，方便判断每个包对应多长时间。
	int speed = ::ioctl(fd.get(), USBDEVFS_GET_SPEED);

	if (speed > 0)
		std::println("设备速度: {}", speed_name(speed));

	// 分配“头部 + N 个包描述”：这是柔性数组的标准用法。
	// sizeof(usbdevfs_urb) 不含 iso_frame_desc[]，必须自己加。
	const std::size_t desc_bytes =
		static_cast<std::size_t>(npkt) * sizeof(usbdevfs_iso_packet_desc);
	const std::size_t urb_bytes = sizeof(usbdevfs_urb) + desc_bytes;

	// new[] 返回的内存满足最大基础对齐；{} 清零。
	// unique_ptr 负责释放，异常路径也不会漏。
	std::unique_ptr<std::byte[]> storage{new std::byte[urb_bytes]{}};
	auto *urb = reinterpret_cast<usbdevfs_urb *>(storage.get());

	// 数据区也是用户提供的一整块连续内存；IN 时清零只是为了演示好读，
	// 内核只会覆盖每包的前 actual_length 字节。
	std::unique_ptr<std::byte[]> data{new std::byte[data_bytes]{}};

	// 填 URB 头部字段。
	urb->type = USBDEVFS_URB_TYPE_ISO; // 0，等时
	urb->endpoint = endpoint; // 含 0x80 方向位
	urb->status = 0; // 输出：整体结果
	urb->flags = USBDEVFS_URB_ISO_ASAP; // 让 HCD 挑最近的可用帧
	urb->buffer = data.get(); // 连续数据区，按包切槽
	urb->buffer_length = static_cast<int>(data_bytes); // 所有包长度之和
	urb->actual_length = 0; // 输出：实际收到的总字节
	urb->start_frame = 0; // ISO_ASAP 下是输出：完成时写实际起始帧
	urb->number_of_packets = npkt; // 这个 URB 覆盖几个（微）帧
	urb->error_count = 0; // 输出：出错包数
	urb->signr = 0; // 不发信号，靠 REAPURB
	urb->usercontext = nullptr; // 只有一个 URB，用不到

	// 每包描述：length 是输入（容量），actual_length/status 是输出。
	// 它们紧跟在头部后面，空间已经在上面一次性分配好了。
	for (int i = 0; i < npkt; ++i) {
		auto &p = urb->iso_frame_desc[i];

		p.length = static_cast<unsigned int>(per_packet);
		p.actual_length = 0;
		p.status = 0;
	}

	// OUT 等时（少见，但 API 一样）：先把每个包槽填上可辨认的数据。
	// 注意每个包仍然固定在自己的槽里，不会因为“上一包少发”而移动。
	if (!in) {
		for (int i = 0; i < npkt; ++i)
			std::fill_n(data.get() + static_cast<std::size_t>(i) * per_packet,
				per_packet, std::byte{static_cast<unsigned char>(0x40 + i)});
	}

	// 方向只影响“我们先填数据还是等设备填数据”，API 完全一样。
	const std::string_view direction = in ? "IN" : "OUT";

	std::println("URB: 头部 {} 字节 + {} 个包描述 × {} = {} 字节；"
		"数据区 {} 字节（{} 包 × {}）",
		sizeof(usbdevfs_urb), npkt, sizeof(usbdevfs_iso_packet_desc),
		urb_bytes, data_bytes, npkt, per_packet);
	std::println("方向: {}，接口 {}，起始帧 {}{}", direction, ifno, 0,
		"（ISO_ASAP：由主机控制器决定，完成时回填）");

	// 等时端点同样要先 claim 接口，否则提交被拒（EPERM）。
	ClaimedInterface claim(fd.get(), ifno);

	// 提交：内核按 URB 里排好的包逐帧发送/接收。成功只代表排上队。
	if (::ioctl(fd.get(), USBDEVFS_SUBMITURB, urb) < 0)
		throw errno_error("USBDEVFS_SUBMITURB(ISO)", errno);

	// 完成：整个 URB 的所有包都走完后才返回一次。
	// 等时 URB 没有超时；正常几毫秒内完成，设备拔掉会以 ESHUTDOWN 结束。
	// 想同时做别的事情，就用 REAPURBNDELAY 轮询或设 signr（见 bulk_urb.cpp）。
	usbdevfs_urb *done = nullptr;
	int rc;

	// 被信号打断时（EINTR）URB 仍在途，必须重试；否则后面的清理会释放
	// 内核还准备回写结果的内存。
	do {
		rc = ::ioctl(fd.get(), USBDEVFS_REAPURB, &done);
	} while (rc < 0 && errno == EINTR);

	if (rc < 0)
		throw errno_error("USBDEVFS_REAPURB(ISO)", errno);
	if (done != urb)
		throw SysError(std::format("收割到意外的 URB {}（期望 {}）",
			static_cast<const void *>(done), static_cast<const void *>(urb)));

	std::println("\nURB 完成: status={}（{}）actual_length={} error_count={} "
		"start_frame={}",
		urb->status,
		urb->status < 0 ? std::string_view{std::strerror(-urb->status)} :
			std::string_view{"整体正常"},
		urb->actual_length, urb->error_count, urb->start_frame);

	// 逐包看结果。整块 buffer 里只有每包 actual_length 指定的前缀有效，
	// 其余是洞（这里因为清零了所以是 0，真实场景可能是旧数据）。
	unsigned long sum = 0;
	int bad = 0;

	for (int i = 0; i < npkt; ++i) {
		const auto &p = urb->iso_frame_desc[i];
		const std::size_t off = static_cast<std::size_t>(i) * per_packet;

		std::println("  包 {:2}: 偏移 {:6} 容量 {:5} 实际 {:5} status={:4} {}",
			i, off, p.length, p.actual_length, p.status,
			iso_status_text(p.status));

		if (p.actual_length == 0)
			std::print("         （空包：这一槽没有有效数据）\n");
		sum += p.actual_length;
		if (p.status != 0)
			++bad;
	}

	std::println("  各包 actual_length 之和 = {}，urb.actual_length = {}，"
		"出错包数 = {}/{}", sum, urb->actual_length, bad, npkt);

	if (in) {
		// 只打印第一个包的有效前缀，演示“按 actual_length 读”。
		const auto &p0 = urb->iso_frame_desc[0];

		if (p0.actual_length > 0) {
			std::size_t n = std::min<std::size_t>(p0.actual_length, 16);

			hexdump(std::format("包 0 前 {} 字节", n),
				std::span<const std::byte>{data.get(), n});
		}
	}

	// 把“洞”讲清楚：位置由每包容量决定，和数据实际长度无关。
	std::println("\n数据布局: 包 i 永远在 buffer + i×{}，前面的包短了/空了"
		"也不会把后面的数据往前挪。", per_packet);
	std::println("所以只能按每包 actual_length 取数据；洞里的字节不属于设备。");
	std::println("持续采集时，把下一个 URB 的 start_frame 设为 {}（本次起始帧"
		"+ 包数），并保持 2~4 个 URB 在途，就能接上；跟不上会出现 -ENOENT。",
		urb->start_frame + npkt);
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
