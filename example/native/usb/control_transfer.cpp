/*
* control_transfer.cpp —— usbfs 控制传输：USBDEVFS_CONTROL 同步调用，以及异步控制 URB

* 一、USB 的设备模型（读这个文件需要的最小模型）
*	USB 是主机(host)与设备(device)之间的主从总线：只有主机发起传输，设备
*	永远被动应答。主机侧看到的是一棵由 hub 组成的树，每个设备在树里有一个
*	地址（1..127；0 是刚上电还没分配地址时的默认地址）。设备用“描述符”
*	(descriptor) 描述自己，每个描述符都是一小段布局固定的字节：
*	  · 设备(device)：厂商 ID、产品 ID、USB 版本、可用配置数……
*	    struct usb_device_descriptor，18 字节，定义在 <linux/usb/ch9.h>。
*	  · 配置(configuration)：一套可用的接口组合，附带供电方式与电流上限。
*	    struct usb_config_descriptor，9 字节；一个配置后面跟着若干接口、
*	    端点描述符，合起来是一段连续字节（所以要按 wTotalLength 整体读）。
*	  · 接口(interface)：一个“功能”，比如 HID 键盘、音频播放、CDC 串口。
*	    一个接口可以有多个备选设置(altsetting)，同一时刻只有一个生效，
*	    用 SET_INTERFACE 切换（音频设备靠它切不同采样率/带宽）。
*	  · 端点(endpoint)：真正收发数据的“管道末端”，除端点 0 外都是单向的。
*	    地址 bEndpointAddress = 方向位 bit7（0x80 = IN 设备→主机）+ 编号
*	    bit3..0；bmAttributes bit1..0 是类型（控制/等时/批量/中断）；
*	    wMaxPacketSize 是这个端点一个包最多多少字节。
*	一个设备同一时刻只有一个配置生效（SET_CONFIGURATION），一个接口同一
*	时刻只有一个 altsetting 生效。端点 0 是默认控制端点，双向，每个设备
*	都有，专门用来做枚举和所有控制请求，不需要在描述符里声明。

* 二、四种传输类型
*	· 控制(control)     ：端点 0 专用，枚举、读写描述符、类/厂商命令。
*	                       有握手、有重传，主机保证送达；每帧预留一小块带宽。
*	· 批量(bulk)        ：大数据、要求不丢（U 盘、打印机）。没有带宽保证，
*	                       有空闲带宽就用，可靠靠握手 + 重传（见 bulk_urb.cpp）。
*	· 中断(interrupt)   ：小数据、要求有界延迟（键盘、鼠标）。主机周期性轮询。
*	· 等时(isochronous) ：固定速率、允许丢（音频、视频）。不重传、无握手，
*	                       每帧（全速 1 ms / 高速 125 us 微帧）一个包，
*	                       带宽要预先预约（见 iso_urb.cpp）。

* 三、控制传输的三个阶段（硬件层面，一次控制传输固定这样走）
*	1. SETUP ：主机发 8 字节 setup 包：bRequestType、bRequest、wValue(LE16)、
*	            wIndex(LE16)、wLength(LE16)。这是唯一一个一定由主机发起、
*	            一定发往设备的阶段，方向恒为 OUT。
*	2. DATA  ：0 到 wLength 字节。方向由 bRequestType 的 bit7 决定：
*	            0x80 = IN（设备→主机），0 = OUT（主机→设备）。
*	            wLength == 0 时没有这个阶段（如 SET_CONFIGURATION）。
*	3. STATUS：一个方向与 DATA 相反、长度为 0 的握手，由硬件自动完成，
*	            用户态看不见。DATA 是 IN，STATUS 就由主机发 OUT；DATA 是
*	            OUT 或不存在，STATUS 就由设备发 IN。
*	所以“读设备描述符”= setup(标准 IN 请求 GET_DESCRIPTOR, wLength=18)
*	+ 18 字节 DATA + 自动 STATUS，一次 USBDEVFS_CONTROL 调用全做完。

* 四、bRequestType 的 8 个位（本文件里 = 0x80）
*	bit 7     方向     0x80 = IN（设备→主机），0x00 = OUT（主机→设备）
*	bit 6:5   请求类型 00 = 标准(standard) 01 = 类(class) 10 = 厂商(vendor)
*	                  11 = 保留
*	bit 4:0   接收者   0 = 设备，1 = 接口，2 = 端点，3 = 其它(Other)
*	                  4 = 端口(Port，无线 USB)，5 = RPipe
*	对应 ch9.h 的 USB_DIR_IN / USB_TYPE_STANDARD / USB_RECIP_DEVICE。

* 五、usbfs 是什么
*	Linux 把每个 USB 设备暴露成 /dev/bus/usb/BBB/DDD 字符设备（BBB = 总线
*	号，DDD = 设备地址，十进制补零三位，和 lsusb 输出里的 Bus/Device 一致）。
*	对这个 fd 做 ioctl 就能绕过（或者配合）内核驱动直接跟设备说话。libusb
*	在 Linux 上就是这些 ioctl 的包装；这里直接用原生接口，不链接 libusb。
*	usbfs 是内核内置功能（CONFIG_USB_DEVICEFS），现代发行版内核都有；需要
*	root 或 udev 规则给的读写权限。发给“设备”的标准请求不需要 claim 接口；
*	针对某个接口/端点的类/厂商请求要先 CLAIMINTERFACE（见 bulk_urb.cpp），
*	否则内核会拒绝（check_ctrlrecip，具体 errno 未核对）。

* 六、usbdevfs_ctrltransfer 各字段（同步版本）
*	bRequestType  见第四节，方向/类型/接收者。
*	bRequest      请求码，标准请求见 ch9.h 的 USB_REQ_*；GET_DESCRIPTOR=0x06。
*	wValue        16 位，含义由请求决定。GET_DESCRIPTOR 时高字节是描述符
*	              类型（USB_DT_DEVICE=1、CONFIG=2、STRING=3、INTERFACE=4、
*	              ENDPOINT=5），低字节是该类型的索引。
*	wIndex        16 位，含义由请求决定。GET_DESCRIPTOR 读设备描述符填 0；
*	              读字符串描述符填语言 ID（0x0409 = 美式英语）；读接口相关
*	              描述符时填接口号。
*	wLength       16 位，DATA 阶段最多多少字节（设备最多回这么多）。
*	timeout       毫秒；0 表示一直等（内核 proc_control 对 0 不设超时，
*	              未逐行核对）。超时后返回 -1/ETIMEDOUT。
*	data          用户缓冲区：IN 时内核把设备数据写进来，OUT 时从这里读走。
*	              方向只由 bRequestType.bit7 决定，与 wLength 无关。
*	所有 16 位字段在总线上都是小端，主机上要用 htole16()/le16toh()。

* 七、本文件演示什么
*	同一件事——标准请求 GET_DESCRIPTOR 读 18 字节设备描述符——做两遍：
*	  1) 同步：一次 ioctl(USBDEVFS_CONTROL)，阻塞到三个阶段全部结束。内核
*	     内部其实就是替你构造一个控制 URB、提交、等完成（drivers/usb/core/
*	     devio.c 的 proc_control），所以它和 2) 在线上完全一样。
*	  2) 异步：自己分配 struct usbdevfs_urb，buffer 前 8 字节摆 setup 包、
*	     后面是数据区，SUBMITURB 提交，REAPURB 收割。同步能做的它都能做；
*	     拆开的意义是能同时挂多个 URB、能和其它 fd 一起等（信号/事件循环），
*	     不必让一个线程卡在一次往返里。
*	形态对比：
*	  同步：1 次 ioctl 提交，1 次 ioctl 返回 = 完成 1 次；线程阻塞；
*	        DATA 缓冲区由用户提供（usbdevfs_ctrltransfer.data）。
*	  异步：1 次 SUBMITURB 提交 1 个 URB，1 次 REAPURB 收割 1 次完成；
*	        URB 结构体和 buffer（8 字节 setup + 数据）都由用户提供，
*	        且必须在被收割之前一直有效。
*	线上单元：两者都是 1 次控制传输 = setup 包 + 若干数据包 + status 包。
*	C++ 部分：UniqueFd 用 RAII 管 fd（异常/提前 return 都会 close），
*	std::array/std::span 管缓冲区并带上长度，std::format/std::println 输出。

* 八、编译 / 运行
*	g++ -std=c++26 -O2 -Wall -Wextra -o /tmp/control_transfer control_transfer.cpp
*	sudo /tmp/control_transfer /dev/bus/usb/001/003
*	预期输出：先打印同步读到的设备描述符原始字节和各字段解读，再用异步
*	控制 URB 读一遍，打印同样的内容，最后确认两次逐字节一致。打不开设备
*	节点时打印原因并退出。WSL2 默认不直通 USB 设备，本机只能验证报错路径。
*/

#include <array>
#include <cerrno>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <format>
#include <limits>
#include <print>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include <fcntl.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <endian.h>
#include <linux/usb/ch9.h>
#include <linux/usbdevice_fs.h>

// 异步控制 URB 的 buffer 前 8 字节必须是 USB 规范里的 setup 包，也就是
// ch9.h 的 usb_ctrlrequest。它被声明为 packed，sizeof 必须正好 8；
// 用编译期断言把“凭记忆的假设”交给编译器验证。
static_assert(sizeof(usb_ctrlrequest) == 8, "setup 包必须是 8 字节");

// 带 errno 的异常：main 统一捕获，打印人话后以非 0 退出。
// usbfs 的错误都是“调用失败 + errno”，用异常传递比层层返回错误码干净。
class SysError : public std::runtime_error {
public:
	explicit SysError(std::string msg) : std::runtime_error(std::move(msg)) {}
};

static SysError errno_error(std::string_view what, int err)
{
	return SysError(std::format("{}: {} (errno={})", what,
		std::string_view{std::strerror(err)}, err));
}

// RAII 管 fd：析构自动 close；禁止拷贝（否则会 double close），允许移动。
// usbfs 的 claim 与 fd 绑定，fd 一关内核就释放所有 claim 和在途 URB，
// 所以这个类同时也是“忘记释放”的最后一道保险。
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
	explicit operator bool() const noexcept { return fd_ >= 0; }

private:
	int fd_ = -1;
};

// 打开设备节点。格式 /dev/bus/usb/BBB/DDD，BBB/DDD 是十进制补零三位，
// 和 lsusb 里的 Bus/Device 对得上。打不开时把 errno 翻成人话再抛出。
// WSL2 默认没有 USB 设备，本机跑这个程序走的就是失败分支。
static UniqueFd open_device(std::string_view path)
{
	// O_CLOEXEC：避免 fork/exec 出来的子进程继承这个 fd（usbfs 的 claim
	// 跟着 fd 走，被继承会很难排查）。
	std::string p{path};
	int fd = ::open(p.c_str(), O_RDWR | O_CLOEXEC);

	if (fd < 0) {
		int err = errno;
		std::string msg = std::format("打不开 {}: {} (errno={})", path,
			std::string_view{std::strerror(err)}, err);

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

// 十六进制打印，用来对比同步和异步两次读到的原始描述符字节。
static void hexdump(std::string_view tag, std::span<const std::byte> bytes)
{
	std::print("{}（{} 字节）:", tag, bytes.size());
	for (std::byte b : bytes)
		std::print(" {:02x}", std::to_integer<unsigned>(b));
	std::println("");
}

// 用 USBDEVFS_CONTROL 同步读一个设备描述符。
// 返回实际读到的字节数；控制传输允许合法的短 DATA 阶段（设备只回一部分），
// 所以可能小于 out.size()。失败抛异常。
static int get_device_descriptor_sync(int fd, std::span<std::byte> out)
{
	usbdevfs_ctrltransfer ctrl{};

	// bRequestType 三段拼起来：0x80(IN) | 0x00(标准) | 0x00(设备)
	// = 0x80。见文件头第四节。
	ctrl.bRequestType = USB_DIR_IN | USB_TYPE_STANDARD | USB_RECIP_DEVICE;

	// bRequest = GET_DESCRIPTOR(0x06)，ch9.h 的 USB_REQ_GET_DESCRIPTOR。
	ctrl.bRequest = USB_REQ_GET_DESCRIPTOR;

	// wValue：高字节描述符类型 USB_DT_DEVICE(1)，低字节索引 0 → 0x0100。
	// 总线上是小端，用 htole16() 摆字节序（x86 上等价于原值）。
	ctrl.wValue = htole16(static_cast<uint16_t>(USB_DT_DEVICE << 8));

	// wIndex：设备描述符不用索引，填 0。读字符串描述符时这里才是语言 ID。
	ctrl.wIndex = htole16(0);

	// wLength：DATA 阶段最大字节数；设备描述符固定 18 字节。
	ctrl.wLength = htole16(static_cast<uint16_t>(out.size()));

	// timeout：毫秒。0 表示无限等待；这里 1 秒足够。设备拔掉时内核会提前
	// 以 ENODEV 结束等待，所以不必担心真卡满 1 秒。
	ctrl.timeout = 1000;

	// data：用户缓冲区。IN 时内核把设备数据写进来（OUT 时从这读走）。
	// 方向只看 bRequestType.bit7，和 wLength 是否为 0 无关。
	ctrl.data = out.data();

	// 一次 ioctl 完成 setup + DATA + STATUS 三个阶段，阻塞到全部结束。
	// 返回值解读：
	//   >= 0  实际传输的 DATA 字节数
	//   -1    errno：EPIPE=设备 STALL（不支持该请求）、ETIMEDOUT=超时、
	//         ENODEV/ESHUTDOWN=设备断开、EINVAL=请求不合法（如 wLength
	//         超过内核限制）、EPERM/EBUSY=接口未 claim 或被占用。
	int n = ::ioctl(fd, USBDEVFS_CONTROL, &ctrl);

	if (n < 0)
		throw errno_error("USBDEVFS_CONTROL(GET_DESCRIPTOR)", errno);
	return n;
}

// 用异步控制 URB 读同样的内容。内核要求：buffer 前 8 字节是 setup 包，
// 紧跟其后是 DATA 区；buffer_length 必须 >= 8 + wLength（多给少给都会被
// 拒绝，具体返回码未核对）。成功返回 DATA 阶段字节数，并把数据拷到 out。
static int get_device_descriptor_urb(int fd, std::span<std::byte> out)
{
	// 缓冲区用 vector：长度运行时才知道；构造之后绝不能再 resize，
	// 因为内核在 URB 完成前一直引用这块内存的地址。
	std::vector<std::byte> buf(8 + out.size(), std::byte{0});

	// URB 结构体放栈上：它没有柔性数组（number_of_packets=0），
	// 而且必须活到 REAPURB 把它还给我们。
	usbdevfs_urb urb{};

	// setup 包：字段和同步版本完全相同，只是这里要亲手按字节摆好。
	// vector<std::byte> 的 data() 只保证 alignof(std::byte)=1；好在
	// usb_ctrlrequest 是 packed、对齐要求为 1，强转没有对齐问题。
	auto *setup = reinterpret_cast<usb_ctrlrequest *>(buf.data());

	setup->bRequestType = USB_DIR_IN | USB_TYPE_STANDARD | USB_RECIP_DEVICE;
	setup->bRequest = USB_REQ_GET_DESCRIPTOR;
	setup->wValue = htole16(static_cast<uint16_t>(USB_DT_DEVICE << 8));
	setup->wIndex = htole16(0);
	setup->wLength = htole16(static_cast<uint16_t>(out.size()));

	// 填 usbdevfs_urb（字段含义见 usbdevice_fs.h）：
	urb.type = USBDEVFS_URB_TYPE_CONTROL; // 控制传输（0=ISO 1=INT 2=CTRL 3=BULK）
	urb.endpoint = 0; // 端点 0；控制方向来自 setup.bRequestType，不看这里
	urb.status = 0; // 输出字段：完成时内核/驱动写入 0 或负 errno
	urb.flags = 0; // SHORT_NOT_OK/ZERO_PACKET 等对控制 URB 无意义
	urb.buffer = buf.data(); // 8 字节 setup + 数据区
	urb.buffer_length = static_cast<int>(buf.size()); // 用户内存总长（含 setup）
	urb.actual_length = 0; // 输出：只算 DATA 字节，不含 setup 的 8 字节
	urb.start_frame = 0; // 仅等时 URB 有意义
	urb.number_of_packets = 0; // 仅等时 URB 有意义；bulk streams 时复用为 stream_id
	urb.error_count = 0;
	urb.signr = 0; // 0 = 完成时不发信号；本程序靠 REAPURB 阻塞等
	urb.usercontext = nullptr; // 原样回传的指针；只有一个 URB 时用不到

	// SUBMITURB 的参数是 urb 指针本身（不是 &urb）。成功只表示“排队成功”：
	// 传输还没发生，设备接不接受也不知道；结果要等 REAPURB。
	if (::ioctl(fd, USBDEVFS_SUBMITURB, &urb) < 0)
		throw errno_error("USBDEVFS_SUBMITURB", errno);

	// REAPURB 的参数是“指向指针的指针”：内核把完成的 URB 指针写进 done。
	// 它阻塞到至少一个 URB 完成为止（没有超时参数，这是 usbfs 的一个坑：
	// 想超时就用 REAPURBNDELAY + 自己定时轮询，见 bulk_urb.cpp）。
	// 返回的指针就是你提交的那个，内核不做拷贝。
	usbdevfs_urb *done = nullptr;
	int rc;

	// 被信号打断时（EINTR）URB 仍在途，不能放弃，重试直到拿到结果。
	do {
		rc = ::ioctl(fd, USBDEVFS_REAPURB, &done);
	} while (rc < 0 && errno == EINTR);

	if (rc < 0)
		throw errno_error("USBDEVFS_REAPURB", errno);
	if (done != &urb)
		throw SysError(std::format("收割到意外的 URB {}（期望 {}）",
			static_cast<const void *>(done), static_cast<const void *>(&urb)));

	// 完成后的状态：
	//   urb.status        0 成功；负数就是 -errno。注意和同步 ioctl 的
	//                     “-1 + errno” 约定不同，异步 URB 直接把负 errno
	//                     放在 status 里，别混着写。
	//   urb.actual_length DATA 阶段实际字节数（不含 setup）。
	if (urb.status != 0)
		throw SysError(std::format("控制 URB 完成但失败: status={} ({})",
			urb.status, std::string_view{std::strerror(-urb.status)}));
	if (urb.actual_length < 0 ||
			static_cast<size_t>(urb.actual_length) > out.size())
		throw SysError(std::format("actual_length={} 不合理（len={}）",
			urb.actual_length, out.size()));

	// 设备数据从 buf + 8 开始，长度 actual_length。
	std::memcpy(out.data(), buf.data() + 8,
		static_cast<size_t>(urb.actual_length));
	return urb.actual_length;
}

// 解读设备描述符。所有 16 位字段在总线上是小端，主机上用 le16toh() 转换。
// 结构体是 packed 的，直接往读到的字节缓冲区上强转是安全的（对齐要求 1）。
static void print_device_descriptor(std::span<const std::byte> raw)
{
	static_assert(sizeof(usb_device_descriptor) == USB_DT_DEVICE_SIZE);

	if (raw.size() < sizeof(usb_device_descriptor))
		throw SysError(std::format("描述符只有 {} 字节，不够 {} 字节",
			raw.size(), sizeof(usb_device_descriptor)));

	const auto *d = reinterpret_cast<const usb_device_descriptor *>(raw.data());
	unsigned usbver = le16toh(d->bcdUSB);
	unsigned vendor = le16toh(d->idVendor);
	unsigned product = le16toh(d->idProduct);
	unsigned devver = le16toh(d->bcdDevice);

	std::println("  bLength            = {}（描述符自身长度，设备描述符固定 18）",
		static_cast<unsigned>(d->bLength));
	std::println("  bDescriptorType    = {}（1 = USB_DT_DEVICE）",
		static_cast<unsigned>(d->bDescriptorType));
	std::println("  bcdUSB             = 0x{:04x}（USB {}.{:02}）",
		usbver, usbver >> 8, usbver & 0xff);
	std::println("  bDeviceClass       = 0x{:02x}（0 = 由接口描述，其它见 USB_CLASS_*）",
		static_cast<unsigned>(d->bDeviceClass));
	std::println("  bDeviceSubClass    = 0x{:02x}",
		static_cast<unsigned>(d->bDeviceSubClass));
	std::println("  bDeviceProtocol    = 0x{:02x}",
		static_cast<unsigned>(d->bDeviceProtocol));
	std::println("  bMaxPacketSize0    = {}（端点 0 单包上限）",
		static_cast<unsigned>(d->bMaxPacketSize0));
	std::println("  idVendor           = 0x{:04x}（去 usb.ids 能查厂商）", vendor);
	std::println("  idProduct          = 0x{:04x}", product);
	std::println("  bcdDevice          = 0x{:04x}（设备自己的版本号）", devver);
	std::println("  iManufacturer      = {}（字符串描述符索引，0 = 没有）",
		static_cast<unsigned>(d->iManufacturer));
	std::println("  iProduct           = {}", static_cast<unsigned>(d->iProduct));
	std::println("  iSerialNumber      = {}",
		static_cast<unsigned>(d->iSerialNumber));
	std::println("  bNumConfigurations = {}（设备一共几套配置，同一时刻只用一套）",
		static_cast<unsigned>(d->bNumConfigurations));
}

static void run(int argc, char **argv)
{
	if (argc != 2)
		throw SysError(std::format("用法: {} <设备节点，如 /dev/bus/usb/001/003>",
			argv[0]));

	std::array<std::byte, USB_DT_DEVICE_SIZE> raw_sync{};
	std::array<std::byte, USB_DT_DEVICE_SIZE> raw_urb{};

	// 用 O_RDWR 打开：只读打开在部分芯片上会被拒绝 OUT 阶段（STATUS）。
	UniqueFd fd = open_device(argv[1]);

	std::println("== 1) USBDEVFS_CONTROL 同步读设备描述符 ==");
	int n = get_device_descriptor_sync(fd.get(), raw_sync);

	if (n < static_cast<int>(sizeof(usb_device_descriptor)))
		throw SysError(std::format("只读到 {} 字节，不够一个设备描述符（{} 字节）",
			n, sizeof(usb_device_descriptor)));

	hexdump("原始字节", std::span<const std::byte>{raw_sync}.first(
		static_cast<size_t>(n)));
	print_device_descriptor(raw_sync);

	std::println("\n== 2) 异步控制 URB 读同一份描述符 ==");
	int m = get_device_descriptor_urb(fd.get(), raw_urb);

	if (m < static_cast<int>(sizeof(usb_device_descriptor)))
		throw SysError(std::format("异步路径只读到 {} 字节，不够一个设备描述符",
			m));

	auto raw_urb_view = std::span<const std::byte>{raw_urb}.first(
		static_cast<size_t>(m));
	hexdump("原始字节", raw_urb_view);
	print_device_descriptor(raw_urb_view);

	if (m == n && raw_sync == raw_urb)
		std::println("\n两条路径读到的 {} 字节完全一致。", n);
	else
		throw SysError(std::format(
			"两次读取不一致（同步 {} 字节，异步 {} 字节）——不应发生", n, m));

	// 不 claim 任何接口就返回：控制请求发给设备本身，不占接口；
	// UniqueFd 析构时 close，内核顺手清掉一切。
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
