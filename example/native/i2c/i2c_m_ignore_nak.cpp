/*
* i2c_m_ignore_nak.cpp —— I2C_M_IGNORE_NAK：把从机的 NACK 当成 ACK，继续把时钟打完
*
* 【它是什么】
* 正常 I2C 传输里，接收方每个字节后的第 9 个时钟必须给出 ACK（拉低 SDA）或 NACK（放开 SDA）。
* 主机看到 NACK 通常立即中止本次传输并报错。I2C_M_IGNORE_NAK（0x1000）让适配器
* “无视从机的 NACK”：不管从机有没有 ACK，都继续把这条消息剩下的字节时钟打完，
* 并且不把 NACK 当错误上报给用户。
*
* 头文件的原文是“treat NACK from client as ACK”（/usr/include/linux/i2c.h），
* 也就是说它只改变“主机对 NACK 的反应”，并不会让从机真的接收数据。
*
* 【什么时候用】
* 只给不规范的从机用。典型场景：
*	- 器件在忙/未就绪时会对地址或数据回 NACK，但它的协议仍要求主机把整条命令送完，
*	  器件靠“边收边判断”而不是靠 NACK 流控；
*	- 某些写-only 器件/显示模组，NACK 没有意义，主机只想把位流打出去。
* 正常器件不要用：它会把“地址写错、器件没插、总线接触不良”这些错误全部吞掉，
* 让你得到一次“成功”的 ioctl，却没有任何器件收到数据。
*
* 【需要的条件】
*	适配器必须支持 I2C_FUNC_PROTOCOL_MANGLING（0x00000004）。这类标志统称
*	“protocol mangling”（协议篡改），因为它们故意偏离 I2C 规范。用 I2C_FUNCS 查。
*	不支持时底层一般返回 -EOPNOTSUPP（具体由适配器驱动决定，未核对）。
*
* 【同类的 protocol mangling 标志】
*	I2C_M_REV_DIR_ADDR (0x2000)：把地址字节里的 R/W 位取反。
*		给那些方向位定义与标准相反的怪异器件用；正常器件用了会读写颠倒。
*	I2C_M_NO_RD_ACK  (0x0800)：读消息里主机不发第 9 个时钟的 ACK/NACK 位。
*		给那些读数据后不接受 ACK 位的器件用；正常器件用了会多/少一个时钟。
*	I2C_M_STOP       (0x8000)：强制在这条消息后发 STOP，即使后面还有消息
*		（正常多条消息之间是 repeated START）。用于必须用 STOP 分隔命令的器件。
*	I2C_M_NOSTART    (0x4000)：跳过 START 和地址（见 i2c_m_nostart.cpp，它属于
*		另一类 I2C_FUNC_NOSTART 能力，不在 PROTOCOL_MANGLING 里）。
*	这些标志的共同点：只在“适配器明确支持 + 器件确实不规范”时使用，
*	否则得到的是看似成功、实际时序错误的数据。
*
* 【形态】
*	一次 ioctl(I2C_RDWR) 提交 1 条写消息，flags = I2C_M_IGNORE_NAK。
*	线上仍是一个 START + 地址 + 数据 + STOP，只是 NACK 不再导致提前终止。
*	返回值和普通 I2C_RDWR 一样：成功是消息条数（1），失败 -1 + errno。
*	关键区别：成功只说明“时钟打完了”，不说明从机 ACK 过、更不说明它收下了数据。
*
* 【本文件的演示】
*	向从机地址写 4 个字节 {0x00, 0x01, 0x02, 0x03}，消息带 I2C_M_IGNORE_NAK。
*	如果器件在地址或数据阶段 NACK，普通写会返回 -1（ENXIO/EREMOTEIO），
*	而这里 ioctl 仍返回 1；本程序会明确打印“成功 ≠ 器件收到”。
*	（示例数据会真的写进器件，接真实硬件前先确认寄存器和地址！）
*
* 【编译】
*	g++ -std=c++26 -O2 -Wall -Wextra -o i2c_m_ignore_nak i2c_m_ignore_nak.cpp
*	不需要链接额外库。
*
* 【运行】
*	./i2c_m_ignore_nak [/dev/i2c-1] [0x50]
*	无设备时打印打开失败并退出（退出码 1），这是本机的预期结果；
*	有设备且适配器支持 PROTOCOL_MANGLING 时打印 ioctl 返回值和提醒。
*
* 【预期输出（有设备时）】
*	打开 /dev/i2c-1，从机地址 0x50
*	适配器功能位 funcs = 0x0xxxxxxxxx
*	--- 写 4 字节，flags = I2C_M_IGNORE_NAK ---
*	ioctl(I2C_RDWR) 返回 1（= nmsgs 1）
*	注意：返回成功只代表时钟打完了，不代表从机 ACK 过或收下了数据。
*/

#include <array>
#include <cerrno>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <print>
#include <span>
#include <string>
#include <string_view>
#include <system_error>
#include <utility>

#include <fcntl.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <linux/i2c.h>
#include <linux/i2c-dev.h>

namespace {

constexpr const char *kDefaultDev = "/dev/i2c-1";
constexpr std::uint16_t kDefaultAddr = 0x50;

static_assert(I2C_M_IGNORE_NAK == 0x1000);
static_assert(I2C_M_REV_DIR_ADDR == 0x2000);
static_assert(I2C_M_NO_RD_ACK == 0x0800);
static_assert(I2C_M_STOP == 0x8000);
static_assert(I2C_FUNC_PROTOCOL_MANGLING == 0x00000004);

class I2cFd {
public:
	explicit I2cFd(const char *path)
		: fd_{::open(path, O_RDWR | O_CLOEXEC)}
	{
		if (fd_ < 0)
			throw std::system_error{errno, std::system_category(),
				std::string{"打开 I2C 适配器 "} + path};
	}

	~I2cFd()
	{
		if (fd_ >= 0)
			::close(fd_);
	}

	I2cFd(const I2cFd &) = delete;
	I2cFd &operator=(const I2cFd &) = delete;

	I2cFd(I2cFd &&other) noexcept
		: fd_{std::exchange(other.fd_, -1)}
	{
	}

	I2cFd &operator=(I2cFd &&other) noexcept
	{
		if (this != &other) {
			if (fd_ >= 0)
				::close(fd_);
			fd_ = std::exchange(other.fd_, -1);
		}
		return *this;
	}

	[[nodiscard]] int get() const noexcept { return fd_; }

private:
	int fd_;
};

I2cFd open_device(std::string_view path)
{
	std::string p{path};
	try {
		return I2cFd{p.c_str()};
	} catch (const std::system_error &e) {
		std::println(stderr, "{}", e.what());
		std::println(stderr, "提示：需要内核 I2C 控制器 + i2c-dev（ls /dev/i2c-*）；");
		std::println(stderr, "      WSL2 / 普通 PC 没有 I2C 控制器时这是预期结果。");
		std::exit(1);
	}
}

std::uint16_t parse_addr(const char *s)
{
	char *end = nullptr;
	errno = 0;
	const unsigned long v = std::strtoul(s, &end, 0);
	if (errno != 0 || end == s || *end != '\0' || v > 0x7f) {
		std::println(stderr, "从机地址 \"{}\" 非法：需要 0x00～0x7F 的 7 位地址", s);
		std::exit(1);
	}
	return static_cast<std::uint16_t>(v);
}

unsigned long adapter_funcs(const I2cFd &dev)
{
	unsigned long funcs = 0;
	if (::ioctl(dev.get(), I2C_FUNCS, &funcs) < 0)
		throw std::system_error{errno, std::system_category(), "I2C_FUNCS"};
	return funcs;
}

void explain_io_error(const char *op, int e)
{
	std::println(stderr, "{} 失败: {} (errno {})", op, std::strerror(e), e);
	switch (e) {
	case ENXIO:
		std::println(stderr, "  地址阶段 NACK：该地址上没有器件。");
		break;
	case EREMOTEIO:
		std::println(stderr, "  数据阶段 NACK：器件拒绝了某个数据字节。");
		break;
	case EOPNOTSUPP:
		std::println(stderr,
			"  适配器不支持该操作（可能没有 I2C_FUNC_PROTOCOL_MANGLING）。");
		break;
	case EINVAL:
		std::println(stderr, "  参数非法（nmsgs/len/标志组合有问题）。");
		break;
	default:
		break;
	}
}

int rdwr_transfer(const I2cFd &dev, std::span<i2c_msg> msgs, const char *what)
{
	struct i2c_rdwr_ioctl_data xfer {
		.msgs = msgs.data(),
		.nmsgs = static_cast<__u32>(msgs.size()),
	};

	const int ret = ::ioctl(dev.get(), I2C_RDWR, &xfer);
	if (ret < 0) {
		const int e = errno;
		explain_io_error(what, e);
		throw std::system_error{e, std::system_category(), what};
	}
	if (ret != static_cast<int>(msgs.size()))
		std::println(stderr, "{} 只完成了 {}/{} 条消息（异常）", what, ret, msgs.size());
	return ret;
}

} // namespace

int main(int argc, char **argv)
{
	const std::string_view dev_path = (argc > 1) ? argv[1] : kDefaultDev;
	const std::uint16_t addr = (argc > 2) ? parse_addr(argv[2]) : kDefaultAddr;

	I2cFd dev = open_device(dev_path);
	std::println("打开 {}，从机地址 0x{:02X}", dev_path, addr);

	try {
		const unsigned long funcs = adapter_funcs(dev);
		std::println("适配器功能位 funcs = {:#010x}", funcs);

		if (!(funcs & I2C_FUNC_I2C)) {
			std::println(stderr, "该适配器不支持 I2C_FUNC_I2C，无法使用 I2C_RDWR。");
			return 1;
		}
		if (!(funcs & I2C_FUNC_PROTOCOL_MANGLING)) {
			/*
			* 不支持时绝不能“去掉标志照发”：那样 NACK 会让传输提前中止，
			* 得到的结果与 IGNORE_NAK 的语义完全不同。
			*/
			std::println(stderr,
				"该适配器没有 I2C_FUNC_PROTOCOL_MANGLING，"
				"不能使用 I2C_M_IGNORE_NAK。");
			std::println(stderr,
				"这是正常适配器的常态；IGNORE_NAK 只给不规范的从机用。");
			return 1;
		}

		/*
		* ---------- 一条写消息，带 I2C_M_IGNORE_NAK ----------
		*
		* 线上正常是：S 0x50+W 00 01 02 03 P。
		* 若从机在地址字节或某个数据字节后回 NACK：
		*	- 不加标志：适配器中止，ioctl 返回 -1，errno 多为 ENXIO/EREMOTEIO；
		*	- 加了标志：适配器继续打完剩余时钟，ioctl 返回 1（成功）。
		* 但“成功”不代表从机 ACK 过，也不代表字节被写进器件。
		*
		* 示例数据 {0x00,0x01,0x02,0x03} 会真的写到 0x50 指向的器件上，
		* 接真实硬件前请按器件手册换成无害的寄存器/数据。
		*/
		std::println("--- 写 4 字节，flags = I2C_M_IGNORE_NAK ---");

		std::array<std::uint8_t, 4> payload{0x00, 0x01, 0x02, 0x03};
		std::array<i2c_msg, 1> msgs{};

		msgs[0].addr = addr;
		msgs[0].flags = I2C_M_IGNORE_NAK;	/* 核心：忽略从机 NACK */
		msgs[0].len = static_cast<__u16>(payload.size());
		msgs[0].buf = payload.data();

		const int n = rdwr_transfer(dev, msgs, "ioctl(I2C_RDWR) IGNORE_NAK");
		std::println("ioctl(I2C_RDWR) 返回 {}（= nmsgs {}）", n, msgs.size());
		std::println("注意：返回成功只代表时钟打完了，不代表从机 ACK 过或收下了数据。");

		/*
		* 同类标志的用法提示（本文件不逐一演示，避免对真实器件乱发时序）：
		*
		*	flags = I2C_M_RD | I2C_M_NO_RD_ACK
		*		读消息，但主机不发第 9 个时钟的 ACK/NACK 位。
		*
		*	flags = 0 | I2C_M_REV_DIR_ADDR
		*		这条消息本来要写，但地址字节的 R/W 位取反，线上变成读；
		*		给方向位反着定义的器件用。千万不要给正常器件用。
		*
		*	flags = 0 | I2C_M_STOP
		*		这条消息结束后强制 STOP，即使 I2C_RDWR 里后面还有消息。
		*
		* 这些位可以组合，但组合出来的时序要对着器件手册逐项确认；
		* 适配器不支持其中任何一位时，最安全的做法是报错退出，而不是删掉标志重试。
		*/

		/*
		* 常见坑：
		*	1. 用 IGNORE_NAK “修好” ENXIO → 实际是地址错了/器件没插，数据全丢；
		*	2. 以为返回 1 就是写入成功 → 只是时钟跑完；
		*	3. 在正常器件上使用 → 真实故障被静默吞掉，难排查；
		*	4. 忽略 I2C_FUNC_PROTOCOL_MANGLING 检查 → 不同适配器行为不一致；
		*	5. 把 I2C_M_IGNORE_NAK 当成“自动重试” → 它不重试，只是不因 NACK 中止。
		*/
	} catch (const std::system_error &e) {
		std::println(stderr, "I2C_M_IGNORE_NAK 操作失败: {}", e.what());
		return 1;
	}

	return 0;
}
