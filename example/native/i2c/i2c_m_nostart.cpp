/*
* i2c_m_nostart.cpp —— 用 I2C_M_NOSTART 把多条消息拼成线上的一段连续写
*
* 【它是什么】
* I2C_M_NOSTART 是 struct i2c_msg 的一个 flags 位（0x4000，见 /usr/include/linux/i2c.h）。
* 打在“第二条及以后”的消息上时，它告诉适配器驱动：
*	这条消息不要发 START，也不要发从机地址，直接接着上一条消息的数据继续打时钟。
* 于是多条 i2c_msg 在用户空间看是分开的缓冲，在线上却是“一次寻址 + 一长串数据”：
*	S addr+W d0 d1 … dk P
* 中间既没有 repeated START，也没有第二次地址字节。
*
* 【解决什么问题】
* 1. 有些器件的命令格式要求“地址/命令之后的数据必须紧跟着时钟流”，中间插入一个
*    repeated START（更别说 STOP）会被当成新事务，命令作废。NOSTART 可以满足它。
* 2. 用户态的数据天然分散在几块内存里（例如寄存器地址在栈上、负载在另一个大缓冲里），
*    用 NOSTART 就不需要先把它们 memcpy 拼成一块连续内存再写。
*    但注意：i2c-dev 仍会对每条消息各自 memdup_user() 一次，所以省掉的只是“用户态拼接
*    拷贝”，不是内核拷贝，也不是零拷贝。
*
* 【形态】
*	一次 ioctl(I2C_RDWR) 提交 N 条消息，其中第 2 条起可带 I2C_M_NOSTART。
*	线上单元：1 个 START + 1 个地址字节 + 所有消息的数据字节（方向保持写）+ 1 个 STOP。
*	返回值和普通 I2C_RDWR 一样：成功返回消息条数，失败返回 -1 + errno。
*
* 【需要的条件】
*	适配器必须支持 I2C_FUNC_NOSTART（用 ioctl(I2C_FUNCS) 查询）。
*	以驱动实际置位为准：不少 SoC 控制器支持，很多 USB-I2C 桥、SMBus-only 控制器不支持
*	（具体哪些驱动置 I2C_FUNC_NOSTART 未核对）；不支持时底层通常返回 -EOPNOTSUPP。
*	I2C_M_NOSTART 只能出现在“前面还有消息”的位置；对第一条消息用它意味着整段没有
*	START 和地址，线上是非法的，不要这么做。
*
* 【本文件的演示】
*	两块内存：
*	  reg[]     = {0x00, 0x10}            —— 16 位寄存器地址
*	  payload[] = {0x01, …, 0x08}         —— 要写入的 8 字节数据
*	用 2 条 msg（第二条带 I2C_M_NOSTART）拼成线上一次写：
*	  S 0x50+W 00 10 01 02 03 04 05 06 07 08 P
*	如果适配器不支持 NOSTART，程序打印原因后退出，不会偷偷改成别的时序。
*
* 【编译】
*	g++ -std=c++26 -O2 -Wall -Wextra -o i2c_m_nostart i2c_m_nostart.cpp
*	不需要链接额外库。
*
* 【运行】
*	./i2c_m_nostart [/dev/i2c-1] [0x50]
*	无设备时打印打开失败并退出（退出码 1），这是本机的预期结果；
*	有设备且适配器支持 NOSTART 时打印 ioctl 返回值（2）和实际写入的线上字节数。
*
* 【预期输出（有设备时）】
*	打开 /dev/i2c-1，从机地址 0x50
*	适配器功能位 funcs = 0x0xxxxxxxxx
*	--- 2 条 msg：第 1 条写寄存器地址，第 2 条 NOSTART 直接跟 8 字节数据 ---
*	ioctl(I2C_RDWR) 返回 2（= nmsgs），线上共 1 次寻址 + 10 个数据字节
*	写寄存器 0x0010 的数据 (8 字节): 01 02 03 04 05 06 07 08
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

static_assert(I2C_M_NOSTART == 0x4000);
static_assert(I2C_FUNC_NOSTART == 0x00000010);

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
			"  适配器不支持这个操作（常见：不支持 I2C_M_NOSTART / 不是 I2C_FUNC_I2C）。");
		break;
	case EINVAL:
		std::println(stderr, "  参数非法（nmsgs/len/标志组合有问题）。");
		break;
	default:
		break;
	}
}

void dump(std::string_view tag, std::span<const std::uint8_t> bytes)
{
	std::print("{} ({} 字节):", tag, bytes.size());
	for (const std::uint8_t b : bytes)
		std::print(" {:02X}", b);
	std::println();
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
		if (!(funcs & I2C_FUNC_NOSTART)) {
			/*
			* 不支持 NOSTART 时不要硬发：i2c_msg 里的标志会被适配器驱动忽略或直接
			* 报 EOPNOTSUPP，前者更危险 —— 你以为是一次连续写，实际中间多了一个 SR。
			* 可行的替代方案是把寄存器和数据 memcpy 成一个缓冲，用 1 条普通写消息发出去，
			* 线上时序完全一样，只是多一次用户态拷贝。
			*/
			std::println(stderr,
				"该适配器没有 I2C_FUNC_NOSTART，不能演示 I2C_M_NOSTART。");
			std::println(stderr,
				"替代做法：把寄存器地址和数据拷进同一个缓冲，用一条普通写消息发送。");
			return 1;
		}

		/*
		* ---------- 两条消息，第二条 NOSTART ----------
		*
		* 第 1 条：普通写，线上 S 0x50+W 0x00 0x10
		* 第 2 条：flags 带 I2C_M_NOSTART，线上不再发 START/地址，直接接着发 8 字节数据。
		* 最后一条消息结束后内核发 STOP。
		*
		* 字段：addr 仍要填（结构体要求），但 NOSTART 消息的 addr 不会出现在线上，
		* 从机是由第 1 条的地址选中的。把第 2 条的 addr 填成别的值没有意义。
		*/
		std::println("--- 2 条 msg：第 1 条写寄存器地址，第 2 条 NOSTART 直接跟 8 字节数据 ---");

		std::array<std::uint8_t, 2> reg{0x00, 0x10};		/* 16 位寄存器地址 */
		std::array<std::uint8_t, 8> payload{0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};
		std::array<i2c_msg, 2> msgs{};

		msgs[0].addr = addr;
		msgs[0].flags = 0;					/* 普通写，发 START + 地址 */
		msgs[0].len = static_cast<__u16>(reg.size());
		msgs[0].buf = reg.data();

		msgs[1].addr = addr;
		msgs[1].flags = I2C_M_NOSTART;				/* 核心：跳过 START 和地址 */
		msgs[1].len = static_cast<__u16>(payload.size());
		msgs[1].buf = payload.data();

		const int n = rdwr_transfer(dev, msgs, "ioctl(I2C_RDWR) NOSTART");
		std::println("ioctl(I2C_RDWR) 返回 {}（= nmsgs {}），线上共 1 次寻址 + {} 个数据字节",
			n, msgs.size(), reg.size() + payload.size());
		dump("写寄存器 0x0010 的数据", payload);

		/*
		* 对比：下面这种写法线上时序完全相同，但需要额外一次 memcpy 和一块 10 字节缓冲：
		*	std::array<std::uint8_t, 10> joined{};
		*	std::copy(reg.begin(), reg.end(), joined.begin());
		*	std::copy(payload.begin(), payload.end(), joined.begin() + reg.size());
		*	i2c_msg m{ .addr = addr, .flags = 0, .len = joined.size(), .buf = joined.data() };
		* 什么时候值得用 NOSTART：数据块很大、不想复制，或者上层本来就是分散的缓冲。
		* 什么时候不值得：适配器不支持、数据很小（拼接一次更省事）。
		*/

		/*
		* 常见坑：
		*	1. 把 I2C_M_NOSTART 用在第一条消息上 → 整段没有 START/地址，线上非法。
		*	2. 以为第二条的 addr 会生效 → 不会，线上没有地址阶段。
		*	3. 方向：NOSTART 的消息实际沿用了上一次传输的方向（写）；不要给它加 I2C_M_RD。
		*	4. 适配器不支持时，标志可能被静默忽略（多一个 SR），所以先查 I2C_FUNC_NOSTART。
		*	5. NOSTART 只是“不发 START/地址”，不会帮你合并内核拷贝，i2c-dev 仍按条复制。
		*/
	} catch (const std::system_error &e) {
		std::println(stderr, "I2C_M_NOSTART 操作失败: {}", e.what());
		return 1;
	}

	return 0;
}
