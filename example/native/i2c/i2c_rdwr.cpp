/*
* i2c_rdwr.cpp —— i2c-dev 的主力接口：ioctl(I2C_RDWR) 一次提交多条 i2c_msg
*
* 【它解决什么问题】
* i2c_slave_rw.cpp 里的 read()/write() 每次只能做一段固定方向的传输，一次调用一个
* START…STOP。想“写寄存器地址后立刻读数据且中间不发 STOP”，或者“先发命令、再发 16 位
* 地址、再读”，就必须把多段拼在一次调用里。I2C_RDWR 就是干这个的：
*	1 次 ioctl → N 条 i2c_msg → 线上 START 段1 Sr 段2 Sr … 段N STOP
*	N 最多 I2C_RDWR_IOCTL_MAX_MSGS = 42（/usr/include/linux/i2c-dev.h）。
* 相邻两条消息之间由内核发 repeated START（Sr），只有最后一条后面发 STOP。
* 因此整个调用在总线层面是原子的：内核持有适配器锁，别的 I2C 传输插不进来。
*
* 【形态】
*	一次 ioctl 交进去：1 个 struct i2c_rdwr_ioctl_data（里面是 i2c_msg 数组指针 + 条数）。
*	每条 i2c_msg 自带 addr（从机地址）、flags（方向/特殊行为）、len、buf。
*	线上单元数 = 消息条数，每条消息一个（重复）START + 地址字节 + 数据 + ACK/NACK。
*	提交一次、同步阻塞完成一次；返回时读方向的缓冲区已经被内核拷回用户空间。
*	缓冲由用户提供；i2c-dev 对每条消息 memdup_user() 拷进内核并标 I2C_M_DMA_SAFE，
*	所以用户缓冲不需要 DMA 对齐，代价是每条消息一次拷贝。
*	限制（drivers/i2c/i2c-dev.c，已核对）：
*	  - nmsgs == 0 或 msgs == NULL → EINVAL；
*	  - nmsgs > 42 → EINVAL；
*	  - 任意一条 len > 8192 → EINVAL；
*	  - 适配器不支持 I2C_FUNC_I2C → EOPNOTSUPP。
*	注意：I2C_RDWR 不检查每条 msg.addr 是否 ≤ 0x7F（i2c-dev.c 里没有这个检查），
*	地址写错不会在 ioctl 入口被拦下，只会表现为 NACK 或访错器件。
*
* 【返回值与 errno】
*	成功：返回实际完成的消息条数，正常等于 nmsgs。
*	失败：返回 -1，errno 说明原因（语义据内核 Documentation/i2c/fault-codes.rst，未核对原文）：
*	  ENXIO      —— 地址阶段 NACK：该地址上没有器件；
*	  EREMOTEIO  —— 数据阶段 NACK：器件 ACK 了地址，但某个数据字节被拒绝；
*	  EINVAL     —— 参数非法（0/超过 42 条、单条 >8192、RECV_LEN 约定不满足等）；
*	  EOPNOTSUPP —— 适配器不支持 I2C_FUNC_I2C（或某条消息要求的特殊标志）；
*	  ETIMEDOUT  —— 总线超时（SDA/SCL 被拉死）；
*	  EIO        —— 其它总线错误（仲裁丢失、干扰）。
*	坑：i2c-dev 只有在整个调用成功（返回值 ≥ 0）时才把读方向的缓冲拷回用户空间；
*	失败时你传进去的读缓冲内容未定义，不要拿它当“读到了一部分”。
*
* 【什么时候用】
*	所有寄存器式器件的“写地址再读”、需要多段拼一次传输的时序、以及想一次调用完成
*	多颗从机的连续操作（每条消息可以写不同的 addr，中间同样用 Sr 连接）。
*	只需要 SMBus 子集（读字节/字/块）时可以用更简单的 ioctl(I2C_SMBUS)；
*	要任意时序就用 I2C_RDWR。
*
* 【需要的条件】
*	Linux + i2c-dev + 真实 I2C 控制器和从机。本机（WSL2）没有 I2C 设备，
*	程序会在 open 处优雅报错退出，这是预期行为。
*
* 【编译】
*	g++ -std=c++26 -O2 -Wall -Wextra -o i2c_rdwr i2c_rdwr.cpp
*	不需要链接额外库。
*
* 【运行】
*	./i2c_rdwr [/dev/i2c-1] [0x50]
*	有设备时：演示 A “写寄存器地址 + 读 16 字节”（2 条消息），
*	演示 B “写命令 + 写 16 位地址 + 读 8 字节”（3 条消息），并打印返回值和数据。
*	无设备时：打印“打开 I2C 适配器 … 失败: …”，退出码 1。
*
* 【预期输出（有设备时，数据随器件而定）】
*	打开 /dev/i2c-1，从机地址 0x50
*	适配器功能位 funcs = 0x0xxxxxxxxx
*	--- A) 写寄存器地址 0x10，再用 repeated START 读 16 字节（2 条 msg）---
*	ioctl(I2C_RDWR) 返回 2（= nmsgs）
*	数据 (16 字节): 00 01 ...
*	--- B) 写命令 0x0B、写 16 位地址 0x0020、读 8 字节（3 条 msg）---
*	ioctl(I2C_RDWR) 返回 3（= nmsgs）
*	数据 (8 字节): ...
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

/* 头文件里的硬限制，编译期确认一下，防止记错。 */
static_assert(I2C_RDWR_IOCTL_MAX_MSGS == 42);
static_assert(I2C_M_RD == 0x0001);

/*
* RAII 持有 /dev/i2c-N 的 fd。i2c-dev 的一个 fd 对应一个内核 i2c_client，
* 但 I2C_RDWR 使用的是每条 msg 自己的 addr，client->addr 在这里不参与。
*/
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

/* errno 到中文解释，语义参考 Documentation/i2c/fault-codes.rst（未能下载原文，未核对）。 */
void explain_io_error(const char *op, int e)
{
	std::println(stderr, "{} 失败: {} (errno {})", op, std::strerror(e), e);
	switch (e) {
	case ENXIO:
		std::println(stderr, "  地址阶段 NACK：该地址上没有器件，检查地址/供电/接线。");
		break;
	case EREMOTEIO:
		std::println(stderr,
			"  数据阶段 NACK：器件 ACK 了地址但拒绝了某个数据字节"
			"（读越界、写保护、器件忙）。");
		break;
	case EINVAL:
		std::println(stderr,
			"  参数非法：nmsgs 为 0 或 >42、某条 len >8192、标志组合不合法。");
		break;
	case EOPNOTSUPP:
		std::println(stderr,
			"  适配器不支持 I2C_FUNC_I2C，或某条消息要求的特殊标志不受支持。");
		break;
	case ETIMEDOUT:
		std::println(stderr, "  总线超时：SDA/SCL 可能被拉死。");
		break;
	case EIO:
		std::println(stderr, "  总线错误：检查接线、上拉、地址冲突。");
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

/*
* I2C_RDWR 的薄封装：把 msg 数组和条数塞进 i2c_rdwr_ioctl_data 后提交。
* std::span 不拥有消息也不拥有缓冲，只是“数组 + 长度”的视图；生命周期由调用方保证。
* 成功时返回完成的消息条数；失败抛 std::system_error（errno 装进 error_code）。
*/
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
	if (ret != static_cast<int>(msgs.size())) {
		/*
		* 正常情况 i2c_transfer() 要么返回 nmsgs，要么返回负 errno；
		* 出现 0 < ret < nmsgs 说明驱动只完成了部分消息，属于异常，明确报出来。
		*/
		std::println(stderr, "{} 只完成了 {}/{} 条消息（异常）", what, ret, msgs.size());
	}
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
		/*
		* 先确认适配器会“裸 I2C”。I2C_RDWR 走 i2c_transfer()，
		* 不支持 I2C_FUNC_I2C 的适配器（有些纯 SMBus 控制器）会直接 EOPNOTSUPP。
		*/
		const unsigned long funcs = adapter_funcs(dev);
		std::println("适配器功能位 funcs = {:#010x}", funcs);
		if (!(funcs & I2C_FUNC_I2C)) {
			std::println(stderr, "该适配器不支持 I2C_FUNC_I2C，I2C_RDWR 无法使用。");
			return 1;
		}

		/*
		* ---------- 演示 A：写寄存器地址 + 读 16 字节（2 条 msg） ----------
		*
		* 线上：
		*	S  0x50+W  0x10  Sr  0x50+R  data0 … data15  P
		*	（S = START，Sr = repeated START，P = STOP）
		* 第 1 条：写方向，buf 里放 1 字节寄存器地址。
		* 第 2 条：读方向（flags 必须带 I2C_M_RD），buf 收 16 字节。
		*
		* 两条消息的 addr 相同；内核在第 1 条之后不会发 STOP，而是发 Sr，
		* 器件因此不会把内部寄存器指针复位 —— 这是“写地址再读”的正确姿势。
		*
		* 注意 I2C_RDWR 不会自动调用 I2C_SLAVE：每条 msg 自带 addr，
		* 也就可以在一次调用里访问不同从机（addr 不同即可）。
		*
		* 字段说明：
		*	addr  —— 7 位从机地址，直接写 0x50，不要左移；
		*	flags —— I2C_M_RD (0x0001) 表示读，0 表示写；其它标志见后续示例；
		*	len   —— 本条消息的数据字节数，≤ 8192；
		*	buf   —— 用户缓冲区；读方向必须可写，写方向只读即可。
		*/
		std::println("--- A) 写寄存器地址 0x10，再用 repeated START 读 16 字节（2 条 msg）---");

		std::array<std::uint8_t, 1> reg{0x10};
		std::array<std::uint8_t, 16> data{};
		std::array<i2c_msg, 2> msgs{};

		msgs[0].addr = addr;
		msgs[0].flags = 0;			/* 0 = 写方向 */
		msgs[0].len = static_cast<__u16>(reg.size());
		msgs[0].buf = reg.data();

		msgs[1].addr = addr;
		msgs[1].flags = I2C_M_RD;		/* 0x0001，头文件保证就是这一位 */
		msgs[1].len = static_cast<__u16>(data.size());
		msgs[1].buf = data.data();

		const int an = rdwr_transfer(dev, msgs, "ioctl(I2C_RDWR) A");
		std::println("ioctl(I2C_RDWR) 返回 {}（= nmsgs {}）", an, msgs.size());
		dump("数据", data);

		/*
		* ---------- 演示 B：写命令 + 写 16 位地址 + 读 8 字节（3 条 msg） ----------
		*
		* 线上：
		*	S 0x50+W 0x0B  Sr 0x50+W 0x00 0x20  Sr 0x50+R data0…data7  P
		*
		* 典型于 EEPROM 的随机读、以及“先发命令码，再发 16 位地址，再读结果”的器件。
		* 如果拆成三次 ioctl/read/write，中间会各有一次 STOP，时序就不是器件要的了。
		* 三条消息的 addr 都是 0x50；如果器件协议允许，也可以换 addr 访问别的从机。
		*
		* 返回 3 表示三条消息都完成（不是“读了 3 字节”）。
		* 若中途 NACK，返回 -1，读缓冲 data2 的内容不可用。
		*/
		std::println("--- B) 写命令 0x0B、写 16 位地址 0x0020、读 8 字节（3 条 msg）---");

		std::array<std::uint8_t, 1> cmd{0x0B};
		std::array<std::uint8_t, 2> word_addr{0x00, 0x20};
		std::array<std::uint8_t, 8> out{};
		std::array<i2c_msg, 3> msgs3{};

		msgs3[0].addr = addr;
		msgs3[0].flags = 0;
		msgs3[0].len = static_cast<__u16>(cmd.size());
		msgs3[0].buf = cmd.data();

		msgs3[1].addr = addr;
		msgs3[1].flags = 0;
		msgs3[1].len = static_cast<__u16>(word_addr.size());
		msgs3[1].buf = word_addr.data();

		msgs3[2].addr = addr;
		msgs3[2].flags = I2C_M_RD;
		msgs3[2].len = static_cast<__u16>(out.size());
		msgs3[2].buf = out.data();

		const int bn = rdwr_transfer(dev, msgs3, "ioctl(I2C_RDWR) B");
		std::println("ioctl(I2C_RDWR) 返回 {}（= nmsgs {}）", bn, msgs3.size());
		dump("数据", out);

		/*
		* 常见坑小结（本文件代码已覆盖的部分）：
		*	1. 读消息忘了 I2C_M_RD → 内核当成写，把读缓冲的内容发出去。
		*	2. 以为返回的是字节数 → 返回的是消息条数。
		*	3. 失败后继续用读缓冲 → 失败时 i2c-dev 不拷贝读数据。
		*	4. nmsgs 超过 42 或某条 len 超过 8192 → EINVAL，是硬限制。
		*	5. 把 10 位地址当 7 位写 → I2C_RDWR 在入口不检查地址范围，会静默发错。
		*/
	} catch (const std::system_error &e) {
		std::println(stderr, "I2C_RDWR 操作失败: {}", e.what());
		return 1;
	}

	return 0;
}
