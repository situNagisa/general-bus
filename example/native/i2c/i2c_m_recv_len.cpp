/*
* i2c_m_recv_len.cpp —— I2C_M_RECV_LEN：由从机返回的第一个字节决定后面还要读多少
*
* 【它是什么】
* 普通读消息的 len 在发出去之前就定死了：主机说读 16 字节，就从机吐 16 字节。
* 但 SMBus 的“块读”（block read）是变长的：主机先发一个命令字节，从机回的第一个
* 字节 N 表示“后面还有 N 个数据字节”，N 最大 I2C_SMBUS_BLOCK_MAX = 32。
*
* I2C_M_RECV_LEN（0x0400，/usr/include/linux/i2c.h）就是把这件事做进一条 i2c_msg：
* 主机只知道“最多可能有多少字节”，先读第一个字节得到 N，再继续读 N 个字节，
* 最后在数据末尾补一个 NACK + STOP。整个过程一条消息、一次 START…STOP。
*
* 【buf / len 的约定】（以本机 /usr/include/linux/i2c.h 为准）
* 头文件对 I2C_M_RECV_LEN 消息的 @len 是这么规定的：
*	“For read transactions where %I2C_M_RECV_LEN is set, the caller guarantees that
*	 this buffer can hold up to %I2C_SMBUS_BLOCK_MAX bytes in addition to the initial
*	 length byte sent by the slave (plus, if used, the SMBus PEC); and this value will
*	 be incremented by the number of block data bytes received.”
* 翻成操作步骤：
*	1. flags 必须同时有 I2C_M_RD 和 I2C_M_RECV_LEN。
*	2. buf[0] 在调用前由用户填好，含义是“除了块数据之外，我预留了几个额外字节”，
*	   至少要 1（放从机发来的长度字节本身）；启用 PEC 时填 2（长度字节 + PEC）。
*	3. msg.len 在调用前是“缓冲区容量”的意思，必须 >= buf[0] + I2C_SMBUS_BLOCK_MAX。
*	   也就是说按最坏情况（N = 32）留够空间。稳妥做法是直接给
*	   I2C_SMBUS_BLOCK_MAX + 2 = 34 字节（长度 1 + 数据 32 + PEC 1）。
*	4. 内核（i2c-dev）检查通过后，把 msg.len 改成 buf[0]（通常就是 1），
*	   适配器据此先读第一个字节；读到 N 之后再把 len 加上 N。
*	5. 返回用户空间时，buf[0] 已变成从机发来的 N，buf[1..N] 是块数据
*	   （若启用 PEC，最后还有一个 PEC 字节）；内核只把实际收到的字节拷回来。
*
* 【i2c-dev 的具体检查】（核对的是内核 master 分支 drivers/i2c/i2c-dev.c，
* 与本机 6.18 的实际源码可能有细微差异，未核对本机版本）
*	if (msgs[i].flags & I2C_M_RECV_LEN) {
*		if (!(msgs[i].flags & I2C_M_RD) ||
*		    msgs[i].len < 1 || msgs[i].buf[0] < 1 ||
*		    msgs[i].len < msgs[i].buf[0] + I2C_SMBUS_BLOCK_MAX)
*			return -EINVAL;
*		msgs[i].len = msgs[i].buf[0];
*	}
* 所以 buf[0] 忘了预填、或 len 只给了 33 但 buf[0] 填了 2，都会 EINVAL。
*
* 【一个容易踩的坑：ioctl 返回后，用户态 msg.len 不会更新】
* i2c-dev 把整个 i2c_msg 数组 memdup 到内核里操作，只把数据缓冲 copy_to_user 回来，
* 不把修改后的 msg 数组拷回用户空间。因此调用后：
*	- 你的 msgs[i].len 还是你填的缓冲区容量（例如 34），不要拿它当“读到了多少”；
*	- 真正读到的长度看 buf[0]（N），总字节数是 N + 1（再加 PEC 则是 N + 2）；
*	- buf 里 N+1 个字节是可信的，后面的内容是调用前的旧值/未初始化值。
* （内核驱动里直接调用 i2c_transfer() 时 msg.len 会被更新；用户态经 i2c-dev 看不到。）
*
* 【需要的内核/硬件条件】
*	适配器要支持 I2C_FUNC_SMBUS_READ_BLOCK_DATA（头文件对 I2C_M_RECV_LEN 的要求），
*	并且支持 I2C_FUNC_I2C 才能走 I2C_RDWR。从机要真的是 SMBus 块读协议：
*	先回 1 个长度字节，再回 N 个数据字节。
*	本机（WSL2）没有 I2C 设备，程序在 open 处报错退出，这是预期行为。
*
* 【形态】
*	一次 ioctl(I2C_RDWR) 提交 2 条消息：第 1 条写命令/寄存器地址，第 2 条是
*	I2C_M_RD | I2C_M_RECV_LEN 的变长读。线上：
*	  S addr+W cmd  Sr addr+R N d0…d(N-1)  P
*	提交一次、同步完成一次；缓冲由用户提供（至少 34 字节），内核按条拷贝。
*	返回值和普通 I2C_RDWR 一样：成功是消息条数（2），失败 -1 + errno。
*
* 【编译】
*	g++ -std=c++26 -O2 -Wall -Wextra -o i2c_m_recv_len i2c_m_recv_len.cpp
*	不需要链接额外库。
*
* 【运行】
*	./i2c_m_recv_len [/dev/i2c-1] [0x50]
*	无设备时打印打开失败并退出（退出码 1）。
*	有设备时打印块长度 N 和 N 个数据字节。
*
* 【预期输出（有设备时，N 和数据随器件而定，N ≤ 32）】
*	打开 /dev/i2c-1，从机地址 0x50
*	适配器功能位 funcs = 0x0xxxxxxxxx
*	--- 写命令 0x00，再用 I2C_M_RECV_LEN 读一个 SMBus 块 ---
*	从机报告块长度 N = 8
*	块数据 (8 字节): 11 22 33 44 55 66 77 88
*	（缓冲区容量 34 字节，用户态 msg.len 仍是 34，实际收到 9 字节）
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
constexpr std::size_t kBlockCapacity = I2C_SMBUS_BLOCK_MAX + 2;	/* 34：长度+数据+PEC */

static_assert(I2C_M_RECV_LEN == 0x0400);
static_assert(I2C_SMBUS_BLOCK_MAX == 32);

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
		std::println(stderr, "  数据阶段 NACK：器件在数据阶段拒绝继续。");
		break;
	case EINVAL:
		std::println(stderr,
			"  参数非法：RECV_LEN 没带 I2C_M_RD、len<1、buf[0]<1，"
			"或 len < buf[0]+32。");
		break;
	case EOPNOTSUPP:
		std::println(stderr, "  适配器不支持 I2C_FUNC_I2C 或块读（SMBUS_READ_BLOCK_DATA）。");
		break;
	case EPROTO:
		std::println(stderr, "  协议错误：从机返回的块长度可能为 0 或超过 32（未核对具体驱动）。");
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
		/*
		* 头文件写明 I2C_M_RECV_LEN 只在适配器支持
		* I2C_FUNC_SMBUS_READ_BLOCK_DATA 时使用。不支持还硬发，时序没有保证。
		*/
		if (!(funcs & I2C_FUNC_SMBUS_READ_BLOCK_DATA)) {
			std::println(stderr,
				"该适配器没有 I2C_FUNC_SMBUS_READ_BLOCK_DATA，不能使用 I2C_M_RECV_LEN。");
			return 1;
		}

		/*
		* ---------- 第 1 条：写命令/寄存器地址 ----------
		*
		* SMBus 块读的完整线上时序是：写命令字节 → Sr → 读块。
		* 这里的 0x00 是示例命令，真实器件查手册（例如“读块”命令码）。
		*/
		std::array<std::uint8_t, 1> cmd{0x00};

		/*
		* ---------- 第 2 条：I2C_M_RECV_LEN 变长读 ----------
		*
		* 缓冲区必须是 I2C_SMBUS_BLOCK_MAX + 2 = 34 字节：
		*	buf[0]  长度字节（调用前填 1，表示至少给它留 1 个额外字节）
		*	buf[1..32] 最多 32 字节块数据
		*	buf[33] 若启用 PEC 时的 PEC 字节（本示例未启用，留作余量）
		*
		* block[0] = 1 是 i2c-dev 的硬性要求：必须 >= 1。
		* msgs[1].len = block.size() = 34，满足 len >= buf[0] + 32。
		* 内核随后会把（内核副本的）len 改成 1，适配器先读长度字节 N，
		* 再读 N 个字节，最后把 len 更新为 1 + N。
		* 注意：这些修改发生在内核副本里，用户态 msgs[1].len 不会变。
		*/
		std::array<std::uint8_t, kBlockCapacity> block{};
		block[0] = 1;		/* 预填：除块数据外预留 1 个长度字节；启用 PEC 时填 2 */

		std::array<i2c_msg, 2> msgs{};

		msgs[0].addr = addr;
		msgs[0].flags = 0;					/* 写命令 */
		msgs[0].len = static_cast<__u16>(cmd.size());
		msgs[0].buf = cmd.data();

		msgs[1].addr = addr;
		msgs[1].flags = I2C_M_RD | I2C_M_RECV_LEN;		/* 读 + 变长 */
		msgs[1].len = static_cast<__u16>(block.size());		/* 容量，不是预期长度 */
		msgs[1].buf = block.data();

		std::println("--- 写命令 0x00，再用 I2C_M_RECV_LEN 读一个 SMBus 块 ---");
		const int n = rdwr_transfer(dev, msgs, "ioctl(I2C_RDWR) RECV_LEN");
		std::println("ioctl(I2C_RDWR) 返回 {}（= nmsgs {}）", n, msgs.size());

		/*
		* 读到的第一个字节就是块长度 N。合法范围 1..32（SMBus 规范）。
		* 从机返回 0 或 >32 时，有些适配器/内核会返回 -EPROTO，有些可能截断，
		* 具体行为依驱动而定，未核对。
		*/
		const std::size_t block_len = block[0];
		if (block_len == 0 || block_len > I2C_SMBUS_BLOCK_MAX) {
			std::println(stderr, "从机报告的块长度 {} 不在 1..{} 内，数据不可信",
				block_len, I2C_SMBUS_BLOCK_MAX);
			return 1;
		}

		std::println("从机报告块长度 N = {}", block_len);
		/* 实际有效字节 = 1 个长度字节 + N 个数据字节；用户态 msg.len 仍是 34。 */
		dump("块数据", std::span<const std::uint8_t>{block}.subspan(1, block_len));
		std::println("缓冲区容量 {} 字节，用户态 msg.len 仍是 {}，实际收到 {} 字节",
			block.size(), msgs[1].len, block_len + 1);

		/*
		* 常见坑：
		*	1. 忘了设置 buf[0]（为 0）→ i2c-dev 直接 EINVAL；
		*	2. len 只按“预期长度”给（例如 8）→ EINVAL，必须按最坏情况 33/34 给；
		*	3. 用返回后的 msg.len 判断读到多少 → 用户态看不到内核的更新，应看 buf[0]；
		*	4. 把 buf[1..] 全部打印 → 超过 N 的部分是旧数据/未初始化；
		*	5. 在适配器不支持块读时硬用 → 时序无保证。
		*/
	} catch (const std::system_error &e) {
		std::println(stderr, "I2C_M_RECV_LEN 操作失败: {}", e.what());
		return 1;
	}

	return 0;
}
