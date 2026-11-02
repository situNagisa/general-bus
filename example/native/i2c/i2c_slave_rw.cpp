/*
* i2c_slave_rw.cpp —— i2c-dev 最基础的用法：ioctl(I2C_SLAVE) 选从机，再用 read()/write() 做单段传输
*
* 【I2C 是什么】
* I2C 是一条两线、半双工、多从机的同步串行总线：
*	SCL —— 时钟。始终由当前的主机（master，这里就是你的程序）产生，从机只能“拉伸”它来要求等待。
*	SDA —— 数据。主机的地址、命令、数据，以及从机的数据、ACK 全走这一根线。
* 两根线都是开漏（open-drain）加外部上拉电阻，任何一方只能把它拉低，不能主动驱动高电平；
* 总线空闲时两根线都被上拉为高。这也是“线与”仲裁能工作的原因。
*
* 【一次 I2C 事务的线上单元】
*	1. START：SCL 为高时 SDA 由高变低，表示“总线现在归我”。
*	2. 地址字节：7 位从机地址 + 1 位方向位（0 = 主机写，1 = 主机读），共 8 位，高位在前。
*	3. ACK/NACK：第 9 个时钟由接收方拉低 SDA 表示 ACK（收到了）；放开 SDA（高）表示 NACK。
*	   地址字节的 ACK 表示“这个地址上有器件”；写数据时 ACK 表示“这个字节我收下了”；
*	   主机读最后一个字节后故意回 NACK，告诉从机“别再发了”。
*	4. 数据字节：每 8 位后面跟 1 位 ACK/NACK，可以连续很多个。
*	5. STOP：SCL 为高时 SDA 由低变高，表示“本次事务结束，总线释放”。
*	6. repeated START（Sr）：不先发 STOP，直接再发一个 START。
*	   它的用途是“换方向但不让出总线”：典型就是“先写寄存器地址、再读数据”，
*	   写和读之间用 Sr 连接，别的 master 无法插进来，从机也不会因为 STOP 复位内部状态。
*
* 7 位地址的有效范围是 0x00～0x7F：0x00 是广播（general call），0x01～0x07 和 0x78～0x7F
* 是保留/特殊用途，普通器件一般落在 0x08～0x77（例如 24C02 EEPROM 常见 0x50）。
* 一条总线上地址不能重复，否则两个从机会同时 ACK，数据被一起破坏。
*
* 【i2c-dev 是什么】
* Linux 把每个 I2C 适配器（控制器）暴露成字符设备 /dev/i2c-N，打开后内核为这个 fd 建一个
* 用户态的 i2c_client。需要 CONFIG_I2C_CHARDEV（模块名 i2c-dev），节点通常是 root:i2c 0660。
* WSL2 / 普通 PC 一般没有 I2C 控制器，也就没有这个节点，本程序会在 open 处报错退出。
*
* 【本文件的形态】
*	一次 write() = 1 条 i2c_msg = 线上一个完整的 START → 地址+W → 数据… → STOP。
*	一次 read()  = 1 条 i2c_msg = 线上一个完整的 START → 地址+R → 数据… → STOP。
*	一次调用提交一次、同步阻塞完成；返回时数据已经拷回用户缓冲，没有队列、没有异步接口。
*	缓冲由用户提供；内核需要一份自己的缓冲来完成传输（i2c-dev.c 里 read() 用 kzalloc、
*	write() 用 memdup_user），所以每次调用都有一次用户↔内核拷贝；用户缓冲不需要对齐。
*	（I2C_RDWR 路径会额外给消息标 I2C_M_DMA_SAFE，见 i2c_rdwr.cpp。）
*	单次 read()/write() 的字节数上限是 8192，超过会被 i2c-dev 直接截断到 8192
*	（drivers/i2c/i2c-dev.c：`if (count > 8192) count = 8192;`）。
*	read()/write() 是“裸 I2C”；也支持 ioctl(I2C_SMBUS) 的 SMBus 子集，本文件不涉及。
*
* 【I2C_SLAVE 与 I2C_SLAVE_FORCE】
*	open() 之后 fd 的从机地址是未设置的，必须先 ioctl(fd, I2C_SLAVE, addr)。
*	内核会检查这个地址是否已经被某个内核驱动（i2c_client）占用：
*	  - I2C_SLAVE：被占用时返回 -1 / EBUSY，不会设置地址；
*	  - I2C_SLAVE_FORCE：跳过这个检查，强行把地址设上。
*	用 FORCE 的后果是内核驱动和你的程序同时操作同一颗芯片：驱动的寄存器缓存、状态机、
*	中断处理都会和你的读写打架，可能把设备配坏。所以除非你确认没有驱动绑定（或已经解绑），
*	否则不应该用 FORCE。本示例遇到 EBUSY 只打印说明，不自动改用 FORCE。
*	注意 I2C_SLAVE 的检查不是“占坑”：地址之后仍可能被驱动绑定，而且另一个进程/另一个 fd
*	也能设成同一个地址。i2c-dev 不提供按从机的互斥锁，需要互斥得自己做（flock 等）。
*
* 【本文件要讲的坑：先写寄存器地址再读，拆成两次调用】
*	write(fd, &reg, 1) 然后 read(fd, buf, 16)，线上是：
*	    S addr+W reg P    S addr+R data… P
*	中间有一个完整的 STOP。它只对“寄存器指针在 STOP 后仍然保持”的器件成立（EEPROM、多数
*	传感器寄存器接口）。对以下器件是错的：
*	  - 要求 STOP 后重新寻址、或者 STOP 会清空命令状态的器件；
*	  - 每次 read 都返回“当前 FIFO 头”而不是“寄存器指针处数据”的器件；
*	  - 两次调用之间被其它进程/驱动插进来操作了同一颗芯片，寄存器指针已经变了。
*	正确做法是用 ioctl(I2C_RDWR) 把“写地址 + 读数据”放进一次调用，让内核用 repeated START
*	连接两段，中间不发 STOP —— 见同目录 i2c_rdwr.cpp。
*
* 【需要的条件】
*	Linux + i2c-dev + 真实 I2C 控制器和从机；设备路径/地址从命令行参数取。
*	本机（WSL2，内核 6.18）没有 /dev/i2c-*，程序会在 open 处优雅报错，这是预期行为。
*
* 【编译】
*	g++ -std=c++26 -O2 -Wall -Wextra -o i2c_slave_rw i2c_slave_rw.cpp
*	不需要链接额外库。
*
* 【运行】
*	./i2c_slave_rw [/dev/i2c-1] [0x50]
*	有设备时：打印适配器功能位，设置超时/重试和从机地址，写 1 字节寄存器地址 0x00，
*	再用一次独立的 read() 读 16 字节并打印。
*	无设备时：打印“打开 I2C 适配器 … 失败: No such file or directory”，退出码 1。
*
* 【预期输出（有设备时，数据随器件而定）】
*	打开 /dev/i2c-1，从机地址 0x50
*	适配器功能位 funcs = 0x0xxxxxxxxx
*	I2C_SLAVE 成功：之后这个 fd 上的 read()/write() 都发给 0x50
*	--- 1) write()：发送寄存器地址 0x00（独立事务，末尾有 STOP）---
*	write() 返回 1
*	--- 2) read()：读 16 字节（另一条独立事务，不会再发寄存器地址）---
*	read() 返回 16
*	数据 (16 字节): 00 01 02 ...
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
constexpr std::uint16_t kDefaultAddr = 0x50;	/* 24C02 之类 EEPROM 的常见地址 */
constexpr std::size_t kReadBytes = 16;

/*
* RAII 持有 /dev/i2c-N 的 fd：
*	构造 open，失败抛 std::system_error；
*	析构 close（i2c-dev 的 release 会释放这个 fd 对应的 i2c_client，不影响其它 fd）；
*	禁止拷贝，允许移动；get() 把裸 fd 交给 ioctl/read/write。
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

/*
* open 失败在本机是常态（WSL2 没有 I2C），所以单独处理：
* 打印中文原因和排查提示后直接退出，不让异常穿过 main 打出一串英文。
*/
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

/* 命令行里的地址按 C 风格解析（支持 0x50、80），只接受 7 位地址。 */
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

/*
* 带返回值的 ioctl 薄封装（只用于“参数就是一个整数”的命令）。
* ioctl 的第三个参数是可变参数，整数必须显式转成 unsigned long，否则类型不确定。
* 成功返回 0，失败返回 -1 并设置 errno。
*/
void i2c_ioctl_ulong(const I2cFd &dev, unsigned long request, unsigned long arg,
	const char *what)
{
	if (::ioctl(dev.get(), request, arg) < 0)
		throw std::system_error{errno, std::system_category(), what};
}

/* 取适配器能力位，I2C_FUNCS 的参数是 unsigned long*，不是整数。 */
unsigned long adapter_funcs(const I2cFd &dev)
{
	unsigned long funcs = 0;
	if (::ioctl(dev.get(), I2C_FUNCS, &funcs) < 0)
		throw std::system_error{errno, std::system_category(), "I2C_FUNCS"};
	return funcs;
}

/*
* 把 read()/write() 的 errno 翻译成人话。这些错误码的含义来自内核
* Documentation/i2c/fault-codes.rst（本机没有该文档，凭内核文档记忆，未核对）：
*	ENXIO      —— 地址阶段 NACK：这个地址上没有器件（没插、没上电、地址错）。
*	EREMOTEIO  —— 地址 ACK 了，但某个数据字节 NACK：越界读、写保护、器件忙。
*	EOPNOTSUPP —— 适配器不支持裸 I2C 传输（例如只实现了 SMBus 子集的适配器）。
*	ETIMEDOUT  —— 总线超时：SDA/SCL 被某个器件一直拉低，或上拉/接线有问题。
*	EIO        —— 其它总线错误（仲裁丢失、线路干扰等）。
*/
void explain_io_error(const char *op, int e)
{
	std::println(stderr, "{} 失败: {} (errno {})", op, std::strerror(e), e);
	switch (e) {
	case ENXIO:
		std::println(stderr, "  地址阶段 NACK：该地址上没有器件，检查地址/供电/接线。");
		break;
	case EREMOTEIO:
		std::println(stderr, "  数据阶段 NACK：器件 ACK 了地址但拒绝了数据，检查读长度/写保护。");
		break;
	case EOPNOTSUPP:
		std::println(stderr, "  适配器不支持该操作（例如只支持 SMBus，不支持裸 I2C）。");
		break;
	case ETIMEDOUT:
		std::println(stderr, "  总线超时：SDA/SCL 可能被拉死，检查上拉电阻和从机。");
		break;
	case EIO:
		std::println(stderr, "  总线错误：检查接线、上拉、是否有两个器件抢同一地址。");
		break;
	default:
		break;
	}
}

/* 按字节打印缓冲；std::span 让调用方传 std::array 或裸指针+长度都行。 */
void dump(std::string_view tag, std::span<const std::uint8_t> bytes)
{
	std::print("{} ({} 字节):", tag, bytes.size());
	for (const std::uint8_t b : bytes)
		std::print(" {:02X}", b);
	std::println();
}

} // namespace

int main(int argc, char **argv)
{
	const std::string_view dev_path = (argc > 1) ? argv[1] : kDefaultDev;
	const std::uint16_t addr = (argc > 2) ? parse_addr(argv[2]) : kDefaultAddr;

	/*
	* O_RDWR：读和写都要用。只读打开在 i2c-dev 上也能建 client，但 write() 会失败，
	* 所以统一用 O_RDWR。O_CLOEXEC 防止 fd 漏给 exec 出去的子进程。
	*/
	I2cFd dev = open_device(dev_path);
	std::println("打开 {}，从机地址 0x{:02X}", dev_path, addr);

	try {
		/*
		* ---------- 先看适配器能力 ----------
		*
		* I2C_FUNCS 返回的是“这个控制器会什么”的位图，位定义见 /usr/include/linux/i2c.h：
		*	I2C_FUNC_I2C               —— 支持任意 i2c_msg 传输（I2C_RDWR/read/write 的基础）
		*	I2C_FUNC_SMBUS_*           —— 支持对应的 SMBus 事务
		*	I2C_FUNC_NOSTART           —— 支持 I2C_M_NOSTART
		*	I2C_FUNC_PROTOCOL_MANGLING —— 支持 I2C_M_IGNORE_NAK 等“改协议”标志
		*	I2C_FUNC_10BIT_ADDR        —— 支持 10 位地址
		* 裸 read()/write()/ioctl(I2C_RDWR) 要求 I2C_FUNC_I2C；不支持就直接报出来，
		* 不要等后面收到一个莫名其妙的 EOPNOTSUPP。
		*/
		const unsigned long funcs = adapter_funcs(dev);
		std::println("适配器功能位 funcs = {:#010x}", funcs);
		if (!(funcs & I2C_FUNC_I2C)) {
			std::println(stderr, "该适配器不支持 I2C_FUNC_I2C（裸 I2C 传输），本示例无法继续。");
			return 1;
		}

		/*
		* ---------- 两个可选的适配器参数 ----------
		*
		* I2C_TIMEOUT：单位是 10 ms，100 就是 1 秒。超时后传输返回 ETIMEDOUT。
		* I2C_RETRIES：地址 NACK 时内核重试的次数（轮询那些上电慢的器件）。
		*
		* 坑：这两个值存在 adapter 结构里，是“整条总线”共享的，不是这个 fd 私有。
		* 你在这里改掉，会影响同一适配器上所有 i2c-dev 用户和内核驱动。
		*/
		i2c_ioctl_ulong(dev, I2C_TIMEOUT, 100, "I2C_TIMEOUT");
		i2c_ioctl_ulong(dev, I2C_RETRIES, 1, "I2C_RETRIES");

		/*
		* ---------- 选定从机地址 ----------
		*
		* I2C_SLAVE 的参数就是 7 位地址本身（不是左移一位后的 8 位形式）。
		* 内核检查：arg > 0x7F（且没开 I2C_TENBIT）→ EINVAL；
		* 地址已被内核驱动占用 → EBUSY。
		* 设置成功后，这个 fd 后续的 read()/write() 都使用该地址，直到再次 ioctl。
		*/
		if (::ioctl(dev.get(), I2C_SLAVE, static_cast<unsigned long>(addr)) < 0) {
			const int e = errno;
			if (e == EBUSY) {
				std::println(stderr,
					"I2C_SLAVE 0x{:02X} 返回 EBUSY：该地址已被某个内核驱动占用。", addr);
				std::println(stderr,
					"I2C_SLAVE_FORCE 可以绕过检查，但会和内核驱动同时操作同一颗芯片，");
				std::println(stderr,
					"可能破坏驱动状态，因此本示例不自动使用 FORCE。");
				return 1;
			}
			throw std::system_error{e, std::system_category(), "I2C_SLAVE"};
		}
		std::println("I2C_SLAVE 成功：之后这个 fd 上的 read()/write() 都发给 0x{:02X}", addr);

		/*
		* ---------- 事务 1：write() 发送 1 字节寄存器地址 ----------
		*
		* 线上：S  addr+W  0x00  P
		*	START → 地址字节（方向位 0）→ 1 个数据字节 → STOP。
		* 返回值：成功是实际写入的字节数（这里 1），失败是 -1 并设置 errno。
		* i2c-dev 的 write() 不会只写一半：要么返回 len，要么返回 -1。
		*
		* 这里的 0x00 只是示例“寄存器地址”。真实器件要查手册：
		* 例如 EEPROM 是字地址，传感器可能是命令码而不是寄存器偏移。
		*/
		std::println("--- 1) write()：发送寄存器地址 0x00（独立事务，末尾有 STOP）---");
		const std::array<std::uint8_t, 1> reg{0x00};
		const ssize_t wn = ::write(dev.get(), reg.data(), reg.size());
		if (wn < 0) {
			explain_io_error("write", errno);
			return 1;
		}
		if (wn != static_cast<ssize_t>(reg.size())) {
			std::println(stderr, "write() 短写：只写了 {} 字节（i2c-dev 不应出现）", wn);
			return 1;
		}
		std::println("write() 返回 {}", wn);

		/*
		* ---------- 事务 2：read() 读 16 字节 ----------
		*
		* 线上：S  addr+R  data[0] … data[15]  P
		* 主机每收一个字节回 ACK，最后一个字节回 NACK，然后 STOP。
		* read() 不会重发上面那个寄存器地址 —— 它是一次全新的、独立的事务。
		* 对“STOP 后寄存器指针保持”的器件，这恰好能读到 0x00 开始的内容；
		* 对其他器件，读到的可能是别的东西。要保证“写地址+读”不可分割，
		* 必须用 I2C_RDWR 让内核用 repeated START 连接（见 i2c_rdwr.cpp）。
		*
		* 返回值：成功是读到的字节数（这里应为 16），失败是 -1。
		*/
		std::println("--- 2) read()：读 {} 字节（另一条独立事务，不会再发寄存器地址）---",
			kReadBytes);
		std::array<std::uint8_t, kReadBytes> buf{};
		const ssize_t rn = ::read(dev.get(), buf.data(), buf.size());
		if (rn < 0) {
			explain_io_error("read", errno);
			return 1;
		}
		std::println("read() 返回 {}", rn);
		dump("数据", std::span<const std::uint8_t>{buf}.first(
			static_cast<std::size_t>(rn)));
	} catch (const std::system_error &e) {
		std::println(stderr, "I2C 操作失败: {}", e.what());
		return 1;
	}

	/*
	* 离开作用域：dev 析构 → close(fd)。i2c-dev 不对从机做任何“复位”或“解锁”，
	* 器件状态原样保留；如果传输中途被打断，从机可能停在半个字节上（总线卡死），
	* 需要靠 SCL 打 9 个时钟之类的总线恢复手段，这不是 i2c-dev 自动处理的。
	*/
	return 0;
}
