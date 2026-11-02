/*
* i2c_funcs.cpp —— ioctl(I2C_FUNCS)：查询一个 I2C 适配器到底会哪些传输
*
* 这个接口是什么
*	打开 /dev/i2c-N 之后，用 ioctl(fd, I2C_FUNCS, &mask) 向 i2c-dev 驱动要一个
*	“功能位图”（unsigned long）。每一位表示适配器驱动声明自己支持的一类传输：
*	任意 I2C 级读写（I2C_FUNC_I2C）、10 位地址、协议 mangling、SMBus PEC、
*	以及一大堆固定的 SMBus 事务（quick / byte / byte data / word data / block ...）。
*	它解决的是“我的代码能不能对这个适配器用某种传输”的问题：在发数据之前先问
*	适配器，而不是发出去再收 EOPNOTSUPP。写通用 I2C 工具、自动探测总线、或者
*	在不同硬件上移植代码时，第一步就应该查它。
*
* 形态（一次调用交几段内存、对应几个线上单元、提交几次、完成几次）
*	一次 ioctl，交 1 个 unsigned long（调用者提供，内核填充），不产生任何线上
*	时序；同步、立即完成，没有队列，没有回调。返回 0 表示成功，-1 且 errno 表示
*	失败。掩码是适配器驱动“声称”的能力，不是每次传输的保证：电气问题、从机
*	不应答等仍然会在真正传输时失败。
*
* 需要的内核/硬件条件
*	- 内核带 i2c-dev（Arch 上 modprobe i2c-dev），并且存在 /dev/i2c-N。
*	- 本机 WSL2 一般没有暴露 I2C 控制器；插 USB 转 I2C 适配器并由其驱动注册
*	  出 i2c adapter 后才会出现 /dev/i2c-N。
*
* 编译
*	g++ -std=c++26 -O2 -Wall -Wextra -o /tmp/native_i2c_funcs i2c_funcs.cpp
*
* 运行
*	/tmp/native_i2c_funcs [设备路径]        # 默认 /dev/i2c-1
*	例：/tmp/native_i2c_funcs /dev/i2c-3
*
* 预期输出
*	有适配器：打印掩码（十六进制）和逐位解释，最后给出“能否用 I2C_RDWR”的结论。
*	本机没有 /dev/i2c-1：打印打开失败的原因和排查提示后以退出码 1 结束。
*
* 关于“只支持 SMBus、不支持任意 I2C_RDWR”的适配器
*	SMBus 是 I2C 的一个“固定句式”子集：只有 quick、send/receive byte、
*	read/write byte data、word data、block data、process call 这几种事务，
*	每种事务的字节数和线上形态都是规定死的。很多 SMBus 主机控制器（部分
*	ACPI SMBus、电源管理芯片、USB 转 I2C 桥的 SMBus 模式）硬件上只有一个
*	按事务类型工作的状态机，不能表达“随便写 N 字节再 repeated START 读 M 字节”
*	这种任意 I2C 序列，因此它的 functionality 里有 I2C_FUNC_SMBUS_* 却没有
*	I2C_FUNC_I2C。对这样的适配器：
*	  - ioctl(fd, I2C_RDWR, ...) 会被 i2c-dev 直接拒绝，返回 -1/EOPNOTSUPP
*	    （v6.18 的 drivers/i2c/i2c-dev.c：i2cdev_ioctl_rdwr() 里先做
*	    i2c_check_functionality(adap, I2C_FUNC_I2C)，不满足就返回 -EOPNOTSUPP；
*	    已在线核对）；
*	  - 只能用 ioctl(fd, I2C_SMBUS, ...) 发上面那些固定 SMBus 事务。
*	反过来是成立的：一个支持任意 I2C 的适配器可以通过 I2C_FUNC_SMBUS_EMUL
*	让 i2c core 用普通 i2c_msg 拼出 SMBus 事务（软件模拟），所以“会 I2C”
*	通常就“会 SMBus”，但“会 SMBus”不等于“会 I2C”。
*	I2C_FUNC_SMBUS_EMUL 这个复合宏本身也是“能力声明”：适配器驱动告诉 core
*	“你可以拿 master_xfer 来模拟这些 SMBus 事务”。
*/

#include <array>
#include <cerrno>
#include <cstdio>
#include <cstring>
#include <print>
#include <string_view>
#include <utility>

#include <fcntl.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <linux/i2c-dev.h>
#include <linux/i2c.h>

namespace {

/* RAII 管理 fd：任何返回路径（包括异常/提前 return）都会 close。
* 这里只用最朴素的移动语义，避免示例被智能指针的细节淹没。 */
class UniqueFd {
public:
	explicit UniqueFd(int fd) noexcept : fd_{fd} {}

	~UniqueFd() { reset(); }

	UniqueFd(const UniqueFd &) = delete;
	UniqueFd &operator=(const UniqueFd &) = delete;

	UniqueFd(UniqueFd &&other) noexcept : fd_{std::exchange(other.fd_, -1)} {}

	UniqueFd &operator=(UniqueFd &&other) noexcept
	{
		if (this != &other)
			reset(std::exchange(other.fd_, -1));
		return *this;
	}

	[[nodiscard]] int get() const noexcept { return fd_; }
	[[nodiscard]] bool valid() const noexcept { return fd_ >= 0; }

	void reset(int fd = -1) noexcept
	{
		if (fd_ >= 0)
			::close(fd_);
		fd_ = fd;
	}

private:
	int fd_;
};

/* 表里只放“单个位”的标志。像 I2C_FUNC_SMBUS_WORD_DATA 这种由两个位 OR 出来的
* 复合宏不能放进逐位表，否则会把两个位当成一个位解释；复合能力在打印完位表后
* 单独判断。 */
struct FuncBit {
	unsigned long mask;
	std::string_view name;
	std::string_view meaning;
};

constexpr std::array<FuncBit, 20> kFuncBits{{
	{ I2C_FUNC_I2C,
		"I2C_FUNC_I2C",
		"支持 I2C 级传输（master_xfer）：任意 i2c_msg 序列，I2C_RDWR 的前提" },
	{ I2C_FUNC_10BIT_ADDR,
		"I2C_FUNC_10BIT_ADDR",
		"支持 10 位从机地址（配合 I2C_M_TEN）；i2c-dev 头文件注释说用户态 10 位地址不受支持" },
	{ I2C_FUNC_PROTOCOL_MANGLING,
		"I2C_FUNC_PROTOCOL_MANGLING",
		"允许非标准时序标志：I2C_M_IGNORE_NAK / NO_RD_ACK / REV_DIR_ADDR / STOP" },
	{ I2C_FUNC_SMBUS_PEC,
		"I2C_FUNC_SMBUS_PEC",
		"支持 SMBus PEC（CRC-8 校验字节），可用 ioctl(I2C_PEC) 打开" },
	{ I2C_FUNC_NOSTART,
		"I2C_FUNC_NOSTART",
		"支持 I2C_M_NOSTART：下一段不发 START，直接接着上一段的字节流" },
	{ I2C_FUNC_SLAVE,
		"I2C_FUNC_SLAVE",
		"适配器可作为 I2C 从机/目标（target）工作；i2c-dev 用户态用不到这个模式" },
	{ I2C_FUNC_SMBUS_BLOCK_PROC_CALL,
		"I2C_FUNC_SMBUS_BLOCK_PROC_CALL",
		"SMBus 2.0 block process call：写一个块，同一事务内读回一个块" },
	{ I2C_FUNC_SMBUS_QUICK,
		"I2C_FUNC_SMBUS_QUICK",
		"SMBus quick：只有地址 + R/W 位，没有数据字节" },
	{ I2C_FUNC_SMBUS_READ_BYTE,
		"I2C_FUNC_SMBUS_READ_BYTE",
		"receive byte：读 1 字节，没有命令字节" },
	{ I2C_FUNC_SMBUS_WRITE_BYTE,
		"I2C_FUNC_SMBUS_WRITE_BYTE",
		"send byte：写 1 字节，没有命令字节" },
	{ I2C_FUNC_SMBUS_READ_BYTE_DATA,
		"I2C_FUNC_SMBUS_READ_BYTE_DATA",
		"写 1 字节命令，再读 1 字节数据" },
	{ I2C_FUNC_SMBUS_WRITE_BYTE_DATA,
		"I2C_FUNC_SMBUS_WRITE_BYTE_DATA",
		"写 1 字节命令，再写 1 字节数据" },
	{ I2C_FUNC_SMBUS_READ_WORD_DATA,
		"I2C_FUNC_SMBUS_READ_WORD_DATA",
		"写 1 字节命令，再读 2 字节（小端：低字节先传）" },
	{ I2C_FUNC_SMBUS_WRITE_WORD_DATA,
		"I2C_FUNC_SMBUS_WRITE_WORD_DATA",
		"写 1 字节命令，再写 2 字节（小端）" },
	{ I2C_FUNC_SMBUS_PROC_CALL,
		"I2C_FUNC_SMBUS_PROC_CALL",
		"process call：写命令 + 2 字节，同一事务内读回 2 字节" },
	{ I2C_FUNC_SMBUS_READ_BLOCK_DATA,
		"I2C_FUNC_SMBUS_READ_BLOCK_DATA",
		"block read：长度字节由从机给出（I2C_M_RECV_LEN），最多 32 字节" },
	{ I2C_FUNC_SMBUS_WRITE_BLOCK_DATA,
		"I2C_FUNC_SMBUS_WRITE_BLOCK_DATA",
		"block write：长度由主机给出（1..32），先写长度再写数据" },
	{ I2C_FUNC_SMBUS_READ_I2C_BLOCK,
		"I2C_FUNC_SMBUS_READ_I2C_BLOCK",
		"I2C 风格块读：长度由主机指定，线上没有长度字节" },
	{ I2C_FUNC_SMBUS_WRITE_I2C_BLOCK,
		"I2C_FUNC_SMBUS_WRITE_I2C_BLOCK",
		"I2C 风格块写：长度由主机指定，线上没有长度字节" },
	{ I2C_FUNC_SMBUS_HOST_NOTIFY,
		"I2C_FUNC_SMBUS_HOST_NOTIFY",
		"SMBus 2.0 host notify：从机通过专用线主动通知主机（需要额外中断线）" },
}};

/* 判断“复合能力”是否完整具备。内核的 i2c_check_functionality() 语义就是
* (functionality & func) == func，必须所有位都有，缺一位就算不支持。 */
[[nodiscard]] constexpr bool has(unsigned long funcs, unsigned long func) noexcept
{
	return (funcs & func) == func;
}

[[nodiscard]] std::string_view yes_no(bool value) noexcept
{
	return value ? "支持" : "不支持";
}

void report_open_error(const char *path, int err)
{
	std::println(stderr, "打开 {} 失败：{} (errno={})", path, std::strerror(err), err);
	std::println(stderr, "排查：");
	std::println(stderr, "  1. ls /dev/i2c-*  确认设备节点存在；没有就说明本机没有注册 I2C 适配器");
	std::println(stderr, "  2. sudo modprobe i2c-dev  加载 i2c-dev 驱动");
	std::println(stderr, "  3. WSL2 默认不直通 I2C 控制器；USB 转 I2C 需要对应驱动注册 adapter");
	std::println(stderr, "  4. 检查权限：普通用户通常需要在 i2c 组，或用 sudo 运行");
}

} // namespace

int main(int argc, char **argv)
{
	/* 设备路径从命令行取，默认 /dev/i2c-1；从机地址在这里没有意义，
	* I2C_FUNCS 只问适配器，不涉及某个从机。 */
	const char *path = argc > 1 ? argv[1] : "/dev/i2c-1";

	/* O_RDWR 是 i2c-dev 的常规打开方式；O_CLOEXEC 防止 fd 泄漏给子进程。
	* open 失败通常是设备不存在（ENOENT）、权限不足（EACCES）或驱动没加载。 */
	UniqueFd fd{::open(path, O_RDWR | O_CLOEXEC)};
	if (!fd.valid()) {
		report_open_error(path, errno);
		return 1;
	}
	std::println("已打开 {} (fd={})", path, fd.get());

	/* I2C_FUNCS 的 ioctl 参数是“指向 unsigned long 的指针”，不是整数本身。
	* 内核在 64 位用户态下按 unsigned long 拷贝，所以必须用 unsigned long，
	* 不能图省事用 uint32_t（32 位内核/兼容层另说，但这里按本机 ABI 写）。
	* 成功返回 0，掩码写入 funcs；失败返回 -1 并设置 errno。 */
	unsigned long funcs = 0;
	if (::ioctl(fd.get(), I2C_FUNCS, &funcs) < 0) {
		std::println(stderr, "ioctl(I2C_FUNCS) 失败：{} (errno={})",
				std::strerror(errno), errno);
		return 1;
	}

	std::println("适配器功能掩码：0x{:08x}（{}）", funcs, funcs);

	/* 逐位解释。每一位都对应“一类用户态可以请求的传输”，而不是某个寄存器。 */
	std::println("\n逐位解释：");
	unsigned long known = 0;
	for (const auto &bit : kFuncBits) {
		known |= bit.mask;
		std::println("  {:#010x}  {:<32} {}  —— {}", bit.mask, bit.name,
				yes_no((funcs & bit.mask) != 0), bit.meaning);
	}

	/* 头文件里 0x40..0x4000 之间没有定义单个功能位，是保留空洞；
	* 如果适配器驱动擅自置了这些位，这里如实打印出来，提醒“未知位”。 */
	const unsigned long unknown = funcs & ~known;
	if (unknown != 0)
		std::println("\n未知/保留位：{:#010x}（当前 uapi 头文件未定义，按保留处理）",
				unknown);

	/* 复合能力汇总：这些宏都是“若干位同时具备”才算支持，便于直接对应到
	* 用户态将要调用的 i2c_smbus_* 包装函数。 */
	std::println("\n常用复合能力（所有位都具备才算支持）：");
	std::println("  SMBus byte       : {}", yes_no(has(funcs, I2C_FUNC_SMBUS_BYTE)));
	std::println("  SMBus byte data  : {}", yes_no(has(funcs, I2C_FUNC_SMBUS_BYTE_DATA)));
	std::println("  SMBus word data  : {}", yes_no(has(funcs, I2C_FUNC_SMBUS_WORD_DATA)));
	std::println("  SMBus block data : {}", yes_no(has(funcs, I2C_FUNC_SMBUS_BLOCK_DATA)));
	std::println("  SMBus I2C block  : {}", yes_no(has(funcs, I2C_FUNC_SMBUS_I2C_BLOCK)));
	std::println("  SMBus 模拟全集   : {}", yes_no(has(funcs, I2C_FUNC_SMBUS_EMUL)));

	/* 结论部分：把“能不能用 I2C_RDWR”单独说清楚，因为这正是新手最容易踩的坑。 */
	std::println("\n结论：");
	if (has(funcs, I2C_FUNC_I2C)) {
		std::println("  支持任意 I2C 级传输：可以用 ioctl(I2C_RDWR) 提交任意 i2c_msg 序列。");
	} else {
		std::println("  不支持任意 I2C 级传输（缺少 I2C_FUNC_I2C）：");
		std::println("  - 对该设备发 ioctl(I2C_RDWR) 会失败，errno = EOPNOTSUPP；");
		std::println("  - 只能用 ioctl(I2C_SMBUS) 发适配器声明支持的固定 SMBus 事务；");
		std::println("  - 这是“纯 SMBus 控制器”的典型形态：硬件状态机只会按 SMBus 句式工作，");
		std::println("    无法表达任意长度 + repeated START 的 I2C 序列。");
	}

	if (has(funcs, I2C_FUNC_I2C) && !has(funcs, I2C_FUNC_SMBUS_WORD_DATA)) {
		std::println("  注意：适配器本身没有 word data 位，但 I2C_FUNC_I2C 存在时，");
		std::println("  i2c core 会用普通 i2c_msg 软件模拟 SMBus 事务（前提是声明了");
		std::println("  I2C_FUNC_SMBUS_EMUL 中对应的位），所以 ioctl(I2C_SMBUS) 仍可能成功。");
	}

	/* 能力位只是“声明”。真正传输还可能因为从机不存在（ENXIO/EREMOTEIO）、
	* 总线被占用（EBUSY）、超时（ETIMEDOUT）而失败，别把这里的输出当成保证。 */
	return 0;
}
