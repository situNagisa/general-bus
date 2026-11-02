/*
 * smbus_word_data.cpp —— ioctl(I2C_SMBUS) 做 SMBus read/write word data
 *
 * 这个接口是什么
 *	SMBus 是 I2C 的一个子集，用固定“句式”完成寄存器式读写：主机先写 1 字节
 *	命令（从机内部寄存器/命令号），再读或写数据。最常用的两种是
 *	  - write word data：命令 + 2 字节数据；
 *	  - read word data ：命令 + 2 字节数据（读方向）。
 *	Linux 把这些 SMBus 事务统一交给一个 ioctl：ioctl(fd, I2C_SMBUS, &args)，
 *	args 是 struct i2c_smbus_ioctl_data，里面用 size 字段选择事务类型
 *	（I2C_SMBUS_QUICK / BYTE / BYTE_DATA / WORD_DATA / PROCESS_CALL /
 *	BLOCK_DATA / I2C_BLOCK_DATA ...），read_write 选择方向，command 是命令字节，
 *	data 指向存放数据的 union i2c_smbus_data。它解决的是“不要自己拼 I2C 时序，
 *	也不要依赖 SMBus 主机控制器专有接口”，一个 ioctl 就能完成一个标准 SMBus
 *	事务。用户态常见的 i2c_smbus_read_word_data()/i2c_smbus_write_word_data()
 *	只是这个 ioctl 的薄包装（见下），自己写包装只需要十几行，不必链接 libi2c。
 *
 * 形态（一次调用交几段内存、对应几个线上单元、提交几次、完成几次）
 *	一次 ioctl = 一个 SMBus 事务 = 线上 1 段或 2 段：
 *	  write word：S Addr+W Cmd D0 D1 [PEC] P        —— 1 段
 *	  read  word：S Addr+W Cmd Sr Addr+R D0 D1 [PEC] P —— 2 段（repeated START）
 *	一次提交、同步等待、一次完成；没有队列，没有异步完成通知。数据缓冲区由
 *	调用者在用户态提供（union i2c_smbus_data），ioctl 期间内核把它拷进内核，
 *	传输完再拷回；data 指针只在调用期间有效。返回 0 表示成功；-1 表示失败，
 *	errno 给出原因（ENXIO/EREMOTEIO 从机不应答、EOPNOTSUPP 适配器不支持该事务、
 *	EINVAL 参数不合法、ETIMEDOUT 超时、EBADMSG PEC 校验失败等）。
 *
 * word data 的字节序
 *	SMBus 规定 word 先传低字节（little-endian on the wire）。用户态只需要给
 *	data.word 赋 uint16_t；读回来的 data.word 已经是主机字节序的值。也就是说
 *	写 0x1234 时线上是 0x34 0x12，read 回来的仍是 0x1234，不要自己再交换。
 *
 * 和 byte data / quick / receive byte 的对照（同一套 ioctl，只换 size/方向）
 *	quick        ：S Addr+W P（写方向）或 S Addr+R P（读方向），无命令、无数据；
 *	               data 可以为 NULL。常用于“设备在不在”的探测。
 *	send byte    ：S Addr+W D P        —— 1 字节，无命令字节（size=BYTE，写）；
 *	receive byte ：S Addr+R D P        —— 1 字节，无命令字节（size=BYTE，读）；
 *	byte data    ：S Addr+W Cmd D P    / S Addr+W Cmd Sr Addr+R D P —— 命令 + 1 字节；
 *	word data    ：命令 + 2 字节（本文件重点）；
 *	process call ：写命令 + 2 字节，同一事务内 repeated START 读回 2 字节；
 *	block data   ：命令 + 长度 + 最多 32 字节（见 smbus_block_read.cpp）。
 *	注意 quick/byte 没有命令字节，此时 command 字段虽然存在但线上不发。
 *
 * PEC（Packet Error Code）
 *	SMBus 2.0 可选在事务末尾加 1 字节 CRC-8 校验。用户态打开方式：
 *	ioctl(fd, I2C_PEC, 1)，参数是整数值（不是指针），对本 fd 后续所有 SMBus
 *	事务生效；内核负责在写事务末尾追加 PEC、在读事务里接收并校验 PEC，校验
 *	失败返回 -EBADMSG。前提是适配器 functionality 含 I2C_FUNC_SMBUS_PEC。
 *	PEC 字节不会出现在 union i2c_smbus_data 里，用户态看不到也不用算。
 *
 * 需要的内核/硬件条件
 *	- 内核 i2c-dev，/dev/i2c-N 可打开；
 *	- 适配器 functionality 含对应 SMBUS_* 位；或者含 I2C_FUNC_I2C，让 i2c core
 *	  用普通 i2c_msg 软件模拟（声明 I2C_FUNC_SMBUS_EMUL 时）；
 *	- 总线上有一个真实的 SMBus 从机（本机 WSL2 没有，因此运行只会优雅报错）。
 *
 * 编译
 *	g++ -std=c++26 -O2 -Wall -Wextra -o /tmp/native_smbus_word_data smbus_word_data.cpp
 *	（不链接 libi2c；所有包装都在本文件内实现）
 *
 * 运行
 *	/tmp/native_smbus_word_data [设备路径] [从机地址] [--pec]
 *	例：/tmp/native_smbus_word_data /dev/i2c-1 0x50
 *	    /tmp/native_smbus_word_data /dev/i2c-3 0x50 --pec
 *
 * 预期输出
 *	有设备：依次打印 write/read word、write/read byte data、quick、receive byte
 *	每一步的线上形态、返回值与读到的数据。
 *	没有设备：打开失败时打印原因和排查提示，退出码 1。
 *
 * 常见坑
 *	- 必须先 ioctl(fd, I2C_SLAVE, addr) 设置从机地址；i2c-dev 的 fd 是“总线 +
 *	  当前从机地址”的组合，不设置就对着地址 0 发事务。
 *	- I2C_SLAVE 的第三个参数是地址数值本身（unsigned long），不是指针。
 *	- 地址被内核驱动占用时 I2C_SLAVE 返回 EBUSY；I2C_SLAVE_FORCE 可以强占，
 *	  但会让原驱动失控，除非你明确知道在干什么，否则不要用。
 *	- 这个示例会真的写从机（对 EEPROM 就是改存储内容）。在真实设备上运行前
 *	  确认命令字节指向的位置可以写。
 *	- read 类调用的返回值：libi2c 的包装成功返回数据、失败返回 -1（errno 被
 *	  吞掉）。本示例的包装成功返回 0、失败返回 -errno，便于打印原因。
 */

#include <array>
#include <cerrno>
#include <charconv>
#include <cstdint>
#include <cstring>
#include <print>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include <fcntl.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <linux/i2c-dev.h>
#include <linux/i2c.h>

namespace {

/* RAII fd：析构时 close，避免任何 return 路径漏掉。 */
class UniqueFd {
public:
	explicit UniqueFd(int fd) noexcept : fd_{fd} {}
	~UniqueFd() { reset(); }

	UniqueFd(const UniqueFd &) = delete;
	UniqueFd &operator=(const UniqueFd &) = delete;
	UniqueFd(UniqueFd &&other) noexcept : fd_{std::exchange(other.fd_, -1)} {}

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

/* 所有 SMBus 包装共用的入口：填 struct i2c_smbus_ioctl_data 再 ioctl。
 *
 * 这个结构就是内核 uapi 里的原样定义（linux/i2c-dev.h）：
 *	read_write：I2C_SMBUS_WRITE(0) 或 I2C_SMBUS_READ(1)
 *	command   ：命令字节（quick/byte 事务不使用）
 *	size      ：事务类型，I2C_SMBUS_* 常量（linux/i2c.h）
 *	data      ：指向 union i2c_smbus_data；quick 可以传 NULL
 * 返回 0 表示 ioctl 成功；失败返回 -errno（不是 -1），让调用者能打印原因。
 *
 * libi2c 里的 i2c_smbus_access() 几乎就是这几行，只不过失败时返回 -1。
 * 所以“i2c_smbus_read_word_data 只是 ioctl 的薄包装”不是比喻：它就是把
 * 上述结构填好，调用一次 ioctl，再从 data.word 里取值并做 0xffff 掩码。 */
[[nodiscard]] int smbus_access(int fd, std::uint8_t read_write, std::uint8_t command,
			       std::uint32_t size, union i2c_smbus_data *data) noexcept
{
	struct i2c_smbus_ioctl_data args {
		.read_write = read_write,
		.command = command,
		.size = size,
		.data = data,
	};

	if (::ioctl(fd, I2C_SMBUS, &args) < 0)
		return -errno;
	return 0;
}

/* write word data：命令 + 2 字节写。
 * data.word 由内核按小端拆成两个线上字节（先低后高）。 */
[[nodiscard]] int write_word_data(int fd, std::uint8_t command, std::uint16_t value) noexcept
{
	union i2c_smbus_data data {};
	data.word = value;
	return smbus_access(fd, I2C_SMBUS_WRITE, command, I2C_SMBUS_WORD_DATA, &data);
}

/* read word data：命令 + 2 字节读。成功时把 0..0xffff 写进 value。
 * 返回 0 成功，负 errno 失败。 */
[[nodiscard]] int read_word_data(int fd, std::uint8_t command, std::uint16_t &value) noexcept
{
	union i2c_smbus_data data {};
	const int ret = smbus_access(fd, I2C_SMBUS_READ, command,
				     I2C_SMBUS_WORD_DATA, &data);
	if (ret < 0)
		return ret;
	value = data.word;
	return 0;
}

/* byte data：命令 + 1 字节。这两个函数演示“只换 size 和 union 成员”。 */
[[nodiscard]] int write_byte_data(int fd, std::uint8_t command, std::uint8_t value) noexcept
{
	union i2c_smbus_data data {};
	data.byte = value;
	return smbus_access(fd, I2C_SMBUS_WRITE, command, I2C_SMBUS_BYTE_DATA, &data);
}

[[nodiscard]] int read_byte_data(int fd, std::uint8_t command, std::uint8_t &value) noexcept
{
	union i2c_smbus_data data {};
	const int ret = smbus_access(fd, I2C_SMBUS_READ, command,
				     I2C_SMBUS_BYTE_DATA, &data);
	if (ret < 0)
		return ret;
	value = data.byte;
	return 0;
}

/* quick：没有命令字节和数据字节，data 传 NULL 是 libi2c 的既有做法；
 * 只发地址 + 写方向位。读方向的 quick 很少用，这里只演示写方向。 */
[[nodiscard]] int write_quick(int fd) noexcept
{
	return smbus_access(fd, I2C_SMBUS_WRITE, 0, I2C_SMBUS_QUICK, nullptr);
}

/* receive byte：没有命令字节，直接读当前指针处的 1 字节。
 * 它和 read_byte_data 的区别就在线上有没有那个命令字节。 */
[[nodiscard]] int read_byte(int fd, std::uint8_t &value) noexcept
{
	union i2c_smbus_data data {};
	const int ret = smbus_access(fd, I2C_SMBUS_READ, 0, I2C_SMBUS_BYTE, &data);
	if (ret < 0)
		return ret;
	value = data.byte;
	return 0;
}

/* 打印一步的结果：成功打印 OK，失败打印 errno 文本。 */
void report(std::string_view what, int ret)
{
	if (ret == 0)
		std::println("  [OK] {}", what);
	else
		std::println("  [FAIL] {}：{} (errno={})", what, std::strerror(-ret), -ret);
}

[[nodiscard]] bool parse_address(std::string_view text, std::uint16_t &addr) noexcept
{
	unsigned value = 0;
	std::string_view digits = text;
	int base = 10;
	if (digits.starts_with("0x") || digits.starts_with("0X")) {
		base = 16;
		digits.remove_prefix(2);
	}
	const auto [ptr, ec] = std::from_chars(digits.data(),
					      digits.data() + digits.size(), value, base);
	if (ec != std::errc {} || ptr != digits.data() + digits.size())
		return false;
	/* 只接受 7 位地址。10 位地址需要 I2C_TENBIT，而 i2c-dev.h 的注释明确说
	 * 用户态 10 位地址“不受支持（代码有缺陷）”，这里不展开。 */
	if (value > 0x7f)
		return false;
	addr = static_cast<std::uint16_t>(value);
	return true;
}

/* 事务形态对照表，运行结束时打印，帮助把 size 常量和线上字节对应起来。 */
struct Shape {
	std::string_view size_name;
	std::string_view wire;
};

constexpr std::array<Shape, 6> kShapes{{
	{ "QUICK",     "S Addr+W P" },
	{ "BYTE",      "S Addr+W D P  (send) / S Addr+R D P  (receive)" },
	{ "BYTE_DATA", "S Addr+W Cmd D P  (写) / S Addr+W Cmd Sr Addr+R D P  (读)" },
	{ "WORD_DATA", "S Addr+W Cmd D0 D1 P  (写) / ... Sr Addr+R D0 D1 P  (读)" },
	{ "PROC_CALL", "写 Cmd+2 字节，再同一事务读回 2 字节" },
	{ "BLOCK",     "Cmd + Len + 最多 32 字节（长度字节由发送方给出）" },
}};

} // namespace

int main(int argc, char **argv)
{
	std::string_view dev_path = "/dev/i2c-1";
	std::uint16_t addr = 0x50;
	bool want_pec = false;
	std::vector<std::string_view> positional;

	/* 命令行：位置参数是设备路径和从机地址，--pec 是开关。
	 * 解析失败时直接打印用法，退出码 2。 */
	for (int i = 1; i < argc; ++i) {
		std::string_view arg { argv[i] };
		if (arg == "--pec")
			want_pec = true;
		else if (arg == "-h" || arg == "--help") {
			std::println("用法：{} [设备路径=/dev/i2c-1] [从机地址=0x50] [--pec]",
				     argv[0]);
			return 0;
		} else if (arg.starts_with("-")) {
			std::println(stderr, "未知选项：{}", arg);
			return 2;
		} else {
			positional.push_back(arg);
		}
	}

	if (positional.size() >= 1)
		dev_path = positional[0];
	if (positional.size() >= 2 && !parse_address(positional[1], addr)) {
		std::println(stderr, "从机地址格式错误（只支持 7 位地址，如 0x50）：{}",
			     positional[1]);
		return 2;
	}
	if (positional.size() > 2) {
		std::println(stderr, "参数过多，最多两个位置参数");
		return 2;
	}

	UniqueFd fd { ::open(std::string(dev_path).c_str(), O_RDWR | O_CLOEXEC) };
	if (!fd.valid()) {
		std::println(stderr, "打开 {} 失败：{} (errno={})", dev_path,
			     std::strerror(errno), errno);
		std::println(stderr, "本机没有 I2C 适配器时这是预期结果；确认 /dev/i2c-* 存在、");
		std::println(stderr, "已加载 i2c-dev、当前用户有权限访问该设备节点。");
		return 1;
	}
	std::println("设备：{}，从机地址：{:#04x}", dev_path, addr);

	/* I2C_SLAVE 把“当前从机地址”绑定到这份打开的 fd 上。成功返回 0；
	 * 地址已被别的内核驱动占用时返回 EBUSY（此时用 I2C_SLAVE_FORCE 会强行
	 * 抢过来，可能破坏原驱动，不演示）。 */
	if (::ioctl(fd.get(), I2C_SLAVE, static_cast<unsigned long>(addr)) < 0) {
		std::println(stderr, "ioctl(I2C_SLAVE, {:#04x}) 失败：{} (errno={})", addr,
			     std::strerror(errno), errno);
		return 1;
	}

	/* 先查适配器能力，能提前判断哪些步骤必然失败。I2C_FUNCS 返回 unsigned long
	 * 位图；复合宏要求所有位都具备（内核 i2c_check_functionality 的语义）。 */
	unsigned long funcs = 0;
	if (::ioctl(fd.get(), I2C_FUNCS, &funcs) < 0) {
		std::println(stderr, "ioctl(I2C_FUNCS) 失败：{} (errno={})",
			     std::strerror(errno), errno);
		return 1;
	}
	const bool has_i2c_level = (funcs & I2C_FUNC_I2C) != 0;
	const bool can_word = (funcs & I2C_FUNC_SMBUS_WORD_DATA) == I2C_FUNC_SMBUS_WORD_DATA;
	const bool can_byte_data =
		(funcs & I2C_FUNC_SMBUS_BYTE_DATA) == I2C_FUNC_SMBUS_BYTE_DATA;
	const bool can_pec = (funcs & I2C_FUNC_SMBUS_PEC) != 0;

	std::println("适配器掩码 0x{:08x}：word data {}，byte data {}，PEC {}，I2C 级传输 {}",
		     funcs, can_word ? "支持" : "不支持", can_byte_data ? "支持" : "不支持",
		     can_pec ? "支持" : "不支持", has_i2c_level ? "支持" : "不支持");
	if (!can_word && !has_i2c_level)
		std::println("警告：适配器没有 word data 位，也不能走 I2C 软件模拟，"
			     "下面的 word 操作预计返回 EOPNOTSUPP。");

	/* PEC 是按 fd 设置的开关。ioctl 的第三个参数是值（0/1），不是指针。
	 * 打开后 kernel 对后续 SMBus 事务自动加/校验 PEC；适配器不支持时
	 * ioctl 本身可能返回 EOPNOTSUPP。 */
	if (want_pec) {
		if (::ioctl(fd.get(), I2C_PEC, 1) < 0)
			std::println("警告：打开 PEC 失败：{} (errno={})，继续不带 PEC",
				     std::strerror(errno), errno);
		else
			std::println("已打开 SMBus PEC：后续事务末尾自动加 CRC-8 校验字节");
	}

	std::println("\n注意：下面会真正写从机；若从机是 EEPROM，命令字节 0x10/0x00 "
		     "指向的位置会被改写。");

	/* —— 1. write word data：命令 0x10，值 0x1234 ——
	 * 线上：S Addr+W 0x10 0x34 0x12 [PEC] P。
	 * ioctl 成功只代表“事务完成且从机 ACK 了”，不代表从机真的存下了。 */
	std::println("\n1) write_word_data(command=0x10, value=0x1234)");
	report("write word", write_word_data(fd.get(), 0x10, 0x1234));

	/* —— 2. read word data：同一命令读回 2 字节 ——
	 * 线上：S Addr+W 0x10 Sr Addr+R 0x34 0x12 [PEC] P。
	 * 读到的 data.word 已经是主机字节序的 0x1234。 */
	std::println("\n2) read_word_data(command=0x10)");
	std::uint16_t word = 0;
	int ret = read_word_data(fd.get(), 0x10, word);
	if (ret == 0)
		std::println("  [OK] read word：0x{:04x}", word);
	else
		std::println("  [FAIL] read word：{} (errno={})", std::strerror(-ret), -ret);

	/* —— 3/4. byte data 对照：只把 2 字节换成 1 字节 —— */
	std::println("\n3) write_byte_data(command=0x00, value=0x5a)");
	report("write byte", write_byte_data(fd.get(), 0x00, 0x5a));

	std::println("\n4) read_byte_data(command=0x00)");
	std::uint8_t byte = 0;
	ret = read_byte_data(fd.get(), 0x00, byte);
	if (ret == 0)
		std::println("  [OK] read byte：0x{:02x}", byte);
	else
		std::println("  [FAIL] read byte：{} (errno={})", std::strerror(-ret), -ret);

	/* —— 5. quick：没有命令、没有数据，只发地址 + 写位 ——
	 * 常用于探测从机是否在总线上；从机 ACK 则 ioctl 返回 0。 */
	std::println("\n5) write_quick()");
	report("quick", write_quick(fd.get()));

	/* —— 6. receive byte：没有命令字节，直接读 1 字节 ——
	 * 很多设备把它解释为“读内部地址指针当前指向的字节”。 */
	std::println("\n6) read_byte()（receive byte）");
	ret = read_byte(fd.get(), byte);
	if (ret == 0)
		std::println("  [OK] receive byte：0x{:02x}", byte);
	else
		std::println("  [FAIL] receive byte：{} (errno={})", std::strerror(-ret), -ret);

	/* 最后把“size 常量 ↔ 线上形态”列出来，方便回头对照上面的输出。
	 * 同一套 ioctl，所有事务的差别只在 size、read_write 和 union 成员。 */
	std::println("\nSMBus 事务形态对照：");
	for (const auto &shape : kShapes)
		std::println("  {:<10} {}", shape.size_name, shape.wire);

	return 0;
}
