/*
 * smbus_block_read.cpp —— SMBus block read：长度字节由从机先给，再按长度读
 *
 * 这个接口是什么
 *	SMBus 的 block data 事务用于“变长回复”：主机写 1 字节命令，然后用
 *	repeated START 读回一个数据块。和 word/byte data 不同，线上第一个读回
 *	的字节不是数据，而是“长度”（0..32），后面才是数据。Linux 用户态用
 *	ioctl(fd, I2C_SMBUS, &args) 发起，size = I2C_SMBUS_BLOCK_DATA。
 *	它解决的是“从机返回多少字节只有从机自己知道”的问题，典型场景是
 *	EEPROM/传感器 FIFO/带可变响应的 SMBus 设备。
 *
 * 形态（一次调用交几段内存、对应几个线上单元、提交几次、完成几次）
 *	一次 ioctl = 一个 SMBus block read 事务 = 线上 2 段：
 *	  S Addr+W Cmd Sr Addr+R Len D0 D1 ... D(len-1) [PEC] P
 *	第 1 段写命令，第 2 段读“长度 + 数据”。一次提交、同步、一次完成。
 *	用户提供 union i2c_smbus_data（block[34]：block[0] 放长度，block[1..]
 *	放数据，多出的 1 字节是 uapi 为兼容性留的），内核把它拷进内核、传输完
 *	再把 block[0] 和 block[1..len] 填好拷回。用户态看不到线上时序。
 *
 * “后一步依赖前一步收到的数据”是什么意思
 *	这是 block read 和普通 read 的本质区别，分三层看：
 *	  1. 协议层：主机在发命令之前不知道要读多少字节；长度字节是传输过程中
 *	     从机发的第一个字节。主机必须先收到它，才能决定后面再 clock 多少
 *	     个字节。所以不能“先 ioctl 读长度，再 ioctl 读数据”拆成两次事务：
 *	     那样中间会插入 STOP/新的 START，从机状态机可能已经变了，而且标准
 *	     SMBus block read 要求长度和数据在同一个事务里。
 *	  2. 内核/适配器层：i2c_smbus_xfer_emulated() 把第 2 段 i2c_msg 标成
 *	     I2C_M_RECV_LEN、len=1，意思是“先收 1 字节当长度，再按它继续收”。
 *	     适配器的 master_xfer 必须理解 I2C_M_RECV_LEN：读到首字节后把 msg->len
 *	     调整成 1 + len（再为 PEC 留位），并继续 clock。硬件做不到这一点的
 *	     适配器不会声明 I2C_FUNC_SMBUS_READ_BLOCK_DATA，用户态调用会得到
 *	     EOPNOTSUPP。i2c core 在传输后还会检查长度，超过 32 返回 -EPROTO。
 *	  3. 用户态层：因为长度未知，缓冲区必须按最大可能值准备（32 字节数据 +
 *	     长度字节 + PEC 余量），不能按“我以为的长度”分配。ioctl 成功后先读
 *	     data.block[0] 得到实际长度，再访问 block[1..len]；不要相信缓冲区里
 *	     调用前的内容。
 *
 * 和 I2C_SMBUS_I2C_BLOCK_DATA 的区别
 *	I2C block data 是“I2C 风格的块传输”，借用了 SMBus ioctl 的外壳：
 *	  - 长度由主机在调用前写进 data.block[0]，线上没有长度字节；
 *	  - 读方向就是“命令 + 读 N 字节”，写方向是“命令 + 写 N 字节”；
 *	  - 内核检查 block[0] > 32 返回 -EINVAL；传输字节数完全按主机给的来；
 *	  - 适配器声明 I2C_FUNC_SMBUS_READ_I2C_BLOCK / WRITE_I2C_BLOCK；没有时
 *	    core 可能用 byte data 模拟（不是所有路径都行）；
 *	  - 在软件模拟路径里 PEC 不适用于 I2C block（core 的 wants_pec 条件把
 *	    I2C_SMBUS_I2C_BLOCK_DATA 排除在外）。
 *	一句话：BLOCK_DATA 的长度是“从机运行时告诉你的”，I2C_BLOCK_DATA 的长度
 *	是“你调用前就定好的”。前者要求适配器支持 I2C_M_RECV_LEN，后者不要求。
 *
 *	写方向的 block 也一样：SMBus write block 由主机先发长度字节再发数据
 *	（长度 1..32）；I2C write block 不发长度字节，长度就是主机给的字节数。
 *
 * 需要的内核/硬件条件
 *	- i2c-dev + /dev/i2c-N；
 *	- 对 block read：适配器 functionality 含 I2C_FUNC_SMBUS_READ_BLOCK_DATA
 *	  （或能由 I2C 级 master_xfer 模拟且声明了对应能力）；
 *	- 一个真正实现了 SMBus block read 的从机。EEPROM 通常只支持 I2C 随机读，
 *	  不支持 SMBus block read，这时应改用 I2C block 读固定长度。
 *
 * 编译
 *	g++ -std=c++26 -O2 -Wall -Wextra -o /tmp/native_smbus_block_read smbus_block_read.cpp
 *
 * 运行
 *	/tmp/native_smbus_block_read [设备路径] [从机地址]
 *	例：/tmp/native_smbus_block_read /dev/i2c-1 0x50
 *
 * 预期输出
 *	有设备：先打印 block read 从机给出的长度和最多 32 字节数据，再打印
 *	I2C block read 固定 8 字节的结果，最后打印两种事务的对照。
 *	没有设备：打开失败时打印原因和提示，退出码 1。
 *
 * 常见坑
 *	- 对 BLOCK_DATA 读，调用前不要自己填 block[0]（内核会覆盖）；对
 *	  I2C_BLOCK_DATA 读，必须自己填 block[0]，否则长度为 0。
 *	- 缓冲区一定要 34 字节（I2C_SMBUS_BLOCK_MAX + 2），按实际长度截取使用。
 *	- 长度字节为 0 是合法的（空块），不要把它当错误。
 *	- 从机不支持 block read 时通常表现为 ENXIO/EREMOTEIO 或 EOPNOTSUPP；
 *	  先查 functionality，再决定用哪种读法。
 *	- 仍然要先 ioctl(fd, I2C_SLAVE, addr) 绑定从机地址。
 */

#include <algorithm>
#include <array>
#include <cerrno>
#include <charconv>
#include <cstdint>
#include <cstring>
#include <print>
#include <span>
#include <string>
#include <string_view>
#include <utility>

#include <fcntl.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <linux/i2c-dev.h>
#include <linux/i2c.h>

namespace {

/* RAII fd：析构 close。 */
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

/* 与前面示例相同的薄包装：填 struct i2c_smbus_ioctl_data 再 ioctl。
 * 返回 0 成功，负 errno 失败。 */
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

/* SMBus block read：size = I2C_SMBUS_BLOCK_DATA。
 *
 * 关键点：data 是一个完整的 union i2c_smbus_data（34 字节 block）。调用前
 * block[0] 无意义；内核在传输中先收从机给的长度字节，写进 block[0]，再收
 * block[1..len]。ioctl 返回 0 后：
 *	n = data.block[0]        // 从机报告的长度，0..32
 *	payload = data.block + 1 // n 个字节
 * 内核在传输后已经检查过 n > I2C_SMBUS_BLOCK_MAX 并返回 -EPROTO，这里再检查
 * 一次是为了防御和说明。
 *
 * out 是调用者提供的上限；返回实际读到的字节数（>=0）或负 errno。
 * 若从机给出的长度大于 out.size()，只拷贝 out.size() 字节但仍返回真实长度，
 * 调用者可以据此发现缓冲区不够。 */
[[nodiscard]] int read_block_data(int fd, std::uint8_t command,
				  std::span<std::uint8_t> out) noexcept
{
	union i2c_smbus_data data {};
	const int ret = smbus_access(fd, I2C_SMBUS_READ, command,
				     I2C_SMBUS_BLOCK_DATA, &data);
	if (ret < 0)
		return ret;

	const std::size_t len = data.block[0];
	if (len > I2C_SMBUS_BLOCK_MAX)
		return -EPROTO;

	const std::size_t copied = std::min(len, out.size());
	std::copy_n(&data.block[1], copied, out.begin());
	return static_cast<int>(len);
}

/* I2C block read：size = I2C_SMBUS_I2C_BLOCK_DATA。
 *
 * 和上面的差别只有两处：
 *	1. 调用前把 out.size() 写进 data.block[0] —— 长度由主机指定；
 *	2. 线上没有长度字节，从机按主机 clock 的字节数返回（或 NAK）。
 * 内核拒绝 block[0] > 32（-EINVAL）。读回的数据仍在 block[1..]。
 * 返回实际读到的字节数或负 errno。 */
[[nodiscard]] int read_i2c_block_data(int fd, std::uint8_t command,
				      std::span<std::uint8_t> out) noexcept
{
	if (out.size() > I2C_SMBUS_BLOCK_MAX)
		return -EINVAL;

	union i2c_smbus_data data {};
	data.block[0] = static_cast<std::uint8_t>(out.size());
	const int ret = smbus_access(fd, I2C_SMBUS_READ, command,
				     I2C_SMBUS_I2C_BLOCK_DATA, &data);
	if (ret < 0)
		return ret;

	std::copy_n(&data.block[1], out.size(), out.begin());
	return static_cast<int>(out.size());
}

void print_hex(std::span<const std::uint8_t> bytes)
{
	for (std::size_t i = 0; i < bytes.size(); ++i)
		std::print("{}{:02x}", i == 0 ? "" : " ", bytes[i]);
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
	if (ec != std::errc {} || ptr != digits.data() + digits.size() || value > 0x7f)
		return false;
	addr = static_cast<std::uint16_t>(value);
	return true;
}

/* 对照表：两种“块”在长度来源和线上形态上的差异。 */
struct BlockKind {
	std::string_view name;
	std::string_view length_source;
	std::string_view wire;
	std::string_view func_bit;
};

constexpr std::array<BlockKind, 2> kBlockKinds{{
	{ "BLOCK_DATA（SMBus）",
	  "从机在事务中先发 1 字节长度",
	  "S Addr+W Cmd Sr Addr+R Len D0..Dn-1 P",
	  "I2C_FUNC_SMBUS_READ_BLOCK_DATA" },
	{ "I2C_BLOCK_DATA",
	  "主机调用前写入 block[0]",
	  "S Addr+W Cmd Sr Addr+R D0..Dn-1 P",
	  "I2C_FUNC_SMBUS_READ_I2C_BLOCK" },
}};

} // namespace

int main(int argc, char **argv)
{
	std::string_view dev_path = "/dev/i2c-1";
	std::uint16_t addr = 0x50;

	for (int i = 1; i < argc; ++i) {
		std::string_view arg { argv[i] };
		if (arg == "-h" || arg == "--help") {
			std::println("用法：{} [设备路径=/dev/i2c-1] [从机地址=0x50]", argv[0]);
			return 0;
		}
		if (i == 1) {
			dev_path = arg;
		} else if (i == 2) {
			if (!parse_address(arg, addr)) {
				std::println(stderr, "从机地址格式错误（7 位，如 0x50）：{}", arg);
				return 2;
			}
		} else {
			std::println(stderr, "参数过多");
			return 2;
		}
	}

	UniqueFd fd { ::open(std::string(dev_path).c_str(), O_RDWR | O_CLOEXEC) };
	if (!fd.valid()) {
		std::println(stderr, "打开 {} 失败：{} (errno={})", dev_path,
			     std::strerror(errno), errno);
		std::println(stderr, "本机没有 I2C 适配器时这是预期结果；");
		std::println(stderr, "确认 /dev/i2c-* 存在、已加载 i2c-dev、当前用户有权限。");
		return 1;
	}

	/* 绑定从机地址（数值，不是指针）。地址被内核驱动占用会 EBUSY。 */
	if (::ioctl(fd.get(), I2C_SLAVE, static_cast<unsigned long>(addr)) < 0) {
		std::println(stderr, "ioctl(I2C_SLAVE, {:#04x}) 失败：{} (errno={})", addr,
			     std::strerror(errno), errno);
		return 1;
	}
	std::println("设备：{}，从机地址：{:#04x}", dev_path, addr);

	/* 查能力：block read 依赖 I2C_FUNC_SMBUS_READ_BLOCK_DATA（其核心是
	 * I2C_M_RECV_LEN 支持）；I2C block 依赖 I2C_FUNC_SMBUS_READ_I2C_BLOCK。 */
	unsigned long funcs = 0;
	if (::ioctl(fd.get(), I2C_FUNCS, &funcs) < 0) {
		std::println(stderr, "ioctl(I2C_FUNCS) 失败：{} (errno={})",
			     std::strerror(errno), errno);
		return 1;
	}
	const bool can_block =
		(funcs & I2C_FUNC_SMBUS_READ_BLOCK_DATA) == I2C_FUNC_SMBUS_READ_BLOCK_DATA;
	const bool can_i2c_block =
		(funcs & I2C_FUNC_SMBUS_READ_I2C_BLOCK) == I2C_FUNC_SMBUS_READ_I2C_BLOCK;

	std::println("适配器掩码 0x{:08x}：SMBus block read {}，I2C block read {}",
		     funcs, can_block ? "支持" : "不支持", can_i2c_block ? "支持" : "不支持");
	if (!can_block)
		std::println("提示：适配器未声明 SMBus block read，下面的第一步预计返回");
	if (!can_block)
		std::println("EOPNOTSUPP —— 这不是代码错误，而是硬件/驱动不支持 I2C_M_RECV_LEN。");

	/* —— 1. SMBus block read：命令 0x00 ——
	 * 缓冲区按最大规格准备：32 字节数据 + 长度 + 兼容余量（union 里已含）。
	 * 调用前 block[0] 是什么都无所谓，成功后被从机给的长度覆盖。 */
	std::println("\n1) read_block_data(command=0x00)：长度字节由从机给出");
	std::array<std::uint8_t, I2C_SMBUS_BLOCK_MAX> payload {};
	const int len = read_block_data(fd.get(), 0x00, payload);
	if (len < 0) {
		std::println("  [FAIL] {} (errno={})", std::strerror(-len), -len);
	} else {
		std::println("  [OK] 从机报告长度 {}：", len);
		std::println("  数据：");
		std::print("    ");
		print_hex(std::span<const std::uint8_t> { payload.data(),
							 static_cast<std::size_t>(len) });
		std::println("");
	}

	/* —— 2. I2C block read：命令 0x00，长度 8 ——
	 * 这里长度是主机在调用前定好的，线上没有长度字节；即使从机支持 SMBus
	 * block read，这两种调用的线上形态也不同。 */
	std::println("\n2) read_i2c_block_data(command=0x00, length=8)：长度由主机指定");
	std::array<std::uint8_t, 8> fixed {};
	const int got = read_i2c_block_data(fd.get(), 0x00, fixed);
	if (got < 0) {
		std::println("  [FAIL] {} (errno={})", std::strerror(-got), -got);
	} else {
		std::println("  [OK] 读回 {} 字节：", got);
		std::print("    ");
		print_hex(fixed);
		std::println("");
	}

	/* —— 3. 把区别列出来 ——
	 * 注意“后一步依赖前一步”的只有 BLOCK_DATA：长度是运行时从从机收到的数据，
	 * 缓冲区上限必须按协议最大值准备；I2C_BLOCK_DATA 的长度在调用前就是已知的。 */
	std::println("\n两种块读的对照：");
	for (const auto &kind : kBlockKinds) {
		std::println("  {}", kind.name);
		std::println("    长度来源：{}", kind.length_source);
		std::println("    线上形态：{}", kind.wire);
		std::println("    能力位  ：{}", kind.func_bit);
	}

	return 0;
}
