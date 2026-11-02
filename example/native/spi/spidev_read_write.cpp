/*
* spidev_read_write.cpp —— spidev 最基础的用法：open、ioctl 配置、read()/write() 各做一次事务
*
* 【SPI 是什么】
* SPI 是主机（master）驱动的同步串行总线，一根总线可以挂多个从机（slave）。
* 最常用的四根线：
*	SCLK —— 时钟。只由主机产生，频率也由主机决定，从机没有资格打时钟。
*	MOSI —— Master Out Slave In，主机在时钟边沿把 1 位送到从机。
*	MISO —— Master In Slave Out，从机在同一个时钟周期把 1 位送回主机。
*	CS   —— 片选（Chip Select / Slave Select），低电平有效，表示“现在这条总线归这个从机”。
*
* 关键点：主机每打一个时钟，MOSI 移出一位、MISO 同时移入一位，两个方向天生同时发生。
* 所以 SPI 没有“纯读”和“纯写”的线上概念，所谓只读 / 只写只是软件丢掉另一个方向的数据。
* 第二个关键点：CS 的下降沿和上升沿界定一次“事务”（transaction）。从机只在 CS 有效期间
* 认命令；CS 一释放，从机的状态机通常就复位，没做完的命令作废。本文件后半段讲的两条坑都源于此。
*
* SPI 没有统一的规范：时钟极性/相位（mode 0..3）、位序、命令格式、时序参数全部由器件手册规定，
* 主机这边只负责“按手册的时序把字节移出去/移进来”。
*
* 【spidev 是什么】
* Linux 把一块 SPI 控制器（master）上的某个片选暴露成一个字符设备 /dev/spidev<bus>.<cs>，
* 例如 /dev/spidev0.0 是 0 号控制器的 0 号片选。一个 fd 对应一个从机。
* 需要内核打开 CONFIG_SPI_SPIDEV，并在设备树 / ACPI 里给这个从机声明 compatible = "spidev"（或者
* 器件驱动本身允许用户态访问）；普通 PC 和 WSL2 没有 SPI 控制器，也就没有这个节点。
* 设备节点通常 root:spidev 0660，非 root 用户要加入 spidev 组。
*
* 【本文件的形态】
*	一次 write()  = 内核构造 1 个 spi_message，其中 1 个 spi_transfer
*		      = 拉低 CS → 打 len 个字节的时钟（MOSI 送用户给的字节）→ 拉高 CS
*	一次 read()   = 同样一次独立事务，MOSI 上送 0x00（读方向没有发送缓冲）
*	一次调用提交一次、同步完成；返回时数据已经拷回用户缓冲。
*	发送/接收缓冲由用户提供，内核在提交时把它换成 DMA 安全的内存（需要拷贝）。
*	spidev 没有异步接口、没有排队，ioctl/read/write 返回即表示这一条事务已经结束。
*
* 【C++ 形态】
*	fd 用 RAII 类 SpiFd 持有：构造函数 open，析构函数 close，禁止拷贝、允许移动；
*	缓冲区用 std::array，长度用 constexpr；参数错误和系统错误用 std::system_error 抛出。
*	ioctl/read/write 仍然直接调原生接口，没有额外封装层。
*
* 【需要的条件】
*	Linux 内核 + spidev 驱动 + 真实的 SPI 控制器和从机；设备路径从命令行参数取。
*	本机（WSL2）没有 SPI 设备，程序会在 open 处优雅报错退出，这是预期行为。
*
* 【编译】
*	g++ -std=c++26 -O2 -Wall -Wextra -o spidev_read_write spidev_read_write.cpp
*	不需要链接额外库。
*
* 【运行】
*	./spidev_read_write [/dev/spidev0.0]
*	有设备时：打印配置读回值，然后做一次写事务、一次读事务，并打印收到的字节。
*	无设备时：打印“打开 SPI 设备 ... 失败: ...”，退出码 1。
*
* 【预期输出（有设备时，数值随器件而定）】
*	打开 /dev/spidev0.0
*	mode = 0x00000000 (CPOL=0 CPHA=0)
*	bits_per_word = 8
*	speed_hz = 500000
*	--- 事务 1：write() 发送 1 字节命令 0x05 ---
*	write() 返回 1
*	--- 事务 2：read() 收 2 字节 ---
*	read() 返回 2，数据: 0x00 0xFF
*/

#include <array>
#include <cerrno>
#include <cstdint>
#include <cstdlib>
#include <print>
#include <string>
#include <system_error>
#include <utility>

#include <fcntl.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <linux/spi/spidev.h>

namespace {

constexpr const char *kDefaultDev = "/dev/spidev0.0";

/*
* RAII 持有 spidev 的文件描述符。
* - 构造时 open，失败抛 std::system_error（errno 装进 std::error_code）；
* - 析构时 close：spidev 在 release 里会恢复默认配置并释放可能被 cs_change 保持的片选；
* - 禁止拷贝（fd 是独占资源），允许移动，用 std::exchange 把源对象置成 -1；
* - get() 返回裸 fd，交给 ioctl/read/write 这些原生接口使用。
*/
class SpiFd {
public:
	explicit SpiFd(const char *path)
		: fd_{::open(path, O_RDWR)}
	{
		if (fd_ < 0)
			throw std::system_error{errno, std::system_category(),
				std::string{"打开 SPI 设备 "} + path};
	}

	~SpiFd()
	{
		if (fd_ >= 0)
			::close(fd_);
	}

	SpiFd(const SpiFd &) = delete;
	SpiFd &operator=(const SpiFd &) = delete;

	SpiFd(SpiFd &&other) noexcept
		: fd_{std::exchange(other.fd_, -1)}
	{
	}

	SpiFd &operator=(SpiFd &&other) noexcept
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
* 带类型检查的 ioctl 薄封装：request 决定参数类型 T，T 由调用方从变量推出来。
* 失败时抛 std::system_error，what() 里带上 errno 对应的英文描述。
* 返回值只用来看“成功/失败”——spidev 的配置类 ioctl 成功时返回 0。
*/
template <typename T>
void spi_ioctl(const SpiFd &spi, unsigned long request, T &arg, const char *what)
{
	if (::ioctl(spi.get(), request, &arg) < 0)
		throw std::system_error{errno, std::system_category(), what};
}

/*
* open 失败是演示里最常见的路径（本机没有 SPI 设备），所以单独拿出来：
* 打印中文原因和提示后直接退出，不让异常穿过 main 形成一串英文栈信息。
*/
SpiFd open_device(const char *path)
{
	try {
		return SpiFd{path};
	} catch (const std::system_error &e) {
		std::println(stderr, "{}", e.what());
		std::println(stderr, "提示：本机没有 SPI 设备（WSL2 恒无）时这是预期结果；");
		std::println(stderr, "      树莓派等板子上一般用 /dev/spidev0.0 或 /dev/spidev0.1。");
		std::exit(1);
	}
}

} // namespace

int main(int argc, char **argv)
{
	const char *path = (argc > 1) ? argv[1] : kDefaultDev;

	/*
	* O_RDWR 是必须的：即使只发命令不读数据，spidev 内部也要走同一条双向路径；
	* 只读打开在某些内核上会让 SPI_IOC_WR_* 失败。
	* 不加 O_NONBLOCK：spidev 的 read/write/ioctl 都是同步阻塞完成的，
	* 一次事务的时长由 len / 速率决定，没有“事件”可等。
	*/
	SpiFd spi = open_device(path);
	std::println("打开 {}", path);

	try {
		/*
		* ---------- 配置 1：SPI 模式 ----------
		*
		* SPI_MODE_x 是 CPOL（时钟空闲电平）和 CPHA（在哪个边沿采样）的组合：
		*	SPI_MODE_0 = CPOL 0, CPHA 0：空闲低，上升沿采样（最常用）
		*	SPI_MODE_1 = CPOL 0, CPHA 1：空闲低，下降沿采样
		*	SPI_MODE_2 = CPOL 1, CPHA 0：空闲高，下降沿采样
		*	SPI_MODE_3 = CPOL 1, CPHA 1：空闲高，上升沿采样（很多 flash 也用）
		* 选错 mode 的典型现象：读回全 0xFF/全 0x00，或者偶发错位。
		* 必须看从机器件手册，主机和从机不一致就完全无法通信。
		*
		* 用 SPI_IOC_WR_MODE32 而不是 SPI_IOC_WR_MODE：后者只传 8 位，
		* 放不下所有标志位（SPI_TX_OCTAL 等已经到 bit 14）。32 位版本是现在的推荐写法。
		* mode 参数里除了 CPOL/CPHA，还可以带 SPI_CS_HIGH、SPI_LSB_FIRST、SPI_3WIRE、
		* SPI_NO_CS、SPI_LOOP 等标志（见 /usr/include/linux/spi/spi.h）。
		*
		* ioctl 的第三个参数是指针，这里由 spi_ioctl 传 &arg；返回值 0 成功，-1 失败。
		*/
		std::uint32_t mode = SPI_MODE_0;
		spi_ioctl(spi, SPI_IOC_WR_MODE32, mode, "SPI_IOC_WR_MODE32");

		/* 读回：确认内核/控制器实际接受的模式（可能被驱动修正过）。 */
		std::uint32_t mode_rd = 0;
		spi_ioctl(spi, SPI_IOC_RD_MODE32, mode_rd, "SPI_IOC_RD_MODE32");
		std::println("mode = {:#010x} (CPOL={} CPHA={})", mode_rd,
			(mode_rd & SPI_CPOL) ? 1 : 0, (mode_rd & SPI_CPHA) ? 1 : 0);

		/*
		* ---------- 配置 2：每字位数 ----------
		*
		* 绝大多数器件是 8 位/字。也有 9 位（某些显示控制器）、16 位（某些 ADC）的器件，
		* 这时要设成 9 / 16；但注意：spidev 的 read()/write()/ioctl 的 len 始终以“字节”计，
		* 即 len = 字数 * 每字字节数，15 位字长这种不是整数字节的情况要靠 ioctl 传结构体处理。
		* 0 表示“用设备默认值”。
		*/
		std::uint8_t bpw = 8;
		spi_ioctl(spi, SPI_IOC_WR_BITS_PER_WORD, bpw, "SPI_IOC_WR_BITS_PER_WORD");
		std::uint8_t bpw_rd = 0;
		spi_ioctl(spi, SPI_IOC_RD_BITS_PER_WORD, bpw_rd, "SPI_IOC_RD_BITS_PER_WORD");
		std::println("bits_per_word = {}", bpw_rd);

		/*
		* ---------- 配置 3：时钟频率 ----------
		*
		* 这是“期望上限”，单位 Hz。控制器只能用自己的分频器凑，实际频率一般向下取整，
		* 所以上面写 500000，读回来可能是 488281 之类的值 —— 以读回值为准。
		* 频率太高会出现数据错误（线长、从机能力、电平转换器都有限制），
		* 调试阶段一律先降到 100 kHz ~ 1 MHz。
		* 同理，SPI_IOC_WR_LSB_FIRST 可以改位序，默认 MSB first，本文件不涉及。
		*/
		std::uint32_t speed = 500000;
		spi_ioctl(spi, SPI_IOC_WR_MAX_SPEED_HZ, speed, "SPI_IOC_WR_MAX_SPEED_HZ");
		std::uint32_t speed_rd = 0;
		spi_ioctl(spi, SPI_IOC_RD_MAX_SPEED_HZ, speed_rd, "SPI_IOC_RD_MAX_SPEED_HZ");
		std::println("speed_hz = {}", speed_rd);

		/*
		* ---------- 事务 1：write() ----------
		*
		* write(fd, buf, len) 在 spidev 里等价于：
		*	struct spi_transfer t = { .tx_buf = buf, .len = len };
		*	struct spi_message m;  spi_message_init_with_transfers(&m, &t, 1);
		*	spi_sync(spi, &m);
		* 线上动作：CS 拉低 → 打 len 个字节的时钟（MOSI 输出 buf 的内容，MISO 收到的位被丢弃）
		*	 → CS 拉高。整条序列只产生一次 CS 有效窗口。
		*
		* 返回值：成功时是写入的字节数（spidev 要么返回完整 len，要么返回 -1，
		* 不会像 pipe 那样只写一部分）；失败返回 -1 并设置 errno。
		*
		* 这里的 0x05 只是示例命令（很多 SPI flash 用它读状态寄存器）；
		* 你实际接什么器件，就查它的手册第一条命令是什么。
		*/
		std::println("--- 事务 1：write() 发送 1 字节命令 0x05 ---");
		std::array<std::uint8_t, 1> cmd{0x05};
		ssize_t n = ::write(spi.get(), cmd.data(), cmd.size());
		if (n < 0)
			throw std::system_error{errno, std::system_category(), "write"};
		if (n != static_cast<ssize_t>(cmd.size())) {
			/* 理论上不会发生；发生了说明驱动行为异常，直接报出来而不是继续。 */
			std::println(stderr, "write() 短写: 只写了 {} 字节", n);
			return 1;
		}
		std::println("write() 返回 {}", n);

		/*
		* ---------- 事务 2：read() ----------
		*
		* read(fd, buf, len) 是另一次完全独立的事务：
		* CS 拉低 → 打 len 个字节的时钟 → CS 拉高。
		* 读方向没有发送缓冲，所以 MOSI 上输出的是 0x00（内核送全 0 的 tx 缓冲）。
		*
		* 【为什么“先写命令再读”不能拆成两次调用】
		* 1. CS 在两次调用之间被抬高又拉低。对从机来说这是两条无关的事务，
		*    第一条“发命令”已经结束，第二条开始时它并没有收到任何命令。
		* 2. write() 发出的命令字节不会被内核记住，read() 不会把它重发一遍；
		*    read() 在 MOSI 上送出的是 0x00。
		* 3. 两次调用之间进程可能被调度走，同一控制器上的其它进程也可能插进来
		*    使用总线（spidev 不提供总线独占，除非你自己 flock）。
		*    即使两次调用紧挨着，中间的空隙长度也不受你控制。
		*
		* 所以下面这段 read() 的输出对真实器件没有意义（尤其在 CMD 后需要数据的器件上），
		* 它只是用来演示“每次 read()/write() 都是独立一次片选”这个事实。
		* 正确做法见随后的 spi_ioc_message.cpp（一条 ioctl 里放多段），
		* 或 spidev_cs_change.cpp（用 cs_change 跨 ioctl 保持片选）。
		*/
		std::println("--- 事务 2：read() 收 2 字节（这是独立事务，结果无实际意义）---");
		std::array<std::uint8_t, 2> resp{0, 0};
		n = ::read(spi.get(), resp.data(), resp.size());
		if (n < 0)
			throw std::system_error{errno, std::system_category(), "read"};
		std::println("read() 返回 {}，数据: 0x{:02X} 0x{:02X}", n, resp[0], resp[1]);
	} catch (const std::system_error &e) {
		std::println(stderr, "SPI 操作失败: {}", e.what());
		return 1;
	}

	/* 离开作用域时 SpiFd 析构 → close → 内核释放片选并恢复默认配置。 */
	return 0;
}
