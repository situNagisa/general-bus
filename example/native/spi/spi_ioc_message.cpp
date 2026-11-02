/*
* spi_ioc_message.cpp —— SPI_IOC_MESSAGE(n)：把 n 段传输放进同一次片选里
*
* 【它解决什么问题】
* read()/write() 每次都是一条独立的 message，各自拉低、拉高 CS。
* 可真实器件几乎都是“先发命令字节，再收/发若干字节”的同一个事务，
* 中间不能释放 CS。SPI_IOC_MESSAGE(n) 就是让用户态在一次 ioctl 里描述 n 段，
* 内核把它们拼成一个 spi_message，在一次 CS 有效期内依次执行。
*
* 【形态】
*	一次 ioctl  = 1 个 spi_message = n 个 spi_transfer 段（数组长度由 ioctl 编号里的
*		       size 字段反推，所以 SPI_IOC_MESSAGE(n) 的 n 必须是编译期常量）；
*	线上单元    = 一次 CS 事务，包含所有段的所有字节；段与段之间默认不释放 CS；
*	提交一次，同步完成：ioctl 返回时所有段都已完成，或者整体失败；
*	内存由用户提供：每段的 tx_buf / rx_buf 指向用户缓冲，内核在提交时把这些
*		      用户指针换成 DMA 安全的内核内存（通常会拷贝一次）。
*
* 【本文件的演示】
*	1. 读 SPI flash 的 JEDEC ID：发 0x9F，再收 3 字节（厂商/类型/容量）；
*	2. 一段全双工：tx、rx 都给出，同一个时钟周期既发又收；
*	3. 打印返回值、段大小、失败路径。
*
* 【需要的条件】
*	Linux + spidev + 真实器件。读 JEDEC ID 需要接一颗 SPI NOR flash
*	（W25Q 系列、GD25Q 系列等），其它器件请按手册改命令字节。
*	本机（WSL2）没有 SPI 设备，open 失败后打印错误退出，这是预期行为。
*
* 【编译】
*	g++ -std=c++26 -O2 -Wall -Wextra -o spi_ioc_message spi_ioc_message.cpp
*
* 【运行】
*	./spi_ioc_message [/dev/spidev0.0]
*
* 【预期输出（接 W25Q128 时）】
*	打开 /dev/spidev0.0
*	sizeof(struct spi_ioc_transfer) = 32
*	ioctl 返回 6 字节（期望 1 + 3 + 2 = 6）
*	JEDEC ID: 0xEF 0x40 0x18 (厂商 EF，类型 40，容量 18 = 16 MiB)
*	全双工: 发出 AA 55，收到 0xFF 0xFF（MISO 空闲电平，随器件而定）
*
* 【常见坑】
*	- 段数 n 和数组实际长度不一致：内核按 ioctl 编号里的 size 算 n，
*	  多算会读到数组外的内存，少算会漏发段（未核对具体错误码）。
*	- 总字节数超过 spidev 的 bufsiz（模块参数，默认 4096）会失败，见文件末尾注释。
*	- tx_buf/rx_buf 是 __u64，用户态要先把指针转成 uintptr_t 再转，不能直接赋指针。
*	- 结构体必须整体清零：内核会读所有字段，垃圾值（比如 bits_per_word=200）会 EINVAL。
*/

#include <array>
#include <cerrno>
#include <cstddef>
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

/* 本示例固定 3 段：命令 1 字节 + 收 ID 3 字节 + 全双工 2 字节。 */
constexpr std::size_t kNrXfers = 3;
constexpr std::size_t kCmdLen = 1;
constexpr std::size_t kIdLen = 3;
constexpr std::size_t kFdxLen = 2;
constexpr int kTotalLen = static_cast<int>(kCmdLen + kIdLen + kFdxLen);

/*
* 编译期检查：SPI_MSGSIZE(n) 在 n 段总大小超过 ioctl size 字段能表示的范围时返回 0。
* _IOC_SIZEBITS 在本机是 14，所以 n * 32 < 16384，最多 511 段。
* 这里 n=3 远小于上限，assert 只是把约束写进代码。
*/
static_assert(SPI_MSGSIZE(kNrXfers) != 0, "段数太多，ioctl 编号放不下");

/* 用户指针 → __u64。spi_ioc_transfer 的缓冲字段在用户态必须用整数表示。 */
inline std::uint64_t to_user_ptr(const void *p) noexcept
{
	return static_cast<std::uint64_t>(reinterpret_cast<std::uintptr_t>(p));
}

/* 与 spidev_read_write.cpp 相同的 RAII fd：构造 open，析构 close，可移动不可拷贝。 */
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

template <typename T>
void spi_ioctl(const SpiFd &spi, unsigned long request, T &arg, const char *what)
{
	if (::ioctl(spi.get(), request, &arg) < 0)
		throw std::system_error{errno, std::system_category(), what};
}

SpiFd open_device(const char *path)
{
	try {
		return SpiFd{path};
	} catch (const std::system_error &e) {
		std::println(stderr, "{}", e.what());
		std::println(stderr, "提示：本机没有 SPI 设备（WSL2 恒无）时这是预期结果；");
		std::println(stderr, "      读 JEDEC ID 需要接一颗 SPI NOR flash。");
		std::exit(1);
	}
}

} // namespace

int main(int argc, char **argv)
{
	const char *path = (argc > 1) ? argv[1] : kDefaultDev;
	SpiFd spi = open_device(path);
	std::println("打开 {}", path);
	std::println("sizeof(struct spi_ioc_transfer) = {}", sizeof(struct spi_ioc_transfer));

	try {
		/*
		* 先用设备默认配置：mode 0、8 位/字、500 kHz。
		* 这三个 ioctl 设置的是该 fd 后续所有传输的默认值，
		* 单段里的 speed_hz / bits_per_word 字段可以在本段临时覆盖（0 = 用默认）。
		*/
		std::uint32_t mode = SPI_MODE_0;
		spi_ioctl(spi, SPI_IOC_WR_MODE32, mode, "SPI_IOC_WR_MODE32");
		std::uint8_t bpw = 8;
		spi_ioctl(spi, SPI_IOC_WR_BITS_PER_WORD, bpw, "SPI_IOC_WR_BITS_PER_WORD");
		std::uint32_t speed = 500000;
		spi_ioctl(spi, SPI_IOC_WR_MAX_SPEED_HZ, speed, "SPI_IOC_WR_MAX_SPEED_HZ");

		std::array<std::uint8_t, kCmdLen> cmd{0x9F};	/* JEDEC ID 命令 */
		std::array<std::uint8_t, kIdLen> id{};
		std::array<std::uint8_t, kFdxLen> fdx_tx{0xAA, 0x55};
		std::array<std::uint8_t, kFdxLen> fdx_rx{};

		/*
		* struct spi_ioc_transfer 每个字段的含义（以本机
		* /usr/include/linux/spi/spidev.h 为准，sizeof = 32 字节）：
		*
		* __u64 tx_buf            本段的发送缓冲地址（用户指针转成整数）。
		*                         NULL 表示本段不发有意义的数据，MOSI 上送 0。
		* __u64 rx_buf            本段的接收缓冲地址。NULL 表示丢弃 MISO 上的位。
		*                         两个都非 NULL 就是全双工。
		* __u32 len               本段字节数，tx 和 rx 共用同一个长度
		*                         （len = 字数 × 每字字节数）。tx_buf 和 rx_buf
		*                         至少有一个非 NULL，否则这段没有意义。
		* __u32 speed_hz          本段临时覆盖的设备速率（Hz），0 = 用设备默认。
		*                         用于“配置寄存器慢速、读数据高速”这类器件。
		* __u16 delay_usecs       本段最后一位打完之后、释放片选（或开始下一段）之前
		*                         的延时，单位微秒。手册里 tSLCH / tSHSL 这类参数
		*                         就是靠它满足。最大值 65535。
		* __u8  bits_per_word     本段临时覆盖的字长，0 = 用设备默认。
		*                         改这个字段通常同时要保证 len 是整数字节。
		* __u8  cs_change         改变本段之后片选的行为，语义反直觉，见
		*                         spidev_cs_change.cpp。这里全部 0 = 保持到消息结束。
		* __u8  tx_nbits          发送方向用几根数据线：0 = 用默认（单线），
		*                         1 = 单线，2 = Dual，4 = Quad，8 = Octal。
		*                         双/四线 SPI（Dual/Quad I/O）才用得到。
		*                         SPI_NBITS_* 宏在内核内部头文件里，用户态头文件没有导出，
		*                         所以这里按数字写。
		* __u8  rx_nbits          接收方向用几根线，取值同上。
		* __u8  word_delay_usecs  同一个 transfer 内部，字与字之间的延时（微秒）。
		*                         只有控制器显式支持才生效，否则被静默忽略
		*                         （spidev.h 原文 "silently ignored"）。类型是 __u8，
		*                         最大 255 微秒，别拿它当毫秒级延时用。
		* __u8  pad               填充，必须为 0。
		*
		* 整个数组用 {} 值初始化，保证所有未写的字段都是 0 —— 这是 spidev 要求的。
		*/
		std::array<struct spi_ioc_transfer, kNrXfers> tr{};

		/* 段 0：只发 1 字节命令 0x9F。rx_buf 为 NULL，收到的位丢弃。 */
		tr[0].tx_buf = to_user_ptr(cmd.data());
		tr[0].len = static_cast<std::uint32_t>(cmd.size());

		/*
		* 段 1：只收 3 字节。tx_buf 为 NULL，内核在 MOSI 上送 0x00。
		* 关键：这一段和段 0 在同一个 spi_message 里，CS 一直是低的，
		* flash 认为“0x9F 命令之后紧跟着的 3 个字节”就是它的 ID 输出。
		*/
		tr[1].rx_buf = to_user_ptr(id.data());
		tr[1].len = static_cast<std::uint32_t>(id.size());

		/*
		* 段 2：全双工。tx、rx 都给：每打一个时钟，MOSI 送 fdx_tx 的 1 位、
		* MISO 同时收 1 位进 fdx_rx。收到的内容与发出的内容无关，
		* 取决于器件此刻在 MISO 上输出什么。
		*/
		tr[2].tx_buf = to_user_ptr(fdx_tx.data());
		tr[2].rx_buf = to_user_ptr(fdx_rx.data());
		tr[2].len = static_cast<std::uint32_t>(fdx_tx.size());

		/*
		* 提交。SPI_IOC_MESSAGE(3) 展开成一个 _IOW('k', 0, char[96]) 的 ioctl 编号：
		* n 决定编号里的 size 字段，内核用 size / sizeof(spi_ioc_transfer) 还原 n，
		* 所以 n 必须是编译期常量，且必须和数组实际长度一致。
		*
		* 返回值：
		*	>= 0 —— 所有段传输的总字节数，等于各段 len 之和；
		*	-1   —— 失败，errno 给出原因（EINVAL 参数错、EMSGSIZE 超过 bufsiz、
		*		EIO 控制器错误等）。
		* 注意“整体失败”：任何一段出错，整条 message 返回错误，
		* 但已经打出去的时钟没法撤销，器件状态可能已经变了，不能只重试失败的那段。
		*/
		int ret = ::ioctl(spi.get(), SPI_IOC_MESSAGE(kNrXfers), tr.data());
		if (ret < 0)
			throw std::system_error{errno, std::system_category(), "SPI_IOC_MESSAGE(3)"};

		std::println("ioctl 返回 {} 字节（期望 {}）", ret, kTotalLen);
		if (ret != kTotalLen)
			std::println(stderr, "警告：返回值与各段 len 之和不一致，驱动行为异常");

		std::println("JEDEC ID: 0x{:02X} 0x{:02X} 0x{:02X}（厂商 {:02X}，类型 {:02X}，容量码 {:02X}）",
			id[0], id[1], id[2], id[0], id[1], id[2]);
		if (id[0] == 0xFF && id[1] == 0xFF && id[2] == 0xFF)
			std::println(stderr, "提示：全 0xFF 通常表示 MISO 悬空、器件没接或片选不对");
		else if (id[0] == 0x00 && id[1] == 0x00 && id[2] == 0x00)
			std::println(stderr, "提示：全 0x00 通常表示器件未上电或 MISO 被拉低");

		std::println("全双工: 发出 {:02X} {:02X}，收到 {:02X} {:02X}",
			fdx_tx[0], fdx_tx[1], fdx_rx[0], fdx_rx[1]);
	} catch (const std::system_error &e) {
		std::println(stderr, "SPI 操作失败: {}", e.what());
		return 1;
	}

	/*
	* 【spidev 的 bufsiz 限制】
	* spidev 驱动有一个模块参数 bufsiz，默认 4096，表示“一条 message 允许的总字节数”。
	* 各段 len 之和超过它时 ioctl 失败（-EMSGSIZE，未核对具体错误码）。
	* 想读更大的块（比如一次读 8 KiB flash）只能：
	*	1. 拆成多次 ioctl —— 但每次都会释放/重新拉低 CS，见 spidev_cs_change.cpp；
	*	2. 加载驱动时改参数（modprobe spidev bufsiz=65536），影响所有 spidev 用户；
	*	3. 换用器件自己的内核驱动，走 spi_sync()，没有这个用户态限制。
	* 另外注意 bufsiz 只是上限，实际还受控制器 DMA 能力限制。
	*/
	return 0;
}
