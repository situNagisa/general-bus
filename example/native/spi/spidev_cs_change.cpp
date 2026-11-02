/*
* spidev_cs_change.cpp —— 用 cs_change 控制片选：跨多次 ioctl 组成一条长事务
*
* 【它解决什么问题】
* spi_ioc_transfer.cs_change 让用户态决定“这一段之后片选怎么办”。
* 最常用的玩法是：把最后一段的 cs_change 设成 1，于是这次 ioctl 返回时 CS 仍然是有效的，
* 紧接着的下一次 ioctl 继续打时钟，从器件角度看前后几次 ioctl 是同一条事务。
* 这样就能处理“后一步的内容依赖前一步收到的数据”的协议（SD 卡、复杂 flash 命令等），
* 而这些协议没法用一次 SPI_IOC_MESSAGE 把全部段预先写好。
*
* 【spidev.h 原文与真实语义的差异（最容易混淆的地方）】
* spidev.h 里写：
*	@cs_change: True to deselect device before starting the next transfer.
* 这句话只对“消息中间的非最后一段”成立：本段结束后先释放 CS，下一段再重新拉低，
* 于是同一条 spi_message 被切成多个小的 CS 事务。
* 但对“消息的最后一段”，内核（spi.c 的 spi_transfer_one_message）的处理是相反的：
*	最后一段 cs_change=1 → 不在消息结束时释放 CS（keep_cs = true）。
* 也就是说：
*	中间段 cs_change=1 = 段之间“临时释放片选”；
*	最后段 cs_change=1 = 消息结束后“保持片选有效”。
* 一个字段、两种相反的方向，这是 spidev 最常见的坑，务必按位置区分。
* （内核源码细节按语义描述，本机没有内核源码树，标“未核对”。）
*
* 【形态】
*	一次 ioctl 仍然是一个 spi_message（这里是 1 段或 n 段）；
*	区别只在 CS 的收尾：cs_change=1 时内核不拉高 CS，fd 保持选中状态；
*	下一次 ioctl 在这个已选中的状态下继续，器件看到的是连续时钟流；
*	内存、返回值、同步完成的语义都不变。
*
* 【需要的条件】
*	控制器必须支持“按段控制片选”。GPIO 片选一般可以；纯硬件自动片选的控制器
*	可能只在消息开始/结束动 CS，或者每次消息开始都无条件重新拉低一次（产生毛刺）。
*	不同驱动差异很大，换板子前先示波器/逻辑分析仪看一眼 CS 波形。
*
* 【编译】
*	g++ -std=c++26 -O2 -Wall -Wextra -o spidev_cs_change spidev_cs_change.cpp
*
* 【运行】
*	./spidev_cs_change [/dev/spidev0.0]
*
* 【预期输出（有设备时）】
*	打开 /dev/spidev0.0
*	--- 演示 1：两次 ioctl 组成一条长事务 ---
*	第 1 次 ioctl：发命令 0x05，cs_change=1，返回 1；此刻 CS 仍为低
*	第 2 次 ioctl：读 1 字节，cs_change=0，返回 1，CS 释放
*	读到 0x00（示例器件状态，随器件而定）
*	--- 演示 2：一条消息中间段 cs_change=1，把消息切成两段 ---
*	ioctl 返回 5 字节（期望 2 + 1 + 2 = 5）
*	线上波形：CS↓ 01 02 CS↑ CS↓ 03 04 05 CS↑
*
* 【常见坑】
*	- 最后一段 cs_change=1 之后，如果程序忘了用 cs_change=0 的消息收尾，
*	  CS 会一直保持低电平，器件一直处于选中状态。
*	- 保持片选期间别的进程若也用同一个 spidev，会插进你的“长事务”里。
*	  spidev 不做总线互斥，需要的话自己对 fd 做 flock(LOCK_EX)。
*	- 中间段 cs_change=1 之后如果 delay_usecs 非 0，延时发生在释放 CS 之前。
*	- 不是所有控制器都实现 word/segment 级 CS 控制，失败时不会有专门错误码，
*	  现象是器件行为不对，只能靠波形判断。
*/

#include <array>
#include <cerrno>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <print>
#include <span>
#include <stdexcept>
#include <string>
#include <system_error>
#include <utility>

#include <fcntl.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <linux/spi/spidev.h>

namespace {

constexpr const char *kDefaultDev = "/dev/spidev0.0";

inline std::uint64_t to_user_ptr(const void *p) noexcept
{
	return static_cast<std::uint64_t>(reinterpret_cast<std::uintptr_t>(p));
}

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
		std::println(stderr, "提示：本机没有 SPI 设备（WSL2 恒无）时这是预期结果。");
		std::exit(1);
	}
}

/*
* 单段传输的最薄封装：一送一收，cs_change 由调用者决定。
* 这里故意用 SPI_IOC_MESSAGE(1) 而不是 write()/read()，因为只有 spi_ioc_transfer
* 才带 cs_change 字段，read()/write() 没有地方表达“这次别释放片选”。
*/
int spi_xfer(const SpiFd &spi, std::span<const std::uint8_t> tx,
	std::span<std::uint8_t> rx, std::uint8_t cs_change)
{
	struct spi_ioc_transfer tr{};
	const std::size_t len = tx.empty() ? rx.size() : tx.size();

	if (!tx.empty() && !rx.empty() && tx.size() != rx.size())
		throw std::invalid_argument{"tx/rx 长度必须相同（len 是一个共用字段）"};
	if (len == 0)
		throw std::invalid_argument{"长度为 0 的段没有意义"};

	/* tx 或 rx 为空时对应指针写 0：MOSI 送 0 / 丢弃 MISO。 */
	tr.tx_buf = tx.empty() ? 0 : to_user_ptr(tx.data());
	tr.rx_buf = rx.empty() ? 0 : to_user_ptr(rx.data());
	tr.len = static_cast<std::uint32_t>(len);
	tr.cs_change = cs_change;

	int ret = ::ioctl(spi.get(), SPI_IOC_MESSAGE(1), &tr);
	if (ret < 0)
		throw std::system_error{errno, std::system_category(), "SPI_IOC_MESSAGE(1)"};
	return ret;
}

} // namespace

int main(int argc, char **argv)
{
	const char *path = (argc > 1) ? argv[1] : kDefaultDev;
	SpiFd spi = open_device(path);
	std::println("打开 {}", path);

	try {
		std::uint32_t mode = SPI_MODE_0;
		spi_ioctl(spi, SPI_IOC_WR_MODE32, mode, "SPI_IOC_WR_MODE32");
		std::uint8_t bpw = 8;
		spi_ioctl(spi, SPI_IOC_WR_BITS_PER_WORD, bpw, "SPI_IOC_WR_BITS_PER_WORD");
		std::uint32_t speed = 500000;
		spi_ioctl(spi, SPI_IOC_WR_MAX_SPEED_HZ, speed, "SPI_IOC_WR_MAX_SPEED_HZ");

		/*
		* ---------- 演示 1：两次 ioctl 组成一条长事务 ----------
		*
		* 场景（以 SPI flash 为例）：0x05 是“读状态寄存器”，命令之后必须紧接着
		* 输出 1 个字节的时钟才能拿到状态值，中间释放 CS 就算另一条事务了。
		*
		* 第 1 次：只发命令，最后一段 cs_change=1。
		* 内核在消息结束时不拉高 CS，ioctl 返回后 CS 仍然是低电平。
		*/
		std::println("--- 演示 1：两次 ioctl 组成一条长事务 ---");
		std::array<std::uint8_t, 1> cmd{0x05};
		int n = spi_xfer(spi, cmd, {}, 1);
		std::println("第 1 次 ioctl：发命令 0x05，cs_change=1，返回 {}；此刻 CS 仍为低", n);

		/*
		* 第 2 次：读 1 字节，cs_change=0。因为 CS 还低着，器件仍然认为
		* 自己在处理 0x05 命令；这次读到的就是状态寄存器的值。
		* 消息结束时 cs_change=0 → 内核正常拉高 CS，结束整条长事务。
		*
		* 如果第 1 次 cs_change 写 0，第 1 次返回时 CS 已经拉高，
		* 第 2 次是一条新事务，器件不会输出状态值（常见现象是读到 0xFF）。
		*/
		std::array<std::uint8_t, 1> status{};
		n = spi_xfer(spi, {}, status, 0);
		std::println("第 2 次 ioctl：读 1 字节，cs_change=0，返回 {}，CS 释放", n);
		std::println("读到 0x{:02X}（示例器件状态，随器件而定）", status[0]);

		/*
		* ---------- 演示 2：一条消息中间段 cs_change=1，把消息切成两段 ----------
		*
		* 3 段：2 字节 + 1 字节 + 2 字节。
		*	段 0 cs_change=1 → 段 0 打完后释放一次 CS，段 1 开始时重新拉低；
		*	段 1 cs_change=0 → 正常继续，不释放；
		*	段 2（最后一段）cs_change=0 → 消息结束释放。
		* 线上看到的是两次 CS 事务：[01 02] 和 [03 04 05]。
		* 适用场景：一个 ioctl 里先给一个“锁存/启动”命令，释放 CS 让器件准备，
		* 再发后续数据；这样仍然是同步完成，一次调用内做完。
		*
		* 注意和演示 1 对比：同样是 cs_change=1，
		* 放在中间段是“释放”，放在最后段是“保持”，方向正好相反。
		*/
		std::println("--- 演示 2：一条消息中间段 cs_change=1，把消息切成两段 ---");
		std::array<std::uint8_t, 2> seg0{0x01, 0x02};
		std::array<std::uint8_t, 1> seg1{0x03};
		std::array<std::uint8_t, 2> seg2{0x04, 0x05};
		std::array<struct spi_ioc_transfer, 3> msg{};

		msg[0].tx_buf = to_user_ptr(seg0.data());
		msg[0].len = static_cast<std::uint32_t>(seg0.size());
		msg[0].cs_change = 1;	/* 非最后一段：本段结束后释放 CS */

		msg[1].tx_buf = to_user_ptr(seg1.data());
		msg[1].len = static_cast<std::uint32_t>(seg1.size());

		msg[2].tx_buf = to_user_ptr(seg2.data());
		msg[2].len = static_cast<std::uint32_t>(seg2.size());

		int ret = ::ioctl(spi.get(), SPI_IOC_MESSAGE(3), msg.data());
		if (ret < 0)
			throw std::system_error{errno, std::system_category(), "SPI_IOC_MESSAGE(3)"};
		std::println("ioctl 返回 {} 字节（期望 2 + 1 + 2 = 5）", ret);
		std::println("线上波形：CS↓ 01 02 CS↑ CS↓ 03 04 05 CS↑");

		/*
		* 【控制器 / 驱动差异（实际调试时最耗时间的部分）】
		* - 大多数支持 GPIO 片选的控制器可以精确控制每段的 CS；
		* - 一些控制器只在消息开始拉低、消息结束拉高，中间段 cs_change 被忽略；
		* - 一些控制器在下一条消息开始时无条件重新拉低 CS，即使它已经是低的，
		*   这会形成一个很窄的高脉冲，对边沿敏感的器件（如 SD 卡）等同于结束事务；
		* - SPI_NO_CS（片选常有效）配置下 cs_change 没有意义；
		* - 内核只会检查字段合法性，不保证控制器真的实现了这个语义，
		*   失败通常没有专门错误码，只能看波形和器件行为。
		* 以上具体实现差异未在本机核对（WSL2 无 SPI 控制器）。
		*/

		/*
		* 长事务期间 CS 一直被占用；spidev 不做总线互斥。
		* 多进程共享同一块总线时，建议在 open 之后 flock(spi.get(), LOCK_EX)，
		* 用完或进程退出时自动释放。示例只在注释里说明，不在这里加锁，
		* 以免掩盖 cs_change 本身的语义。
		*/
	} catch (const std::system_error &e) {
		std::println(stderr, "SPI 操作失败: {}", e.what());
		return 1;
	}

	/*
	* 收尾：正常情况下上面每条路径都以 cs_change=0 结束，CS 已经释放。
	* 若程序中途异常退出，SpiFd 析构的 close 会让 spidev 驱动清理片选状态
	* （驱动是否一定拉高 CS 视实现而定，未核对）。所以宁可显式发一次 cs_change=0 的消息，
	* 也不要依赖 close 的副作用。
	*/
	return 0;
}
