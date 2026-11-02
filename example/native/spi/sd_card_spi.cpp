/*
* sd_card_spi.cpp —— SD 卡 SPI 模式读一个块：CMD17 + 轮询 R1 + 等数据令牌 + 512 字节
*
* 【它解决什么问题 / 演示什么】
* SD 卡在 SPI 模式下就是挂在 SPI 上的一个从机，协议全部是“发命令 → 等响应 → 等数据”，
* 而且**后一步做什么取决于前一步收到了什么**：
*	- 发完 CMD17 后，要逐字节轮询，直到收到一个 bit7=0 的 R1；
*	- R1 为 0 才继续等数据令牌 0xFE；R1 非 0 说明命令被拒，应直接放弃；
*	- 等到 0xFE 之后才知道“现在开始 512 字节数据”，然后收 512 字节 + 2 字节 CRC16。
* 因为每一段的长度和是否继续都依赖上一段收到的字节，无法用一次 SPI_IOC_MESSAGE(n)
* 预先写好所有段；只能由用户态一步一步驱动。
* 又因为整条读块过程必须处在同一次 CS 有效期内（SD 卡释放 CS 会终止当前命令），
* 这里用 spidev_cs_change.cpp 里的技巧：每个中间 ioctl 的最后一段 cs_change=1，
* 保持 CS 有效；最后一个 ioctl 用 cs_change=0 收尾。
*
* 【SD SPI 模式的帧格式（本文件用到的部分）】
*	命令帧固定 6 字节：
*		byte0 = 0x40 | cmd（cmd 6 位，bit7=0 表示命令，bit6=1 表示传输）
*		byte1..4 = 32 位大端参数
*		byte5 = CRC7 + 停止位。SPI 模式下只有 CMD0（0x95）和 CMD8（0x87）要求 CRC 正确，
*			进入 SPI 模式后卡不再检查 CRC，惯例填 0x01。
*	R1 响应 1 字节，最高位为 0 表示有响应：
*		bit0 idle，bit1 erase reset，bit2 illegal command，bit3 CRC error，
*		bit4 sequence error，bit5 address error，bit6 parameter error。
*	发完命令后卡可能延迟 1~8 字节（NCR）才给出 R1，所以要逐字节轮询，MOSI 一直送 0xFF。
*	读单块（CMD17）成功后：可能再等若干字节，出现数据令牌 0xFE，
*	随后是 512 字节数据 + 2 字节 CRC16；出错时出现错误令牌 0x0X（高 4 位 0，低 4 位非 0）。
*
* 【初始化流程（本文件做了简化）】
*	上电后 ≥74 个时钟（CS 为高）→ CMD0 进入 SPI 模式 → CMD8 探测电压/版本
*	→ 循环 CMD55 + ACMD41 直到卡退出 idle → CMD58 读 OCR（判断 SDHC/SDXC）
*	→ CMD16 设块长 512（SDHC 会忽略）。
*	简化之处（注释里也分别标了）：
*	- 74 时钟：spidev 每条消息都会拉低 CS，无法在“未选中”状态下打时钟，
*	  这里直接发 10 个 0xFF；多数卡能接受，严格做法要用 SPI_NO_CS 或专用驱动（未核对）。
*	- 没有处理全部错误分支（erase reset、电压不匹配等），只做最基本的成功路径。
*	- CMD8 失败（v1 卡）时继续用 HCS=0 的 ACMD41，其它差异不处理。
*	- 512 字节后面的 CRC16 读出来直接丢弃，不校验。
*	- 不支持 CMD18 多块读、写、擦除、SDIO。
*	- 超时轮询次数是经验值（逐字节轮询也很慢，生产代码会按 8 字节一批轮询）。
*
* 【形态】
*	每次 ioctl 只交 1 段（spi_ioc_transfer 数组长度 1），对应 1 个 spi_transfer；
*	真实的一条读块事务由十几次 ioctl 拼成，靠中间各次 cs_change=1 保持 CS、
*	最后一次 cs_change=0 收尾；每次提交同步完成，返回即这一段已经打完；
*	所有缓冲（命令帧、块数据）都由用户提供，内核在提交时换成 DMA 安全内存。
*	C++ 侧用一个 SdCard 类持有 RAII 的 fd（SpiFd），速率、卡类型作为成员状态。
*
* 【需要的条件】
*	Linux + spidev + 真正的 SD 卡槽（SPI 模式接法：CS/MOSI/MISO/SCLK，3.3V，
*	MISO 和 CS 建议上拉）。WSL2 没有 SPI 设备，open 失败后报错退出，属预期。
*
* 【编译】
*	g++ -std=c++26 -O2 -Wall -Wextra -o sd_card_spi sd_card_spi.cpp
*
* 【运行】
*	./sd_card_spi [/dev/spidev0.0]
*
* 【预期输出（插卡时）】
*	打开 /dev/spidev0.0
*	CMD0 完成，R1=0x01（idle）
*	CMD8 回显 00 00 01 AA
*	ACMD41 完成，卡退出 idle
*	OCR=0x40FF8000，CCS=1（SDHC/SDXC，按块地址寻址）
*	读 LBA 0：前 16 字节 = EB 3C 90 2E 2E 2E ...
*	（第 510/511 字节一般是 55 AA，即 MBR 签名）
*
* 【常见坑】
*	- CS 一旦释放，正在进行的读块立即作废；本文件靠 cs_change=1 保持。
*	- 发命令、收响应时 MOSI 必须送 0xFF（SD 卡靠这个产生时钟），送 0x00 也能出时钟，
*	  但 0xFF 是协议惯例，避免某些卡把 0x00 当命令。
*	- CMD17 参数：SDHC/SDXC 是块地址（LBA），SDSC 是字节地址，别混。
*	- 初始化阶段速度必须低（≤400 kHz），初始化完成后才能提速（本文件 20 MHz）。
*	- 每次 ioctl 之间可能被调度，但 CS 保持低；别的进程若同时用这块总线会破坏协议。
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
constexpr std::uint32_t kInitSpeed = 400'000;	/* SD 规范：初始化阶段 ≤ 400 kHz */
/*
* 读数据阶段，SD SPI 一般可到 25 MHz。控制器做不到时会自行向下取整；
* 个别驱动对超出能力的 speed_hz 直接返回 EINVAL（未核对），所以换板子时先降频试。
*/
constexpr std::uint32_t kFastSpeed = 20'000'000;
constexpr std::size_t kBlockSize = 512;
constexpr std::uint8_t kDataToken = 0xFE;
constexpr std::uint8_t kDummy = 0xFF;		/* 空闲时 MOSI 发送的字节 */

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
	SpiFd(SpiFd &&other) noexcept : fd_{std::exchange(other.fd_, -1)} {}

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
		std::println(stderr, "      读 SD 卡需要把卡槽接到 SPI 控制器并让内核暴露 spidev 节点。");
		std::exit(1);
	}
}

/*
* SD 卡 SPI 模式的驱动对象：持有 fd 和当前速率，
* 每次 ioctl 都是一段传输，hold_cs 决定这段之后片选是否保持。
*/
class SdCard {
public:
	explicit SdCard(SpiFd spi) : spi_{std::move(spi)} {}

	/* 初始化：上电时钟、CMD0、CMD8、ACMD41、CMD58、CMD16。成功返回，失败抛异常。 */
	void init()
	{
		/*
		* 上电后至少 74 个时钟且 CS 为高。spidev 的每条消息都会拉低 CS，
		* 这里只能在选中状态下发 10 个 0xFF（80 个时钟）作为简化；
		* 严格实现需要让 CS 保持高电平（SPI_NO_CS 或专用驱动），未核对各卡行为。
		*/
		std::array<std::uint8_t, 10> clocks{};
		clocks.fill(kDummy);
		xfer(clocks, {}, false);

		/*
		* CMD0：GO_IDLE_STATE，让卡进入 idle 并切换到 SPI 模式。
		* CRC 必须正确（0x95），因为这时卡还没进入 SPI 模式、仍在检查 CRC。
		* 成功时 R1 = 0x01（idle）。已经处于 SPI 模式的卡可能回 0x00，也接受。
		*/
		std::uint8_t r1 = 0xFF;
		if (!command(0, 0, 0x95, r1, false))
			throw std::runtime_error{"CMD0 无响应"};
		if (r1 != 0x01 && r1 != 0x00)
			throw std::runtime_error{"CMD0 失败，R1 非 idle"};
		std::println("CMD0 完成，R1=0x{:02X}（idle）", r1);

		/*
		* CMD8：SEND_IF_COND，参数 0x1AA = 电压 2.7~3.6V + 检查图样 0xAA。
		* 卡支持时 R1=0x01，并在 R1 之后紧接着回 4 字节原样回显 00 00 01 AA。
		* CRC 必须正确（0x87）。不支持的老卡（SD v1）返回 illegal command。
		* 注意：R1 之后的 4 字节必须还在同一次 CS 内收，所以 command() 用 hold=true，
		* 收完回显再手动释放。
		*/
		bool v2 = command(8, 0x1AA, 0x87, r1, true);
		if (v2 && r1 == 0x01) {
			std::array<std::uint8_t, 4> echo{};
			xfer({}, echo, true);
			const std::array<std::uint8_t, 4> expect{0x00, 0x00, 0x01, 0xAA};
			if (echo != expect)
				throw std::runtime_error{"CMD8 回显不匹配，电压或卡不支持"};
			std::println("CMD8 回显 {:02X} {:02X} {:02X} {:02X}",
				echo[0], echo[1], echo[2], echo[3]);
		} else {
			/* v1 卡：不认 CMD8，后面 ACMD41 的参数里不能置 HCS 位。 */
			std::println("CMD8 无有效响应（R1=0x{:02X}），按 SD v1 卡处理", r1);
		}
		release_cs();

		/*
		* ACMD41：SD_SEND_OP_COND，让卡完成上电初始化并退出 idle。
		* ACMD 不是独立命令，必须先发 CMD55（APP_CMD）再发 ACMD41。
		* 参数 HCS（bit30）=1 表示主机支持高容量卡；v1 卡不能置。
		* 卡忙时 R1 保持 0x01，需要反复重试（这里最多 200 次、每次隔 10 ms）。
		* 注意：这是简化实现，没有区分 CMD55 本身的错误位。
		*/
		const std::uint32_t hcs = v2 ? 0x40000000u : 0u;
		bool ready = false;
		for (int attempt = 0; attempt < 200; ++attempt) {
			if (!command(55, 0, 0x01, r1, false))
				continue;	/* 没有响应，重试 */
			if (!command(41, hcs, 0x01, r1, false))
				continue;
			if (r1 == 0x00) {
				ready = true;
				break;
			}
			::usleep(10000);
		}
		if (!ready)
			throw std::runtime_error{"ACMD41 超时，卡未退出 idle"};
		std::println("ACMD41 完成，卡退出 idle");

		/*
		* CMD58：READ_OCR，读 4 字节 OCR。
		* CCS（bit30，OCR 首字节的 bit6）=1 表示 SDHC/SDXC，用块地址寻址；
		* =0 表示 SDSC，用字节地址寻址。这个区别直接影响 CMD17 的参数。
		* 同样：4 字节 OCR 必须在同一次 CS 内读，command() hold=true。
		*/
		if (command(58, 0, 0x01, r1, true) && r1 == 0x00) {
			std::array<std::uint8_t, 4> ocr{};
			xfer({}, ocr, true);
			const std::uint32_t ocr_val = (static_cast<std::uint32_t>(ocr[0]) << 24) |
				(static_cast<std::uint32_t>(ocr[1]) << 16) |
				(static_cast<std::uint32_t>(ocr[2]) << 8) |
				static_cast<std::uint32_t>(ocr[3]);
			high_capacity_ = (ocr[0] & 0x40) != 0;
			std::println("OCR={:#010x}，CCS={}（{}）", ocr_val,
				high_capacity_ ? 1 : 0,
				high_capacity_ ? "SDHC/SDXC，按块地址寻址" : "SDSC，按字节地址寻址");
		}
		release_cs();

		/*
		* CMD16：SET_BLOCKLEN，块长设 512。SDHC/SDXC 固定 512，会忽略这条命令；
		* SDSC 需要它。简化：失败只提示不终止。
		*/
		if (command(16, static_cast<std::uint32_t>(kBlockSize), 0x01, r1, false) && r1 == 0x00)
			std::println("CMD16 设置块长 {} 字节", kBlockSize);

		/*
		* 初始化完成，切成高速。速率是每段 transfer 里的 speed_hz 覆盖值，
		* 通过 xfer() 统一填进结构体，所以改成员即可对后续所有传输生效。
		*/
		speed_ = kFastSpeed;
		std::println("初始化完成，切换到 {} Hz", speed_);
	}

	/*
	* 读一个 512 字节块。
	* 整个过程 CS 必须保持有效：CMD17 → R1 → 数据令牌 → 512 字节 → 2 字节 CRC。
	* 中间每个 ioctl 都设 cs_change=1，最后一个（读 CRC）之后释放。
	*/
	void read_block(std::uint32_t lba, std::span<std::uint8_t, kBlockSize> buf)
	{
		/*
		* CMD17：READ_SINGLE_BLOCK。
		* 参数寻址方式取决于卡类型：
		*	SDHC/SDXC —— 直接给块号 LBA；
		*	SDSC      —— 给字节地址 LBA * 512。
		* CRC 填 0x01（SPI 模式下不检查）。
		* command(..., hold=true)：R1 之后还要继续等数据令牌，CS 不能松开。
		*/
		const std::uint32_t addr = high_capacity_ ? lba : lba * kBlockSize;
		std::uint8_t r1 = 0xFF;
		if (!command(17, addr, 0x01, r1, true)) {
			release_cs();
			throw std::runtime_error{"CMD17 无响应"};
		}
		if (r1 != 0x00) {
			/*
			* R1 非 0：命令被拒（地址越界、CRC、参数错等）。
			* 此时卡不会再发数据，直接释放 CS 放弃，不要把后续字节当数据读。
			* 这正是“必须看 R1 才能决定下一步”的例子。
			*/
			release_cs();
			throw std::runtime_error{"CMD17 被拒绝"};
		}

		/*
		* 等数据令牌 0xFE。卡在 R1 之后还要一段时间准备数据（NAC），
		* 期间 MISO 上都是 0xFF。逐字节发送并检查：
		*	0xFF —— 还没准备好，继续；
		*	0xFE —— 数据开始，跳出；
		*	其它 —— 错误令牌（0x0X），放弃。
		* 上限 65536 次是经验值（20 MHz 下约 26 ms 时钟；有些卡更慢，可加大）。
		* 生产代码会把轮询改成每次 8 字节以减少 ioctl 次数，这里为讲清楚逐字节写。
		*/
		bool got_token = false;
		for (std::uint32_t i = 0; i < 65536; ++i) {
			const std::uint8_t b = xfer_byte(kDummy, true);
			if (b == kDataToken) {
				got_token = true;
				break;
			}
			if (b != kDummy) {
				release_cs();
				throw std::runtime_error{"收到数据错误令牌"};
			}
		}
		if (!got_token) {
			release_cs();
			throw std::runtime_error{"等待数据令牌超时"};
		}

		/*
		* 收 512 字节。tx 为空，MOSI 送 0x00，MISO 上的数据进 buf。
		* 这一段仍然 cs_change=1，因为后面还有 2 字节 CRC 要在同一次事务里读完。
		* len = 512 字节，远小于 spidev 默认 bufsiz 4096。
		*/
		xfer({}, buf, true);

		/*
		* 收 2 字节 CRC16。本示例不校验（简化），但必须把这 2 个字节的时钟打完，
		* 否则卡的状态机不认为块传输结束。
		* 这个 ioctl 用 cs_change=0，消息结束后 CS 释放，整条读块事务结束。
		*/
		std::array<std::uint8_t, 2> crc{};
		xfer({}, crc, false);

		std::println("读块完成：LBA={}，CRC16=0x{:02X}{:02X}（本示例不校验）",
			lba, crc[0], crc[1]);
	}

private:
	/*
	* 一次 ioctl = 一个 spi_transfer。
	* hold_cs=true 时把 cs_change 置 1：内核在这次消息结束后不拉高 CS，
	* 下一次 ioctl 继续在同一条事务里（详见 spidev_cs_change.cpp）。
	*/
	int xfer(std::span<const std::uint8_t> tx, std::span<std::uint8_t> rx, bool hold_cs)
	{
		struct spi_ioc_transfer tr{};
		const std::size_t len = tx.empty() ? rx.size() : tx.size();

		if (!tx.empty() && !rx.empty() && tx.size() != rx.size())
			throw std::invalid_argument{"tx/rx 长度必须相同"};
		if (len == 0)
			throw std::invalid_argument{"长度为 0 的段没有意义"};

		tr.tx_buf = tx.empty() ? 0 : to_user_ptr(tx.data());
		tr.rx_buf = rx.empty() ? 0 : to_user_ptr(rx.data());
		tr.len = static_cast<std::uint32_t>(len);
		tr.speed_hz = speed_;
		tr.bits_per_word = 8;	/* SD 卡固定 8 位/字 */
		tr.cs_change = hold_cs ? 1 : 0;

		const int ret = ::ioctl(spi_.get(), SPI_IOC_MESSAGE(1), &tr);
		if (ret < 0)
			throw std::system_error{errno, std::system_category(), "SPI_IOC_MESSAGE(1)"};
		return ret;
	}

	/* 全双工发 1 字节、收 1 字节：轮询 R1 / 等令牌都用它。 */
	std::uint8_t xfer_byte(std::uint8_t byte, bool hold_cs)
	{
		std::uint8_t got = kDummy;
		std::array<std::uint8_t, 1> out{byte};
		xfer(out, std::span<std::uint8_t>{&got, 1}, hold_cs);
		return got;
	}

	/*
	* 发送 6 字节命令帧并等 R1。
	* 返回 false 表示 8 字节内没等到 bit7=0 的响应；
	* hold=false 时无论成败都会释放 CS，hold=true 时由调用者负责收尾。
	*/
	bool command(std::uint8_t cmd, std::uint32_t arg, std::uint8_t crc,
		std::uint8_t &r1, bool hold)
	{
		std::array<std::uint8_t, 6> frame{};
		frame[0] = static_cast<std::uint8_t>(0x40 | (cmd & 0x3F));
		frame[1] = static_cast<std::uint8_t>(arg >> 24);
		frame[2] = static_cast<std::uint8_t>(arg >> 16);
		frame[3] = static_cast<std::uint8_t>(arg >> 8);
		frame[4] = static_cast<std::uint8_t>(arg);
		frame[5] = crc;

		/* 发命令时保持 CS：卡要求从命令到响应之间 CS 一直有效。 */
		xfer(frame, {}, true);

		/* 轮询 R1：最多 8 个字节（NCR 上限），每个字节都是一次 ioctl，CS 保持。 */
		bool got = false;
		for (int i = 0; i < 8; ++i) {
			const std::uint8_t b = xfer_byte(kDummy, true);
			if ((b & 0x80) == 0) {
				r1 = b;
				got = true;
				break;
			}
		}

		if (!hold)
			release_cs();
		return got;
	}

	/* 发 1 个 0xFF 并让 CS 在消息结束后释放（cs_change=0）。 */
	void release_cs()
	{
		xfer_byte(kDummy, false);
	}

	SpiFd spi_;
	std::uint32_t speed_ = kInitSpeed;
	bool high_capacity_ = false;
};

} // namespace

int main(int argc, char **argv)
{
	const char *path = (argc > 1) ? argv[1] : kDefaultDev;
	SpiFd spi = open_device(path);
	std::println("打开 {}", path);

	try {
		/*
		* 先配置 fd 默认值：mode 0、8 位/字、初始化速率。
		* 每段 transfer 里还会用 speed_hz 覆盖速率，所以这里主要是给初始值。
		*/
		std::uint32_t mode = SPI_MODE_0;
		spi_ioctl(spi, SPI_IOC_WR_MODE32, mode, "SPI_IOC_WR_MODE32");
		std::uint8_t bpw = 8;
		spi_ioctl(spi, SPI_IOC_WR_BITS_PER_WORD, bpw, "SPI_IOC_WR_BITS_PER_WORD");
		std::uint32_t speed = kInitSpeed;
		spi_ioctl(spi, SPI_IOC_WR_MAX_SPEED_HZ, speed, "SPI_IOC_WR_MAX_SPEED_HZ");

		SdCard card{std::move(spi)};
		card.init();

		std::array<std::uint8_t, kBlockSize> block{};
		card.read_block(0, block);

		/* 打印块头 16 字节：MBR 一般是 EB 3C 90 ...，第 510/511 字节是 55 AA。 */
		std::print("LBA 0 前 16 字节:");
		for (std::size_t i = 0; i < 16; ++i)
			std::print(" {:02X}", block[i]);
		std::println("");
		std::println("块尾 2 字节（MBR 签名应为 55 AA）: {:02X} {:02X}",
			block[kBlockSize - 2], block[kBlockSize - 1]);
	} catch (const std::system_error &e) {
		std::println(stderr, "SPI 操作失败: {}", e.what());
		return 1;
	} catch (const std::runtime_error &e) {
		std::println(stderr, "SD 卡操作失败: {}", e.what());
		return 1;
	}

	return 0;
}
