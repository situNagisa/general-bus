/*
* 不在本机编译：这是 STM32 裸机寄存器级代码，直接操作 QUADSPI 外设的
* 寄存器，依赖 CMSIS 设备头（stm32h7xx.h / stm32f7xx.h）和 arm-none-eabi
* 工具链，只能在目标 MCU 上交叉编译；本机（x86-64 WSL2）既没有 CMSIS 头，
* 也没有 QUADSPI 外设。寄存器/位名对着 ST 官方 CMSIS 头核对过
* （cmsis-device-h7 的 stm32h743xx.h、cmsis-device-f7 的 stm32f767xx.h），
* 字段编码对着 ST HAL（stm32h7xx_hal_qspi.c）核对过。
*
* 一、这是什么
* QUADSPI 是 STM32F7/H7 上的专用 SPI flash 控制器。它不像普通 SPI 那样
* 只会“发一段、收一段”，而是把一条 flash 命令拆成硬件写死的五段：
*
*	指令 → 地址 → 交替字节 → dummy → 数据
*
* 每段用几条线（1/2/4）可以单独配置，所以能直接产生
* 1-1-4、1-4-4 这类 flash 时序。三种工作模式：
* - 间接模式（indirect read/write）：CPU 通过 FIFO 读写，地址、长度任意，
*   适合读/写任意数据，也是写 flash 驱动最先要打通的一条路；
* - 自动轮询模式（automatic polling）：硬件周期性读状态寄存器，等擦写完成；
* - 内存映射模式（memory mapped）：flash 映射进地址空间，取指取数像访问
*   内存一样（XIP）。
* 本示例只演示“间接读”。
*
* 二、解决什么问题 / 什么时候用
* 用普通 SPI 读 flash 的数据，要自己拼“指令 + 地址 + dummy + 数据”
* 四段 spi_transfer，而且每段线数不同还得靠控制器支持。QUADSPI 把这些
* 阶段变成寄存器里的一行配置：写 CCR 描述形状，写 DLR 给长度，写 AR 启动，
* 然后从 FIFO 里取数据。你只需要轮询 SR。
*
* 三、形态
* - 一次传输 = 一次片选有效 = 一条 flash 命令 = 一次“写 AR”。
* - 五个阶段顺序由硬件规定，不能重排；不想发的阶段把对应 MODE 字段写 00。
* - 提交 1 次（写 AR），完成 1 次（SR.TCF 置位）。中间数据阶段会反复出现
*   “FIFO 水位达到阈值”的 FTF，但它只是“有数据可取”的进度标志，
*   不是完成事件；整条命令只有 TCF 一个完成标志，它在最后一段数据也
*   传完之后才置起。
* - 待读的数据由调用者提供缓冲区；外设内部有 32 字节 FIFO，DR 是 32 位
*   端口：32 位读一次弹出 4 字节，8 位读一次弹出 1 字节。数据搬运完全
*   由 CPU 轮询完成（也可以开 DMA，但要按型号确认，见 init() 里的注释）。
*
* 四、需要什么
* - STM32F7/H7 等带 QUADSPI 的型号（F4/G4/L4 没有 QUADSPI）。
* - flash 的 CLK/CS/IO0..IO3 接到 QUADSPI 引脚，GPIO 复用功能已配好
*   （H7 上是 AF9/AF10，具体引脚查数据手册，未核对）。
* - RCC 已给 QUADSPI 供时钟（H7 上是 D2CCIP2R 里的 QUADSPISEL，F7 上是
*   DCKCFGR2 里的 QSPISEL，具体位名未核对）。
*
* 五、怎么编译、怎么运行
*	arm-none-eabi-g++ -std=c++26 -O2 -mcpu=cortex-m7 -mthumb \
*		-mfpu=fpv5-d16 -mfloat-abi=hard -Wall -Wextra \
*		-I<cmsis>/Include -I<cmsis>/Device/ST/STM32H7xx/Include \
*		-DSTM32H743xx -c quadspi_stm32.cpp
*   arm-none-eabi-g++ 对 -std=c++26 的支持看 GCC 版本，不支持就退回
*   -std=c++23（本文件只用到 C++23 的 std::expected）。裸机上用
*   <expected>/<span> 需要一套完整 libstdc++；很多工程用 -ffreestanding
*   加精简 STL，或者把返回值换成 int + errno。这里按“有完整标准库”写，
*   重点在寄存器时序。
*
* 运行时：在 main() 里初始化时钟/GPIO 后调 quadspi_demo()，接上 UART
* 打印。预期读到 W25Q128 的 JEDEC ID（ef 40 18），再从地址 0 读出
* 256 字节。
*/

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <expected>
#include <span>
#include <utility>

/* CMSIS 设备头。F7 工程换成 stm32f7xx.h，寄存器名相同。 */
#include "stm32h7xx.h"

namespace qspi {

/* 一个阶段在 CCR 里的 2 位编码。注意它不是“线数”本身：
* 00 = 不发这个阶段，01 = 1 线，10 = 2 线，11 = 4 线。
*/
enum class lines : std::uint32_t {
	none = 0b00,
	one  = 0b01,
	two  = 0b10,
	four = 0b11,
};

/* ADSIZE / ABSIZE：地址或交替字节的宽度编码，00/01/10/11 对应
* 8/16/24/32 位。NOR flash 常见 24 位（3 字节）；32 MiB 以上的片子
* 用 32 位（4 字节）地址。
*/
enum class width : std::uint32_t {
	b8  = 0b00,
	b16 = 0b01,
	b24 = 0b10,
	b32 = 0b11,
};

/* FMODE[1:0]：功能模式，决定写 AR 之后硬件干什么。
* 间接读就是 01。
*/
enum class fmode : std::uint32_t {
	indirect_write = 0b00,
	indirect_read  = 0b01,
	automatic_poll = 0b10,
	memory_mapped  = 0b11,
};

enum class error {
	timeout,
	busy,
	bad_config,
};

/*
* 一条读命令的“形状”。五个阶段全在这里，但顺序不用写 —— 硬件固定
* 是 指令 → 地址 → 交替字节 → dummy → 数据，调用者只能选择跳不跳。
*/
struct read_cmd {
	std::uint8_t instruction;	/* CCR.INSTRUCTION[7:0]：8 位操作码 */
	lines imode = lines::one;	/* 指令阶段线数（IMODE） */
	lines admode = lines::none;	/* 地址阶段线数（ADMODE），none = 无地址 */
	width asize = width::b24;	/* 地址宽度（ADSIZE） */
	lines abmode = lines::none;	/* 交替字节阶段线数（ABMODE） */
	width absize = width::b8;	/* 交替字节宽度（ABSIZE） */
	std::uint32_t alternate = 0;	/* ABR：交替字节/模式位的值 */
	std::uint8_t dummy = 0;		/* DCYC[4:0]：dummy 时钟周期数，0..31 */
	lines dmode = lines::one;	/* 数据阶段线数（DMODE） */
};

/* 0x6B = Fast Read Quad Output（W25Q 系列）：
* 1 线指令、1 线 24 位地址、8 个 dummy 时钟、4 线数据。
* 这里 DCYC 直接就是“时钟周期数”，不用像 spi-mem 那样按字节换算。
*/
inline constexpr read_cmd fast_read_quad_out{
	.instruction = 0x6b,
	.imode = lines::one,
	.admode = lines::one,
	.asize = width::b24,
	.abmode = lines::none,
	.dummy = 8,
	.dmode = lines::four,
};

/* 0xEB = Fast Read Quad I/O：1 线指令、4 线地址、4 线数据。
* 这种命令在地址后面通常还要跟模式位（例如 W25Q 的 M7..M0），
* 就用“交替字节”阶段发：ABMODE=4 线，ABSIZE=8 位，值写进 ABR。
* 0x20 是 W25Q 的“持续读”模式位，具体取值和 dummy 周期数要看
* 片子 datasheet（未核对；这里只作示例）。
*/
inline constexpr read_cmd fast_read_quad_io{
	.instruction = 0xeb,
	.imode = lines::one,
	.admode = lines::four,
	.asize = width::b24,
	.abmode = lines::four,
	.absize = width::b8,
	.alternate = 0x20,
	.dummy = 6,
	.dmode = lines::four,
};

class controller {
public:
	/*
	* 初始化。prescaler 只能取偶数（最低位恒 0），实际 QSPI 时钟 =
	* f_AHB / (prescaler + 1)。
	* flash_size_log2 = log2(flash 容量字节数)；
	* FSIZE 字段 = log2(容量) - 1（HAL 头注释：FlashSize+1 就是地址位数）。
	* 16 MiB = 2^24 → FSIZE = 23 = 0x17。
	*/
	[[nodiscard]] static std::expected<void, error>
	init(std::uint32_t prescaler, std::uint32_t flash_size_log2) noexcept
	{
		if ((prescaler & 1u) != 0u || prescaler > 255u ||
				flash_size_log2 < 16u || flash_size_log2 > 32u)
			return std::unexpected(error::bad_config);

		/* 改配置前先关外设。EN 清零要几个时钟周期才生效，
		* 等它真的变成 0 再改 DCR。时钟没开时这里会死等 ——
		* 调试时先确认 RCC 已经把 QSPI 时钟打开。 */
		QUADSPI->CR = 0u;
		while ((QUADSPI->CR & QUADSPI_CR_EN) != 0u) {
		}
		/* 关掉 EN 后还要等 BUSY 落下，才能安全改 DCR/CR（HAL 也是先等
		* BUSY 清零再写配置）。 */
		while ((QUADSPI->SR & QUADSPI_SR_BUSY) != 0u) {
		}

		/* DCR：
		*   CKMODE = 0  时钟模式 0（CPOL=0、CPHA=0，flash 的默认模式）
		*   CSHT = 3    片选拉高后至少空闲 4 个周期。CSHT 编码是
		*               “周期数 - 1”，HAL 的 QSPI_CS_HIGH_TIME_4_CYCLE
		*               就是这个值。
		*   FSIZE = 23  2^(FSIZE+1) = 16 MiB。FSIZE 只影响地址范围，
		*               写错不报错，内存映射模式下会让地址回绕。
		*/
		QUADSPI->DCR = ((flash_size_log2 - 1u) << QUADSPI_DCR_FSIZE_Pos) |
				(0u << QUADSPI_DCR_CKMODE_Pos) |
				(3u << QUADSPI_DCR_CSHT_Pos);

		/* CR：
		*   PRESCALER  时钟分频（偶数）
		*   FTHRES = 3 FIFO 阈值编码是“字节数 - 1”，3 表示 4 字节
		*              （1 个字）就置 FTF。HAL 写的是
		*              (FifoThreshold - 1) << QUADSPI_CR_FTHRES_Pos。
		*   TCEN = 0   不开超时计数器（TOF 用的那个），这里不用
		*   TCIE = 0   不开 TCF 中断（本示例纯轮询）
		*   EN = 1     使能外设
		* 注意 H7 上 CR.DMAEN 在 CMSIS 里标的是“保留，仅为软件兼容”；
		* 要用 DMA 搬 DR 得按型号确认（未核对）。
		*/
		QUADSPI->CR = (prescaler << QUADSPI_CR_PRESCALER_Pos) |
				(3u << QUADSPI_CR_FTHRES_Pos) |
				QUADSPI_CR_EN;

		return {};
	}

	/*
	* 间接读：从 flash 的 addr 读 dst.size() 字节。
	*
	* 数据阶段同时等 FTF 和 TCF：
	*   - FTF 表示 FIFO 水位到了阈值（这里是 4 字节），可以按 32 位读 DR；
	*   - 末尾不足一个字、或者整条命令只有 3 个字节时，FTF 永远不会再置起，
	*     只能等 TCF，然后按 8 位读 DR 把尾巴取出来（HAL_QSPI_Receive()
	*     也是 FT|TC 一起等、逐字节读 DR 的）。
	* 读 DR 会把数据从 FIFO 弹出，所以只能在两种标志之一置起时读。
	*/
	[[nodiscard]] static std::expected<void, error>
	read(const read_cmd& cmd, std::uint32_t addr,
			std::span<std::byte> dst) noexcept
	{
		if (dst.empty())
			return {};
		if (cmd.dummy > 31u)	/* DCYC 只有 5 位 */
			return std::unexpected(error::bad_config);

		/* 外设还在忙时不能开新传输（上一次没结束、或正在被 ABORT 打断）。 */
		if ((QUADSPI->SR & QUADSPI_SR_BUSY) != 0u)
			return std::unexpected(error::busy);

		/* 失败路径兜底：如果下面提前 return（超时），析构函数会发起
		* ABORT，把外设拉回空闲，否则下一次传输永远被 BUSY 挡住。 */
		abort_guard guard;

		start(cmd, addr, dst.size());

		for (std::size_t off = 0; off < dst.size();) {
			/* FTF 或 TCF 任一置起，FIFO 里就有东西可读 */
			if (!wait_set(QUADSPI_SR_FTF | QUADSPI_SR_TCF))
				return std::unexpected(error::timeout);

			const std::size_t left = dst.size() - off;
			if (left >= 4u &&
					(QUADSPI->SR & QUADSPI_SR_FTF) != 0u) {
				/* 满一个字：32 位读 DR，一次取 4 字节 */
				const std::uint32_t word = QUADSPI->DR;
				std::memcpy(dst.data() + off, &word, 4);
				off += 4;
			} else {
				/* 尾巴（或只有 3 字节的 ID）：8 位读 DR，弹 1 字节 */
				dst[off++] = static_cast<std::byte>(
					*reinterpret_cast<volatile std::uint8_t *>(
						&QUADSPI->DR));
			}
		}

		/* TCF：整条命令（五个阶段全部）结束的唯一完成标志。
		* 数据虽然取完了，但外设可能还在收尾（最后几个时钟、CS 拉高），
		* 所以要等它；然后写 FCR.CTCF 清掉，留给下一次用。 */
		if (!wait_set(QUADSPI_SR_TCF))
			return std::unexpected(error::timeout);
		QUADSPI->FCR = QUADSPI_FCR_CTCF;

		guard.dismiss();
		return {};
	}

	/*
	* 读 3 字节 JEDEC ID（0x9F）。形状上只有 指令 + 数据 两个阶段
	* （ADMODE/ABMODE/DCYC 全为 0）；3 个字节不是 4 的倍数，
	* read() 里会走“等 TCF + 8 位读 DR”的那条尾巴分支。
	*/
	[[nodiscard]] static std::expected<std::array<std::byte, 3>, error>
	read_jedec_id() noexcept
	{
		static constexpr read_cmd rdid{
			.instruction = 0x9f,
			.imode = lines::one,
			.admode = lines::none,
			.abmode = lines::none,
			.dummy = 0,
			.dmode = lines::one,
		};

		std::array<std::byte, 3> id{};
		const auto r = read(rdid, 0u, id);
		if (!r)
			return std::unexpected(r.error());
		return id;
	}

	/*
	* 中止当前传输：置 ABORT，等硬件自己清零，再把可能残留的标志清掉。
	* 超时路径和下一次传输前的清理都可能用到。
	*/
	static void abort() noexcept
	{
		QUADSPI->CR |= QUADSPI_CR_ABORT;
		for (std::uint32_t spin = 0; spin < 100'000u; ++spin) {
			if ((QUADSPI->CR & QUADSPI_CR_ABORT) == 0u)
				break;
		}
		QUADSPI->FCR = QUADSPI_FCR_CTEF | QUADSPI_FCR_CTCF |
			QUADSPI_FCR_CSMF | QUADSPI_FCR_CTOF;
	}

private:
	/*
	* 等某个 SR 状态位置起，带自旋上限，避免 flash 没接好时死等。
	* 真实工程里应该用 SysTick/超时计数，而不是固定循环次数。
	*/
	static bool wait_set(std::uint32_t mask) noexcept
	{
		for (std::uint32_t spin = 0; spin < 1'000'000u; ++spin) {
			if ((QUADSPI->SR & mask) != 0u)
				return true;
		}
		return false;
	}

	/*
	* 启动一次间接读，顺序不能乱：
	*   清标志 → 写 DLR → 写 ABR → 写 CCR → 写 AR（开跑）。
	* CCR 描述形状，必须在 AR 之前写；AR 是“扣扳机”的那一下。
	*/
	static void start(const read_cmd& cmd, std::uint32_t addr,
			std::size_t nbytes) noexcept
	{
		/* 先清残留标志。不清的话，上一次的 TCF 还在，后面的等待会
		* 立刻“成功”，然后读到脏数据。这是最常见的坑之一。
		* 注意 FTF 没有对应的清除位（FCR 里只有 CTEF/CTCF/CSMF/CTOF），
		* 它是 FIFO 水位的状态位，随 FIFO 满/空自然变化。 */
		QUADSPI->FCR = QUADSPI_FCR_CTEF | QUADSPI_FCR_CTCF |
			QUADSPI_FCR_CSMF | QUADSPI_FCR_CTOF;

		/* DLR[31:0]：要读的字节数 - 1。 */
		QUADSPI->DLR = static_cast<std::uint32_t>(nbytes - 1u);

		/* ABR：交替字节内容（ABMODE = none 时不会被发出）。 */
		QUADSPI->ABR = cmd.alternate;

		/* CCR：五个阶段的形状一次写全。
		*   INSTRUCTION[7:0]  操作码
		*   IMODE[9:8]        指令阶段线数
		*   ADMODE[11:10]     地址阶段线数
		*   ADSIZE[13:12]     地址宽度
		*   ABMODE[15:14]     交替字节阶段线数
		*   ABSIZE[17:16]     交替字节宽度
		*   DCYC[22:18]       dummy 时钟周期数
		*   DMODE[25:24]      数据阶段线数
		*   FMODE[27:26]      01 = 间接读
		*   SIOO[28]          内存映射模式下指令只发一次，这里无关
		*   FRCM[29]          自由运行时钟（H7 有，F7 没有），不用
		*   DHHC[30]          DDR hold 半周期，不用
		*   DDRM[31]          DDR 模式，不用
		*/
		QUADSPI->CCR = ccr_value(cmd);

		/* AR：flash 内部地址。写这个寄存器 = 片选拉低、开始传输。
		* BUSY=1 时写 AR 行为未定义，调用方已经检查过。 */
		QUADSPI->AR = addr;
	}

	[[nodiscard]] static constexpr std::uint32_t
	ccr_value(const read_cmd& cmd) noexcept
	{
		return (static_cast<std::uint32_t>(cmd.instruction)
			<< QUADSPI_CCR_INSTRUCTION_Pos) |
		(static_cast<std::uint32_t>(cmd.imode)
			<< QUADSPI_CCR_IMODE_Pos) |
		(static_cast<std::uint32_t>(cmd.admode)
			<< QUADSPI_CCR_ADMODE_Pos) |
		(static_cast<std::uint32_t>(cmd.asize)
			<< QUADSPI_CCR_ADSIZE_Pos) |
		(static_cast<std::uint32_t>(cmd.abmode)
			<< QUADSPI_CCR_ABMODE_Pos) |
		(static_cast<std::uint32_t>(cmd.absize)
			<< QUADSPI_CCR_ABSIZE_Pos) |
		(static_cast<std::uint32_t>(cmd.dummy)
			<< QUADSPI_CCR_DCYC_Pos) |
		(static_cast<std::uint32_t>(cmd.dmode)
			<< QUADSPI_CCR_DMODE_Pos) |
		(static_cast<std::uint32_t>(fmode::indirect_read)
			<< QUADSPI_CCR_FMODE_Pos);
	}

	/*
	* RAII：正常路径 dismiss()，异常/超时路径上析构时自动 ABORT。
	*/
	class abort_guard {
	public:
		abort_guard() noexcept = default;
		~abort_guard()
		{
			if (!done_)
				controller::abort();
		}
		abort_guard(const abort_guard &) = delete;
		abort_guard &operator=(const abort_guard &) = delete;

		void dismiss() noexcept { done_ = true; }

	private:
		bool done_ = false;
	};
};

} // namespace qspi

/*
* 演示入口：初始化 → 读 JEDEC ID → 从 0 地址读一页。
* 返回值 < 0 表示哪一步失败，方便在调试器里看；打印请自行接 UART。
*/
extern "C" int quadspi_demo() noexcept
{
	using namespace qspi;

	/* 假设 AHB 200 MHz：prescaler=3 → QSPI 时钟 50 MHz；
	* flash 16 MiB → log2 = 24。 */
	if (!controller::init(3u, 24u))
		return -1;

	const auto id = controller::read_jedec_id();
	if (!id)
		return -2;

	std::array<std::byte, 256> page{};
	if (!controller::read(fast_read_quad_out, 0u, page))
		return -3;

	/* 到这里 id->data() 是 3 字节 JEDEC ID，page 是 256 字节数据。
	* 想试 0xEB（4 线地址 + 模式位），把上面的 read 换成：
	*   controller::read(fast_read_quad_io, 0u, page)
	*/
	(void)id;
	(void)page;
	return 0;
}
