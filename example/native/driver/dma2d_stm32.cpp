/* 不在本机编译：MCU 寄存器级代码（STM32F4 DMA2D/Chrom-ART），需要 arm-none-eabi-g++、CMSIS 头文件和一块 STM32F429 板子。
*
* 这是驱动实现 link 的手段，link 层看不到它。
* link 层只看到“把 width×height 的一块像素从 A 搬到 B”；FGOR/OOR 的行间隔、
* NLR 的行列数、CR 的启动位和 TC 标志，都在驱动里，link 层一概不见。
*
* ============================================================================
* 这个接口是什么
* ============================================================================
* DMA2D（Chrom-ART Accelerator）是 STM32F4 里的 2D 搬运/混合加速器。它不经过
* CPU，直接在内存之间按矩形搬像素，能做：
*   - M2M：内存到内存，同格式拷贝；
*   - M2M_PFC：拷贝同时做像素格式转换（RGB565 -> ARGB8888 等）；
*   - M2M_BLEND：前景/背景两层混合，带 alpha；
*   - R2M：用固定颜色填充矩形。
*
* 这里只用 M2M 模式演示“多行、行间隔固定”的跨步拷贝：从一张大图里抠出
* 一个 width×height 的窗口，放到另一张大图的指定位置。两边的“每行像素数”
* （stride）可以不同，硬件每搬完一行就加上行偏移跳到下一行。
*
* 什么时候用：LCD 刷新、图层合成、摄像头格式转换、DMA 不能直接表达的
* 二维搬运（普通 DMA 只会线性搬字节，遇到行尾空隙就没办法）。
*
* ============================================================================
* 形态
* ============================================================================
*   - 一次启动搬 width × height 个像素，对应 height 行；每行 width 个像素。
*     它不是“一次交几段内存”，而是“一块矩形 + 两个行偏移”。源和目的各有
*     一个起始地址和一个行偏移，行的长度由 NLR.PL 统一给出。
*   - 提交一次：把 FGMAR/OMAR/FGOR/OOR/NLR/格式写进寄存器，然后置
*     CR.START。硬件自己跑完整个矩形并清 START。
*   - 完成一次：传输完成置 ISR.TCIF，可以开中断（CR.TCIE）或轮询。
*   - 内存由调用方提供，必须是物理地址（F4 没有 MMU，指针就是物理地址），
*     并按像素大小对齐。
*
* ============================================================================
* 寄存器与“跨步”语义（STM32F429，RM0090 第 11 章）
* ============================================================================
*   DMA2D_CR      模式（MODE[17:16]：00=M2M）与启动（START）、完成中断（TCIE）
*   DMA2D_FGMAR   前景（源）起始地址
*   DMA2D_OMAR    输出（目的）起始地址
*   DMA2D_FGOR    [13:0] LO：源的行间隔，单位是“像素”
*   DMA2D_OOR     [13:0] LO：目的的行间隔，单位是“像素”
*   DMA2D_NLR     [29:16] PL：每行像素数；[15:0] NL：行数
*   DMA2D_FGPFCCR [3:0] CM：源像素格式（0=ARGB8888）
*   DMA2D_OPFCCR  [2:0] CM：目的像素格式（0=ARGB8888）
*   DMA2D_ISR     TCIF（完成）、TEIF（传输错误）、CEIF（配置错误）等
*   DMA2D_IFCR    写 1 清对应的 ISR 标志
*
* 行偏移 LO 不是 stride，而是“行尾到下一行行首之间跳过的像素数”，
* 即 LO = stride - width。宽 64、stride 96 的源，LO = 32；目的 stride 128，
* LO = 64。若 stride == width（紧凑图像），LO = 0。
* 每行搬完后硬件把当前地址加上 (PL + LO) 个像素，所以多行之间可以是
* 固定间隔的“跨步”形态，不需要每行重新编程地址。
*
* 注意 NLR 的字段顺序：PL 在高 16 位（bit16 起），NL 在低 16 位。
* 也就是 DMA2D->NLR = (width << 16) | height，别写反。
* （已用 RM0090 的位段描述和多家公开代码交叉核对；本机无 RM0090 原文，
* 位段以手册为准。）
*
* ============================================================================
* 需要的条件
* ============================================================================
*   - STM32F429/F439 等带 DMA2D 的型号，DMA2D 时钟（RCC_AHB1ENR_DMA2DEN）；
*   - ARM 交叉工具链与 CMSIS 设备头（stm32f4xx.h 提供 DMA2D、RCC 指针和
*     DMA2D_* 位宏）；
*   - F4 没有 D-Cache，不需要 cache 维护；在 H7 上要先 clean/invalidate，
*     且 H7 的 FGOR/OOR 可用 CR.LOM 选择“字节”为单位的偏移，寄存器语义不同。
*
* ============================================================================
* 怎么编译、怎么运行、预期输出
* ============================================================================
*   arm-none-eabi-g++ -std=c++26 -mcpu=cortex-m4 -mthumb \
*       -mfpu=fpv4-sp-d16 -mfloat-abi=hard -O2 -ffreestanding \
*       -fno-exceptions -fno-rtti -I<cmsis>/Include \
*       -T stm32f429.ld startup.o dma2d_stm32.cpp -o dma2d_demo.elf
*   （工具链不支持 -std=c++26 时用 -std=c++2c 或 -std=c++23。）
*   烧录运行，串口输出：
*     DMA2D: 64x32 px, src stride 96 (LO=32), dst stride 128 (LO=64)
*     verify: 矩形一致；目的图行间空隙未被改写 -> OK
*
* 关中断轮询版本；开中断时把 while 换成 DMA2D_IRQHandler 里读 ISR 即可。
*/

#include "stm32f4xx.h"

#include <array>
#include <cstdint>
#include <cstdio>
#include <span>
#include <algorithm>

namespace {

constexpr std::uint32_t kWidth = 64;		/* 矩形宽（像素） */
constexpr std::uint32_t kHeight = 32;		/* 矩形高（行） */
constexpr std::uint32_t kSrcStride = 96;	/* 源图每行 96 像素 */
constexpr std::uint32_t kDstStride = 128;	/* 目的图每行 128 像素 */
constexpr std::uint32_t kSrcOffset = 5;		/* 源窗口左上角在行内的偏移 */
constexpr std::uint32_t kDstOffset = 7;		/* 目的窗口左上角在行内的偏移 */
constexpr std::uint32_t kSentinel = 0xDEADBEEFu;	/* 目的图空隙的哨兵值 */
constexpr std::uint32_t kSpinLimit = 10'000'000;	/* 轮询上限，防止死等 */

/* 像素格式编码，对应 FGPFCCR.CM / OPFCCR.CM。M2M 模式要求两边相同；
* M2M_PFC 模式才做转换。这里只列常用的几个。
*/
enum class PixelFormat : std::uint32_t {
	Argb8888 = 0,
	Rgb888 = 1,
	Rgb565 = 2,
	Argb1555 = 3,
	Argb4444 = 4,
};

/* 目的缓冲区：整张图，行间留空隙 */
std::array<std::uint32_t, kDstStride * kHeight> g_dst;
/* 源缓冲区：整张图，行间留空隙 */
std::array<std::uint32_t, kSrcStride * kHeight> g_src;

/* 打开 DMA2D 时钟。外设没时钟时读写寄存器会挂死（总线错误或读回 0），
* 这是裸机最常见的坑。
*/
void dma2d_enable_clock()
{
	RCC->AHB1ENR |= RCC_AHB1ENR_DMA2DEN;
	(void)RCC->AHB1ENR;	/* 读回一次，确保时钟使能已经生效 */
}

/* 一次跨步拷贝：从 src 的 (src_off, 0) 开始取 width×height 的窗口，
* 放到 dst 的 (dst_off, 0)。stride 单位都是像素。
*
* 返回 true 表示 TC 置位且无错误。
*/
[[nodiscard]] bool dma2d_copy_strided(std::span<const std::uint32_t> src,
					std::uint32_t src_stride,
					std::uint32_t src_off,
					std::span<std::uint32_t> dst,
					std::uint32_t dst_stride,
					std::uint32_t dst_off,
					std::uint32_t width,
					std::uint32_t height)
{
	/* 参数边界：PL 只有 14 位、LO 只有 14 位；stride 必须不小于 width */
	if (width == 0 || height == 0 || width > 0x3FFFu || height > 0xFFFFu)
		return false;
	if (src_stride < width + src_off || dst_stride < width + dst_off)
		return false;
	if (src.size() < src_stride * height || dst.size() < dst_stride * height)
		return false;

	/* 1. 配模式：MODE[17:16]=00 是 M2M。TCIE 打开“传输完成中断”，
	* 轮询模式下开着也无妨（标志照样置位）。
	* 写 CR 时 START 必须为 0，否则这次写被忽略。
	* 这里用 CMSIS 的字段名，不依赖 HAL 的 DMA2D_M2M 别名（HAL 头不在
	* 裸机工程里时没有这个别名）。
	*/
	DMA2D->CR = (0U << DMA2D_CR_MODE_Pos) | DMA2D_CR_TCIE;

	/* 2. 源/目的起始地址。硬件只认物理地址；F4 上没有 MMU，取指针即可。
	* 每个像素 4 字节，地址要 4 字节对齐（ARGB8888），否则 CEIF。
	*/
	auto src_addr = reinterpret_cast<std::uintptr_t>(src.data()) +
			src_off * sizeof(std::uint32_t);
	auto dst_addr = reinterpret_cast<std::uintptr_t>(dst.data()) +
			dst_off * sizeof(std::uint32_t);
	DMA2D->FGMAR = static_cast<std::uint32_t>(src_addr);
	DMA2D->OMAR = static_cast<std::uint32_t>(dst_addr);

	/* 3. 行偏移：不是 stride，是行尾到下一行行首的间隔。
	* 源：96 - 64 = 32 像素；目的：128 - 64 = 64 像素。
	* 单位为像素（ARGB8888 下 1 像素 = 4 字节）。
	*/
	DMA2D->FGOR = src_stride - width;
	DMA2D->OOR = dst_stride - width;

	/* 4. 像素格式。M2M 不做转换，两边写同样的格式；
	* 若要用 M2M_PFC，则 FGPFCCR 写源格式、OPFCCR 写目的格式。
	* FGPFCCR 的 alpha 模式/CLUT 这里都不需要，保持 0。
	*/
	DMA2D->FGPFCCR = static_cast<std::uint32_t>(PixelFormat::Argb8888);
	DMA2D->OPFCCR = static_cast<std::uint32_t>(PixelFormat::Argb8888);

	/* 5. 行列数：PL（每行像素数）在 [29:16]，NL（行数）在 [15:0]。
	* 写成 (width << 16) | height，不能反过来。
	*/
	DMA2D->NLR = (width << DMA2D_NLR_PL_Pos) | (height << DMA2D_NLR_NL_Pos);

	/* 6. 清旧的完成标志，再启动。IFCR 是写 1 清零；不先清的话，
	* 上一轮的 TCIF 会让我们误以为这一轮已经完成。
	*/
	DMA2D->IFCR = DMA2D_IFCR_CTCIF;
	DMA2D->CR |= DMA2D_CR_START;	/* START 由硬件在传输结束时清 0 */

	/* 7. 等完成。真实产品应开 TC 中断，这里为了自包含用轮询。
	* TEIF=传输错误，CEIF=配置错误，CAEIF=CLUT 访问错误：任何错误都退出。
	*/
	bool complete = false;
	for (std::uint32_t spin = 0; spin < kSpinLimit; ++spin) {
		std::uint32_t isr = DMA2D->ISR;
		if (isr & (DMA2D_ISR_TEIF | DMA2D_ISR_CEIF | DMA2D_ISR_CAEIF)) {
			/* 出错时也要清标志，否则下次一进来就误判 */
			DMA2D->IFCR = DMA2D_IFCR_CTEIF |
					DMA2D_IFCR_CCEIF |
					DMA2D_IFCR_CCAEIF;
			return false;
		}
		if (isr & DMA2D_ISR_TCIF) {
			complete = true;
			break;
		}
	}

	/* 8. 清 TCIF。DMA2D 完成一次、标志置一次，不清会粘住。 */
	DMA2D->IFCR = DMA2D_IFCR_CTCIF;

	if (!complete) {
		/* 轮询超时：主动 abort，别把一次没完成的传输留在硬件里 */
		DMA2D->CR |= DMA2D_CR_ABORT;
		return false;
	}
	return true;
}

/* 校验：目的图整块逐像素核对——矩形区域等于源窗口的对应像素，
* 矩形之外（左右空隙和行间空隙）必须还是哨兵值。后者证明行偏移真的
* 生效了，而不是按紧凑方式把行与行连在一起覆盖。
*/
bool verify_copy()
{
	for (std::uint32_t y = 0; y < kHeight; ++y) {
		for (std::uint32_t x = 0; x < kDstStride; ++x) {
			std::uint32_t want = kSentinel;

			if (x >= kDstOffset && x < kDstOffset + kWidth)
				want = g_src[y * kSrcStride + kSrcOffset +
						(x - kDstOffset)];
			if (g_dst[y * kDstStride + x] != want)
				return false;
		}
	}
	return true;
}

} // namespace

int main()
{
	/* 造一张源图：宽 64 的窗口放在 stride 96 的行里，每行 32 像素的空隙。
	* 像素值编上 (x, y)，方便校验。
	*/
	for (std::uint32_t y = 0; y < kHeight; ++y)
		for (std::uint32_t x = 0; x < kSrcStride; ++x)
			g_src[y * kSrcStride + x] = (y << 16) | x;

	g_dst.fill(kSentinel);

	dma2d_enable_clock();

	bool started = dma2d_copy_strided(g_src, kSrcStride, kSrcOffset,
						g_dst, kDstStride, kDstOffset,
						kWidth, kHeight);
	bool ok = started && verify_copy();

	std::printf("DMA2D: %ux%u px, src stride %u (LO=%u), dst stride %u (LO=%u)\n",
			kWidth, kHeight, kSrcStride, kSrcStride - kWidth,
			kDstStride, kDstStride - kWidth);
	std::printf("verify: %s\n", ok ? "OK" : "FAILED");

	/* 裸机程序不返回；真实工程里在这里进主循环 */
	for (;;)
		__asm volatile ("wfi");
}
