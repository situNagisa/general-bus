// 不在本机编译：STM32F4/F7 的 DMA 是芯片寄存器级代码，依赖 CMSIS 头（stm32f4xx.h）
// 和目标板；本机是 Linux/WSL2，没有这些寄存器，也没有 ADC 外设。
//
// ── 这是驱动实现 link 的手段，link 层看不到它 ──
// ibus 的 link 层只认识“一次交一段内存、对应线上一串数据”：提交一次，完成一次，
// 内存由 link 提供或借用。DMA 用哪个 Stream、循环模式怎么转、半传输中断什么时候来，
// 全在驱动内部，link 层看不到。本文件只讲驱动这一半：用循环 DMA 把 ADC 结果
// 连续搬进一个环，并在“前半满”“后半满”两个时刻分别把对应的一半交给 link。
//
// ── 循环模式（circular mode）是什么 ──
// 普通模式搬完 NDTR 就停，需要软件重新提交；循环模式在 NDTR 减到 0 时由硬件
// 自动把 NDTR 重新装回初值、把内存指针绕回缓冲区开头，EN 始终保持 1。
// 因此只需要武装一次，DMA 就永远在“缓冲区开头 → 结尾 → 开头”之间转。
//
// ── 半传输中断（HT）+ 传输完成中断（TC） ──
// 把一块长度 2*kHalf 的缓冲看成前后两半：
//   - 搬到第 kHalf 个时，NDTR 跨过一半，硬件置 HTIF 并进 HT 中断；
//     此刻前半已满，DMA 正在写后半。
//   - 搬到第 2*kHalf 个时，NDTR 归零并自动重装，硬件置 TCIF 并进 TC 中断；
//     此刻后半已满，DMA 已经绕回并开始写前半。
// 所以 HT 中断处理前半，TC 中断处理后半；两边轮流，处理期间 DMA 都在写另一边，
// 这就是“乒乓”式流水线，CPU 和 DMA 可以并行。
//
// ── 形态 ──
// 一次 dma_start() 之后不再提交任何东西；完成事件每半块发生一次，一圈两次。
// 每次完成交给 link 的是一段 kHalf 个采样的内存（借用，不复制）；缓冲区由驱动
// 提供（静态数组）。link 必须在这一半再次被 DMA 写之前处理完，否则丢掉的是
// 一整块数据，而且硬件不会报任何错。
//
// ── 处理速度必须跟上，否则静默覆盖 ──
// 半块的时间预算是：kHalf 个采样 × 每个采样的转换时间（采样时间 + 转换时间，
// 见 ADC 时钟配置）。假设 kHalf=128、每个采样约 500 个 ADC 周期、ADC 时钟
// 21 MHz，则半块约 3 ms。ISR 或 link 的处理如果超过 3 ms，DMA 已经写完另一半
// 并绕回来覆盖这一半了：你读到的可能是新旧混合的数据，但 HTIF/TCIF 照样正常，
// 不会置 OVR/TE，也不会有任何错误标志——这就是“静默覆盖”。
// 本文件用两个手段暴露它：
//   1) 一次中断里同时看到 HTIF 和 TCIF，说明至少一整块没来得及处理（g_late）；
//   2) 用 DWT 周期计数器量每次处理耗时（g_proc_max），和理论半块时间对比。
// 真要彻底避免，只能减少每块处理量、提高中断优先级、把处理搬到 DMA，或者
// 用双缓冲/多块环形队列给 link 留出更多余量。
//
// ── 硬件/软件条件 ──
// - STM32F4/F7，CMSIS 头；ADC1 的 4 个通道接 PA0..PA3。
// - 连续转换 + 扫描模式；DMA2 Stream0 通道 0。
// - F7 开 D-Cache 时，DMA 写入的缓冲区要做 cache invalidate 或放 non-cacheable 区，
//   否则 CPU 可能一直读到旧数据。F4 无 D-Cache，不需要。
//
// ── 编译与运行 ──
//   arm-none-eabi-g++ -std=c++26 -O2 -mcpu=cortex-m4 -mthumb \
//       -ffreestanding -fno-exceptions -fno-rtti -T stm32f4.ld \
//       startup_stm32f4xx.s dma_circular_half.cpp -o dma_circular_half.elf
// 烧录后通过 SWO/ITM 看输出。预期输出：每完成一圈打印前后两半的统计量，
// 以及处理耗时和“迟到”次数，例如：
//   half=1/2 sum=... min=... max=... | max_proc=1520 cycles late=0
// late 一旦非 0，说明某次处理超过了一整块的时间，数据已经不可信；
// max_proc 接近理论半块周期时也已经在危险边缘。

#include "stm32f4xx.h"

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <span>
#include <string_view>

namespace {

constexpr std::size_t kHalf = 128;                 // 半块采样数
constexpr std::size_t kRing = 2 * kHalf;           // NDTR 的初值 = 整块
constexpr std::uint32_t kAdcDmaChannel = 0;        // DMA2 Stream0 CHSEL=0 → ADC1
constexpr std::uint32_t kAdcSampleTime = 7;        // 480 个 ADC 周期

using sample_t = std::uint16_t;

// 前后两半在同一个数组里连续存放。DMA 只认首地址和 NDTR，
// 它不知道“前半/后半”的概念，分界是靠 HT/TC 中断时间点划出来的。
alignas(32) std::array<sample_t, kRing> g_buf;

struct half_result {
	std::uint32_t sum;
	sample_t min;
	sample_t max;
};

half_result g_results[2]{};          // 0=前半，1=后半，中断写、主循环读
volatile std::uint32_t g_half_count = 0;
volatile std::uint32_t g_full_count = 0;
volatile std::uint32_t g_late = 0;       // 一次中断同时看到 HT+TC 的次数
volatile std::uint32_t g_proc_max = 0;   // 单次处理的最大周期数
volatile std::uint32_t g_errors = 0;

class irq_lock {
public:
	irq_lock() noexcept : primask_{__get_PRIMASK()} { __disable_irq(); }
	~irq_lock() { __set_PRIMASK(primask_); }
	irq_lock(const irq_lock&) = delete;
	irq_lock& operator=(const irq_lock&) = delete;

private:
	std::uint32_t primask_;
};

template <class T>
std::uint32_t dma_addr(T* p) noexcept {
	return static_cast<std::uint32_t>(reinterpret_cast<std::uintptr_t>(p));
}

void trace(std::string_view text) noexcept {
	for (const char c : text) {
		ITM_SendChar(static_cast<std::uint32_t>(static_cast<unsigned char>(c)));
	}
}

void dwt_init() noexcept {
	CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
	DWT->CYCCNT = 0;
	DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

void adc_init() noexcept {
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_DMA2EN;
	RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
	(void)RCC->AHB1ENR;
	(void)RCC->APB2ENR;

	GPIOA->MODER |= GPIO_MODER_MODER0 | GPIO_MODER_MODER1 |
		GPIO_MODER_MODER2 | GPIO_MODER_MODER3;

	ADC1->CR1 = ADC_CR1_SCAN;
	ADC1->CR2 = ADC_CR2_ADON | ADC_CR2_CONT | ADC_CR2_DMA;
	ADC1->SMPR2 = (kAdcSampleTime << ADC_SMPR2_SMP0_Pos) |
		(kAdcSampleTime << ADC_SMPR2_SMP1_Pos) |
		(kAdcSampleTime << ADC_SMPR2_SMP2_Pos) |
		(kAdcSampleTime << ADC_SMPR2_SMP3_Pos);
	ADC1->SQR1 = static_cast<std::uint32_t>(4 - 1) << ADC_SQR1_L_Pos;
	ADC1->SQR3 = (0U << ADC_SQR3_SQ1_Pos) | (1U << ADC_SQR3_SQ2_Pos) |
		(2U << ADC_SQR3_SQ3_Pos) | (3U << ADC_SQR3_SQ4_Pos);

	for (volatile std::uint32_t i = 0; i < 1000; ++i) {
	}
}

// 循环模式只需要武装一次：NDTR 之后由硬件在每次 TC 时自动重装，
// 内存地址也由硬件绕回，软件不再参与。
void dma_start() noexcept {
	DMA2_Stream0->CR &= ~DMA_SxCR_EN;
	while (DMA2_Stream0->CR & DMA_SxCR_EN) {
	}

	DMA2_Stream0->PAR = dma_addr(&ADC1->DR);
	DMA2_Stream0->M0AR = dma_addr(g_buf.data());
	DMA2_Stream0->NDTR = static_cast<std::uint32_t>(kRing);

	// CIRC=1 是这里的核心：TC 时 NDTR 自动重装、地址绕回。
	// HTIE/TCIE 都开：前半满、后半满各进一次中断。
	DMA2_Stream0->CR = (kAdcDmaChannel << DMA_SxCR_CHSEL_Pos) |
		(0U << DMA_SxCR_DIR_Pos) | DMA_SxCR_MINC |
		DMA_SxCR_PSIZE_0 | DMA_SxCR_MSIZE_0 |
		DMA_SxCR_PL_0 | DMA_SxCR_CIRC |
		DMA_SxCR_HTIE | DMA_SxCR_TCIE;

	DMA2_Stream0->CR |= DMA_SxCR_EN;
	while (!(DMA2_Stream0->CR & DMA_SxCR_EN)) {
	}
}

// 统计一段数据，模拟 link 的处理工作。真实 link 可能在这里做协议解析、
// 拷贝到自己的队列或者投递给执行器；耗时必须小于半块时间。
half_result summarize(std::span<const sample_t> s) noexcept {
	half_result r{s.front(), s.front(), s.front()};
	for (const sample_t v : s.subspan(1)) {
		r.sum += v;
		if (v < r.min) {
			r.min = v;
		}
		if (v > r.max) {
			r.max = v;
		}
	}
	return r;
}

// index=0 处理前半，index=1 处理后半。两者跑在同一段 ISR 里，
// 谁被调用由 HTIF/TCIF 决定。
void handle_half(std::size_t index) noexcept {
	const std::uint32_t t0 = DWT->CYCCNT;

	// span 只借用 g_buf 的一部分，不复制。link 处理期间 DMA 正在写另一半，
	// 所以这一半是稳定的；但处理必须在下一次绕回之前结束。
	const std::span<const sample_t> half{g_buf.data() + index * kHalf, kHalf};
	g_results[index] = summarize(half);

	// 记录处理耗时，和理论半块周期比较。DWT 是自由运行计数器，减法自然处理回绕。
	const std::uint32_t dt = DWT->CYCCNT - t0;
	if (dt > g_proc_max) {
		g_proc_max = dt;
	}

	if (index == 0) {
		++g_half_count;
	} else {
		++g_full_count;
	}
}

}  // namespace

extern "C" void DMA2_Stream0_IRQHandler(void) {
	const std::uint32_t sr = DMA2->LISR;
	const bool ht = (sr & DMA_LISR_HTIF0) != 0;
	const bool tc = (sr & DMA_LISR_TCIF0) != 0;

	// 两个标志同时挂着：说明上一次中断之后整整半块（或更久）没人处理，
	// DMA 已经绕回并覆盖了先前那一半。数据不一定完全坏，但已经不可信。
	// 这是循环模式唯一能“察觉”处理不过来的廉价信号。
	if (ht && tc) {
		++g_late;
	}

	if (ht) {
		// 先清标志再处理：如果处理时间超了，新的 HT 标志会再次置位，
		// 退出 ISR 后立刻再进来（而不是被误清掉）。
		DMA2->LIFCR = DMA_LIFCR_CHTIF0;
		__DMB();
		handle_half(0);
	}

	if (tc) {
		// TC 时 NDTR 已经被硬件重装，DMA 正在写后半（接着绕回前半），
		// 这里处理的是刚满的后半。
		DMA2->LIFCR = DMA_LIFCR_CTCIF0;
		__DMB();
		handle_half(1);
	}

	// 传输错误会让循环停住。计数并重启：重新写 NDTR/M0AR/CR。
	// 注意重启会从缓冲区开头重新开始，之前未处理的数据就没了。
	if (sr & DMA_LISR_TEIF0) {
		DMA2->LIFCR = DMA_LIFCR_CTEIF0;
		++g_errors;
		dma_start();
	}
	if (sr & DMA_LISR_DMEIF0) {
		DMA2->LIFCR = DMA_LIFCR_CDMEIF0;
		++g_errors;
	}
	if (sr & DMA_LISR_FEIF0) {
		DMA2->LIFCR = DMA_LIFCR_CFEIF0;
		++g_errors;
	}
}

int main() {
	dwt_init();
	adc_init();

	NVIC_SetPriority(DMA2_Stream0_IRQn, 5U);
	NVIC_EnableIRQ(DMA2_Stream0_IRQn);

	// 循环模式：武装一次，之后直到断电/出错都不需要再碰 DMA。
	dma_start();
	ADC1->CR2 |= ADC_CR2_SWSTART;

	std::uint32_t shown = 0;
	for (;;) {
		half_result r0{};
		half_result r1{};
		std::uint32_t half_count;
		std::uint32_t full_count;
		std::uint32_t proc_max;
		std::uint32_t late;
		{
			irq_lock lock;
			r0 = g_results[0];
			r1 = g_results[1];
			half_count = g_half_count;
			full_count = g_full_count;
			proc_max = g_proc_max;
			late = g_late;
		}

		// 每完成一整圈（后半也处理完）打印一次。
		const std::uint32_t laps = (half_count < full_count) ? half_count : full_count;
		if (laps == shown) {
			__WFI();
			continue;
		}
		shown = laps;

		char line[192];
		std::snprintf(line, sizeof line,
			"lap %lu: front sum=%lu min=%u max=%u | back sum=%lu min=%u max=%u "
			"| max_proc=%lu cycles late=%lu\n",
			static_cast<unsigned long>(laps),
			static_cast<unsigned long>(r0.sum), static_cast<unsigned>(r0.min),
			static_cast<unsigned>(r0.max),
			static_cast<unsigned long>(r1.sum), static_cast<unsigned>(r1.min),
			static_cast<unsigned>(r1.max),
			static_cast<unsigned long>(proc_max), static_cast<unsigned long>(late));
		trace(line);
	}
}
