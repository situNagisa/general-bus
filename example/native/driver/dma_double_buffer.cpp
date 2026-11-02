// 不在本机编译：STM32F4/F7 的 DMA 是芯片寄存器级代码，依赖 CMSIS 头（stm32f4xx.h）
// 和目标板；本机是 Linux/WSL2，没有这些寄存器，也没有 ADC 外设。
//
// ── 这是驱动实现 link 的手段，link 层看不到它 ──
// ibus 的 link 层只认识“一次交一段内存、对应线上一串数据”：提交一次、完成一次，
// 内存由 link 提供或借用。DMA 有几个缓冲、CT 位现在指向哪一块、M0AR/M1AR 什么时候
// 被换掉，全在驱动内部，link 层看不到。本文件只讲驱动这一半：用双缓冲模式让 DMA
// 在两块内存之间轮转，每次填满一块就把这块交给 link，同时把空闲块换进另一个槽位。
//
// ── 双缓冲模式（DBM, double-buffer mode）是什么 ──
// 普通/循环模式只有一个内存指针 M0AR；DBM 多一个 M1AR，DMA 每次搬完一块就自动
// 切换到另一块，并把 NDTR 重装。两块缓冲长度相同（NDTR 只有一个），但地址可以
// 完全无关，甚至可以来自不同的内存池。
//   - DMA_SxCR 的 DBM=1 打开双缓冲。
//   - CT 位指示“当前正在写哪一块”：CT=0 写 M0AR，CT=1 写 M1AR。
//     硬件在每块传输完成时翻转 CT；软件只能在 EN=0 时写 CT。
//   - 使能 DBM 后循环模式自动生效（RM0090：CIRC 位无关，硬件按双缓冲一直轮转），
//     所以只需要武装一次。（本机没有 RM 原文，这句未逐页核对。）
//
// ── TC 中断里到底哪块是“刚填满的” ──
// 关键时序：一块搬完的瞬间，硬件先把 CT 翻到另一块，再置 TCIF。等 CPU 进中断时，
// CT 已经指向“现在正在写”的新块，刚填满的是另一块：
//   CT=1 → 当前写 M1AR，刚填满的是 M0AR；
//   CT=0 → 当前写 M0AR，刚填满的是 M1AR。
// 所以中断里要处理的是“CT 的反面”，不是 CT 指向的那块。
//
// ── 安全地更换 M0AR/M1AR ──
// 普通模式下 EN=1 后 M0AR/M1AR 是写保护的；DBM 是例外：运行中可以改写指针，
// 前提是只改“当前不在用”的那一个。正在被 CT 指向的那块如果改了地址，DMA 会把
// 这一块剩下的数据写到新地址去，直接损坏数据。本文件在每次 TC 里：
//   1) 处理刚填满的块；
//   2) 从空闲队列取一块新缓冲，写到“CT 的反面”那个槽位（当前不用的槽位）；
//   3) 把刚处理完的块放回空闲队列，下一轮再换进来。
// 这样缓冲池可以大于 2，指针一直在换，但永远不碰正在写的那块。
//
// ── 形态 ──
// 一次 dma_start() 之后不再提交；每填满一块产生一次 TC 完成事件，每次交给 link
// 一段 kBlock 个采样的内存（借用）。缓冲区由驱动提供（这里的 g_pool 是静态池）。
// link 必须在驱动把这块还给 DMA 之前处理完或拷走——本示例的 link_deliver 会拷贝，
// 所以返回后立刻可以回收。
//
// ── 处理太慢会怎样 ──
// 如果 ISR 处理时间超过一块的填充时间，处理期间 CT 会再次翻转：刚拷的那块可能
// 正被 DMA 重新写（读到撕裂数据），而按旧 CT 去换指针会改到正在写的槽位。
// 本文件在拷贝后重新读一次 CT，发现变了就记 g_late 并放弃这次换指针；但这只能
// 防止“改错指针”，救不回已经被覆盖的数据。真正的办法是保证处理时间小于一块时间。
//
// ── 硬件/软件条件 ──
// - STM32F4/F7，CMSIS 头；ADC1 的 4 个通道接 PA0..PA3，连续转换 + 扫描。
// - DMA2 Stream0 通道 0（映射表未核对）；DBM 在该 Stream 上可用。
// - F7 开 D-Cache 时，DMA 缓冲要放 non-cacheable 区或做 cache 维护。
//
// ── 编译与运行 ──
//   arm-none-eabi-g++ -std=c++26 -O2 -mcpu=cortex-m4 -mthumb
//       -ffreestanding -fno-exceptions -fno-rtti -T stm32f4.ld
//       startup_stm32f4xx.s dma_double_buffer.cpp -o dma_double_buffer.elf
// 烧录后通过 SWO/ITM 看输出。预期输出每块一行，m0/m1 是当前两个槽位指向的
// 池下标，可以看到它们在 4 个缓冲之间轮转：
//   block 1: sum=... | m0=2 m1=1 late=0
//   block 2: sum=... | m0=2 m1=3 late=0
//   block 3: sum=... | m0=0 m1=3 late=0
//   block 4: sum=... | m0=0 m1=1 late=0
// late 非 0 表示某次处理超时，那一块数据不可信。

#include "stm32f4xx.h"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <span>
#include <string_view>

namespace {

constexpr std::size_t kBlock = 128;           // 每块采样数，两个槽位必须一样长
constexpr std::size_t kPool = 4;              // 缓冲池块数：2 个在用 + 2 个空闲
constexpr std::uint32_t kAdcDmaChannel = 0;   // DMA2 Stream0 CHSEL=0 → ADC1
constexpr std::uint32_t kAdcSampleTime = 7;   // 480 个 ADC 周期
constexpr std::size_t kChannels = 4;

using sample_t = std::uint16_t;

// 缓冲池。DMA 只通过 M0AR/M1AR 看到其中两块，其余块在空闲队列里等着被换进来。
alignas(32) std::array<std::array<sample_t, kBlock>, kPool> g_pool;

// 当前 M0AR/M1AR 指向的块。必须和寄存器同步维护，换指针时一起改。
sample_t* g_m0 = nullptr;
sample_t* g_m1 = nullptr;

// 空闲队列：环形数组，head 取、tail 放。只有 ISR 访问，不需要加锁。
// 初始时 pool[0]、pool[1] 分别给 M0AR/M1AR，pool[2]、pool[3] 在队列里。
std::array<sample_t*, kPool> g_free{};
std::size_t g_free_head = 0;
std::size_t g_free_tail = 0;

std::array<sample_t, kBlock> g_snapshot;      // link 拷走的稳定副本
volatile std::uint32_t g_blocks = 0;
volatile std::uint32_t g_late = 0;            // 处理期间 CT 又翻转的次数
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

// 把指针映射回池下标，只用于打印，直观看到轮转。
std::size_t pool_index(const sample_t* p) noexcept {
	for (std::size_t i = 0; i < kPool; ++i) {
		if (p == g_pool[i].data()) {
			return i;
		}
	}
	return kPool;
}

void adc_init() noexcept {
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_DMA2EN;
	RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
	(void)RCC->AHB1ENR;
	(void)RCC->APB2ENR;

	// ADC 时钟由 ADC->CCR 的 ADCPRE 分频，F4 上不能超过 36 MHz；
	// 本示例假设板级初始化已把它设到合规频率（例如 PCLK2/4 = 21 MHz）。

	GPIOA->MODER |= GPIO_MODER_MODER0 | GPIO_MODER_MODER1 |
		GPIO_MODER_MODER2 | GPIO_MODER_MODER3;

	ADC1->CR1 = ADC_CR1_SCAN;
	ADC1->CR2 = ADC_CR2_ADON | ADC_CR2_CONT | ADC_CR2_DMA;
	ADC1->SMPR2 = (kAdcSampleTime << ADC_SMPR2_SMP0_Pos) |
		(kAdcSampleTime << ADC_SMPR2_SMP1_Pos) |
		(kAdcSampleTime << ADC_SMPR2_SMP2_Pos) |
		(kAdcSampleTime << ADC_SMPR2_SMP3_Pos);
	ADC1->SQR1 = static_cast<std::uint32_t>(kChannels - 1) << ADC_SQR1_L_Pos;
	ADC1->SQR3 = (0U << ADC_SQR3_SQ1_Pos) | (1U << ADC_SQR3_SQ2_Pos) |
		(2U << ADC_SQR3_SQ3_Pos) | (3U << ADC_SQR3_SQ4_Pos);

	for (std::uint32_t i = 0; i < 1000; ++i) {
		__NOP();
	}
}

// 双缓冲初始化。DBM 使能后不用设 CIRC：硬件会一直在这两块之间轮转。
// 注意 M1AR 只在 DBM 下有意义；非 DBM 时它在 EN=1 后是写保护的。
void dma_start() noexcept {
	DMA2_Stream0->CR &= ~DMA_SxCR_EN;
	while (DMA2_Stream0->CR & DMA_SxCR_EN) {
	}

	// 重建两个槽位和空闲队列。出错重启时也走这里。
	g_m0 = g_pool[0].data();
	g_m1 = g_pool[1].data();
	g_free[0] = g_pool[2].data();
	g_free[1] = g_pool[3].data();
	g_free_head = 0;
	g_free_tail = 2;

	DMA2_Stream0->PAR = dma_addr(&ADC1->DR);
	DMA2_Stream0->M0AR = dma_addr(g_m0);
	DMA2_Stream0->M1AR = dma_addr(g_m1);
	DMA2_Stream0->NDTR = static_cast<std::uint32_t>(kBlock);

	// DBM=1 打开双缓冲；TCIE 只要块完成中断。
	// HTIE 不开：半块中断在这套方案里不用，但 HTIF 仍会被硬件置位，
	// 所以 TC 中断里顺手把它清掉，避免脏标志。
	DMA2_Stream0->CR = (kAdcDmaChannel << DMA_SxCR_CHSEL_Pos) |
		(0U << DMA_SxCR_DIR_Pos) | DMA_SxCR_MINC |
		DMA_SxCR_PSIZE_0 | DMA_SxCR_MSIZE_0 |
		DMA_SxCR_PL_0 | DMA_SxCR_DBM | DMA_SxCR_TCIE;

	// CT 只能在 EN=0 时由软件写。清 0 表示第一块写 M0AR。
	DMA2_Stream0->CR &= ~DMA_SxCR_CT;

	DMA2_Stream0->CR |= DMA_SxCR_EN;
	while (!(DMA2_Stream0->CR & DMA_SxCR_EN)) {
	}
}

// 驱动与 link 的边界。这里只做拷贝，模拟 link 把数据取走；
// 返回后这块内存就可以还给 DMA 了。
void link_deliver(std::span<const sample_t> block) noexcept {
	std::copy(block.begin(), block.end(), g_snapshot.begin());
}

}  // namespace

extern "C" void DMA2_Stream0_IRQHandler(void) {
	const std::uint32_t sr = DMA2->LISR;

	if (sr & DMA_LISR_TCIF0) {
		// 清 TC 和可能残留的 HT 标志；只写自己这两位。
		DMA2->LIFCR = DMA_LIFCR_CTCIF0 | DMA_LIFCR_CHTIF0;
		__DMB();

		// 记录进入时的 CT。硬件在置 TCIF 之前已经翻转 CT，
		// 所以此时 CT 指向“正在写的新块”，反面才是刚填满的块。
		const std::uint32_t ct_at_entry = DMA2_Stream0->CR & DMA_SxCR_CT;
		const bool current_is_m1 = ct_at_entry != 0;
		sample_t* const filled = current_is_m1 ? g_m0 : g_m1;

		// 处理刚填满的块。link_deliver 会拷走，耗时应当远小于一块的填充时间。
		link_deliver({filled, kBlock});

		// 拷贝期间如果又完成了一块，CT 已经再次翻转：
		// 现在是哪块在写说不清，按旧 CT 换指针可能改到正在写的槽位。
		// 放弃这次换指针，等下一次（挂起的 TC 会再进中断）。
		if ((DMA2_Stream0->CR & DMA_SxCR_CT) != ct_at_entry) {
			g_late = g_late + 1;
			return;
		}
		g_blocks = g_blocks + 1;

		// 从空闲队列取一块新缓冲，换到“当前不用”的槽位：
		// CT=1 时 DMA 正在写 M1AR，只能改 M0AR；CT=0 时反之。
		// 这正是 DBM 允许的、运行中安全改指针的时机。
		sample_t* const fresh = g_free[g_free_head % kPool];
		++g_free_head;
		if (current_is_m1) {
			g_m0 = fresh;
			DMA2_Stream0->M0AR = dma_addr(fresh);
		} else {
			g_m1 = fresh;
			DMA2_Stream0->M1AR = dma_addr(fresh);
		}

		// 刚处理完的块已经拷贝走，可以回到空闲队列，
		// 下一轮再被换进某个槽位。顺序不能反：必须先处理完再回收。
		g_free[g_free_tail % kPool] = filled;
		++g_free_tail;
	}

	// 传输错误会让 DBM 停转。这里计数后整条流重新初始化；
	// 真实驱动应把错误上报给 link，并决定是否丢弃当前数据。
	if (sr & DMA_LISR_TEIF0) {
		DMA2->LIFCR = DMA_LIFCR_CTEIF0;
		g_errors = g_errors + 1;
		dma_start();
	}
	if (sr & DMA_LISR_DMEIF0) {
		DMA2->LIFCR = DMA_LIFCR_CDMEIF0;
		g_errors = g_errors + 1;
	}
	if (sr & DMA_LISR_FEIF0) {
		DMA2->LIFCR = DMA_LIFCR_CFEIF0;
		g_errors = g_errors + 1;
	}
}

int main() {
	adc_init();

	NVIC_SetPriority(DMA2_Stream0_IRQn, 5U);
	NVIC_EnableIRQ(DMA2_Stream0_IRQn);

	dma_start();
	ADC1->CR2 |= ADC_CR2_SWSTART;

	std::uint32_t shown = 0;
	for (;;) {
		std::array<sample_t, kBlock> snap{};
		std::uint32_t blocks;
		std::uint32_t m0;
		std::uint32_t m1;
		std::uint32_t late;
		{
			irq_lock lock;
			snap = g_snapshot;
			blocks = g_blocks;
			m0 = static_cast<std::uint32_t>(pool_index(g_m0));
			m1 = static_cast<std::uint32_t>(pool_index(g_m1));
			late = g_late;
		}

		if (blocks == shown) {
			__WFI();
			continue;
		}
		shown = blocks;

		std::uint32_t sum = 0;
		for (const sample_t v : snap) {
			sum += v;
		}

		char line[160];
		std::snprintf(line, sizeof line,
			"block %lu: sum=%lu | m0=%lu m1=%lu late=%lu\n",
			static_cast<unsigned long>(blocks), static_cast<unsigned long>(sum),
			static_cast<unsigned long>(m0), static_cast<unsigned long>(m1),
			static_cast<unsigned long>(late));
		trace(line);
	}
}
