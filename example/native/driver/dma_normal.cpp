// 不在本机编译：STM32F4/F7 的 DMA 是芯片寄存器级代码，依赖 CMSIS 头（stm32f4xx.h）
// 和目标板；本机是 Linux/WSL2，没有这些寄存器，也没有 ADC 外设。
//
// ── 这是驱动实现 link 的手段，link 层看不到它 ──
// ibus 的 link 层只认识“一次交一段内存、对应线上一串数据”这件事：调用
// communicate<kind::receive>(link, span) 提交，link 在自己的执行器上完成。
// 这些数据是 CPU 一条条读 DR 搬的，还是 DMA 搬的；用了哪个 Stream、哪个通道；
// 普通模式还是循环模式；中断何时来——全在驱动内部，link 层既看不到也管不着。
// 本文件只讲驱动这一半：怎么让 DMA 把 ADC 的转换结果搬进内存，并在搬完时
// 把这段内存交给 link。
//
// ── DMA 是什么 ──
// DMA（Direct Memory Access）是芯片里的一个总线主设备：CPU 把“源地址、目的地址、
// 搬多少个、什么宽度、什么方向”写进它的寄存器，然后它自己在外设和内存之间搬数据，
// 每搬完一个单元 CPU 不需要参与。ADC 每转换完一个通道就产生一次 DMA 请求，
// DMA 就把 ADC->DR 里的结果读走、写进内存数组，CPU 可以去做别的事。
//
// ── Stream 和通道选择 ──
// STM32F4/F7 有 DMA1、DMA2 两个控制器，每个控制器 8 个 Stream（0..7）。
// 每个 Stream 的请求输入不固定，由 DMA_SxCR 的 CHSEL[2:0] 从 8 个通道里选一个；
// 哪个外设接在哪个控制器的哪个 Stream 的哪个通道，是芯片数据手册里的固定表，
// 不能随便改。本文件用 ADC1：
//   ADC1 → DMA2 Stream0 通道 0（也可以换成 DMA2 Stream4 通道 0）。
// 选错 Stream/通道，DMA 会一直等不到请求，看起来像“永远不进中断”。
//
// ── 普通模式（normal mode）的形态 ──
// 一次 dma_arm() 提交 N 个采样（N 个线上单元），只提交一次；DMA 搬完 N 个后
// 产生一次传输完成（TC）中断，并且硬件自动把 EN 清 0、停下来。要收下一批，
// 软件必须重新写 NDTR（以及需要的 M0AR/PAR）再置 EN。也就是说：
//   提交一次 → 完成一次 → 重新提交 → 再完成一次……
// 内存由驱动提供（这里的 g_dma_buf 是静态数组），link 只拿到一份视图/快照。
//
// ── 这个模式的致命点：停下到重启之间会丢数据 ──
// TC 之后 DMA 已经停了，但 ADC 还在连续转换。转换结果不断覆盖 ADC->DR，
// 没人搬走，这些采样就直接丢了（ADC 的 OVR 标志会置位）。dma_arm() 里的
// 关流、写 NDTR/M0AR/CR、再置 EN 都要花时间，这段时间就是一个“盲区”。
// 本文件用 DWT 周期计数器把盲区量出来（max_gap），并说明它意味着丢了多少采样。
// 要消除盲区只能改用循环模式（见 dma_circular_half.cpp、uart_dma_ring.cpp），
// 或者用双缓冲（dma_double_buffer.cpp）。
//
// ── 硬件/软件条件 ──
// - STM32F4 或 F7，CMSIS 头 stm32f4xx.h（F7 用 stm32f7xx.h，DMA 寄存器名相同）。
// - ADC1 的 4 个通道接在 PA0..PA3，用连续转换 + 扫描模式 + DMA。
// - 需要启动文件、SystemInit（时钟）和 SysTick 等基础设施；本文只写 DMA/ADC 部分。
// - F7 如果开了 D-Cache，DMA 写入的内存对 CPU 可能不可见：要么把缓冲区放到
//   MPU 配置的 non-cacheable 区，要么在中断里做 cache invalidate。F4 没有 D-Cache。
//
// ── 编译与运行 ──
//   arm-none-eabi-g++ -std=c++26 -O2 -mcpu=cortex-m4 -mthumb \
//       -ffreestanding -fno-exceptions -fno-rtti -T stm32f4.ld \
//       startup_stm32f4xx.s dma_normal.cpp -o dma_normal.elf
//   （工具链要支持 -std=c++26；老工具链换成 -std=c++2b 也能编过。）
// 烧录后通过 SWO/ITM 看输出（ITM_SendChar 只在调试器打开 ITM 时才有输出）。
// 预期输出：每完成一帧打印 4 个通道的原始值，以及累计重装次数和最大盲区周期数，
// 例如：
//   frame 1: 1203 1188 2047 1301 | rearm=1 max_gap=214 cycles
//   frame 2: 1205 1190 2045 1299 | rearm=2 max_gap=214 cycles
// max_gap 越大，盲区里被 ADC 覆盖掉的采样越多；按 ADC 采样率换算即可估算丢点数。

#include "stm32f4xx.h"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <span>
#include <string_view>

namespace {

// ADC 一次扫描 4 个通道；DMA 每搬一个结果就是一次请求。
constexpr std::size_t kChannels = 4;

// DMA2 Stream0 的 CHSEL=0 对应 ADC1（查芯片的 DMA 请求映射表）。
constexpr std::uint32_t kAdcDmaChannel = 0;

// 采样时间 480 个 ADC 时钟周期（SMP=7），给高阻信号源留足建立时间。
constexpr std::uint32_t kAdcSampleTime = 7;

using sample_t = std::uint16_t;

// DMA 的目的缓冲区。DMA 通过 AHB 总线直接写它，CPU 不参与。
// alignas(32) 让起始地址对齐到 cache line，F7 上做 cache 维护时不会误伤邻居数据。
alignas(32) std::array<sample_t, kChannels> g_dma_buf;

// 中断里的稳定快照。DMA 在 TC 后已经停下，此时把 g_dma_buf 拷到这里，
// 主循环再从容地读 g_frame，不会和下一次 DMA 传输打架。
std::array<sample_t, kChannels> g_frame;

// 下面几个变量在中断和主循环之间共享，读的时候要用 irq_lock 关中断。
volatile std::uint32_t g_frames = 0;          // 完成的帧数
volatile std::uint32_t g_rearms = 0;          // 重装 DMA 的次数
volatile std::uint32_t g_max_gap_cycles = 0;  // TC 到重新使能之间的最大周期数（盲区）
volatile std::uint32_t g_errors = 0;          // 传输错误/FIFO 错误次数

// RAII 临界区：构造时保存 PRIMASK 并关中断，析构时恢复原状。
// 比“关中断/开中断”成对调用安全，中途 return 也不会把中断永久关掉。
class irq_lock {
public:
	irq_lock() noexcept : primask_{__get_PRIMASK()} { __disable_irq(); }
	~irq_lock() { __set_PRIMASK(primask_); }
	irq_lock(const irq_lock&) = delete;
	irq_lock& operator=(const irq_lock&) = delete;

private:
	std::uint32_t primask_;
};

// 把指针转成 DMA 寄存器要的 32 位地址。Cortex-M 的地址就是 32 位，
// 先转 uintptr_t 再收窄，避免直接 reinterpret_cast 到整数类型的实现定义行为。
template <class T>
std::uint32_t dma_addr(T* p) noexcept {
	return static_cast<std::uint32_t>(reinterpret_cast<std::uintptr_t>(p));
}

// 通过 ITM/SWO 打印，不需要把 printf 重定向到串口（本文件的 USART 没占用）。
// 没有接调试器时 ITM_SendChar 会直接返回，不会卡住。
void trace(std::string_view text) noexcept {
	for (const char c : text) {
		ITM_SendChar(static_cast<std::uint32_t>(static_cast<unsigned char>(c)));
	}
}

// 打开 DWT 的周期计数器，只用来量“DMA 停了多久”，不参与功能。
void dwt_init() noexcept {
	CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
	DWT->CYCCNT = 0;
	DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

// ADC1 初始化：PA0..PA3 模拟输入，扫描 4 个通道，连续转换，DMA 请求使能。
// 注意这里只配置、不启动；启动放在 DMA 武装好之后，否则前几个转换没人搬。
void adc_init() noexcept {
	// 开 GPIOA、DMA2、ADC1 的时钟。外设寄存器写入前必须先开时钟，否则写入被忽略。
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_DMA2EN;
	RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
	// 读回寄存器，等时钟真正生效（AHB/APB 上的写有延迟，读回相当于一次屏障）。
	(void)RCC->AHB1ENR;
	(void)RCC->APB2ENR;

	// MODER 每 2 位一个引脚，11 = 模拟模式。模拟输入不需要上下拉。
	GPIOA->MODER |= GPIO_MODER_MODER0 | GPIO_MODER_MODER1 |
		GPIO_MODER_MODER2 | GPIO_MODER_MODER3;

	// SCAN=1：一次启动按 SQR3 的顺序扫多个通道。
	ADC1->CR1 = ADC_CR1_SCAN;

	// ADON=1 开 ADC；CONT=1 连续转换（一次 SWSTART 后不停）；
	// DMA=1 允许 ADC 发 DMA 请求；DDS 保持 0，表示每次转换都发请求。
	ADC1->CR2 = ADC_CR2_ADON | ADC_CR2_CONT | ADC_CR2_DMA;

	// 4 个通道都用 480 周期采样。
	ADC1->SMPR2 = (kAdcSampleTime << ADC_SMPR2_SMP0_Pos) |
		(kAdcSampleTime << ADC_SMPR2_SMP1_Pos) |
		(kAdcSampleTime << ADC_SMPR2_SMP2_Pos) |
		(kAdcSampleTime << ADC_SMPR2_SMP3_Pos);

	// L[3:0] = 序列长度 - 1。这里 4 个通道，所以写 3。
	ADC1->SQR1 = static_cast<std::uint32_t>(kChannels - 1) << ADC_SQR1_L_Pos;

	// SQR3 里每个 5 位字段放一个通道号：通道 0、1、2、3，顺序扫描。
	ADC1->SQR3 = (0U << ADC_SQR3_SQ1_Pos) | (1U << ADC_SQR3_SQ2_Pos) |
		(2U << ADC_SQR3_SQ3_Pos) | (3U << ADC_SQR3_SQ4_Pos);

	// ADON 置位后 ADC 需要 tSTAB 才稳定；这里用空循环粗等，正式代码可用定时器。
	for (volatile std::uint32_t i = 0; i < 1000; ++i) {
	}
}

// 普通模式重新武装：关流 → 写 PAR/M0AR/NDTR → 写 CR → 置 EN。
// 这个函数从 TC 中断里调用，调用期间 ADC 一直在转换，没被搬走的采样就是丢的。
void dma_arm() noexcept {
	// 1. 先清 EN 并等硬件确认停下。普通模式下 TC 后 EN 已经被硬件清 0，
	//    这里再清一次是为了兼容错误恢复路径（例如 TE 后流可能还挂着）。
	DMA2_Stream0->CR &= ~DMA_SxCR_EN;
	while (DMA2_Stream0->CR & DMA_SxCR_EN) {
		// 等 EN 真正读回 0。EN 是“写 1 使能、写 0 请求停止”，
		// 硬件要等当前 AHB 传输结束才停，不能刚写完就改 NDTR。
	}

	// 2. 配源地址和目的地址。PAR 是外设数据寄存器，ADC 每次转换完结果在这里。
	DMA2_Stream0->PAR = dma_addr(&ADC1->DR);
	DMA2_Stream0->M0AR = dma_addr(g_dma_buf.data());

	// 3. NDTR 是“还要搬多少个”，每搬一个减 1。必须在 EN=0 时写。
	DMA2_Stream0->NDTR = static_cast<std::uint32_t>(g_dma_buf.size());

	// 4. 写控制寄存器：
	//    CHSEL=0       选 ADC1 的请求线
	//    DIR=00        外设 → 内存（01 是内存→外设，10 是内存→内存）
	//    MINC=1        内存地址每搬一个自增（外设地址 PAR 不自增）
	//    PSIZE=01      外设侧 16 位（ADC->DR 是 16 位）
	//    MSIZE=01      内存侧 16 位（sample_t）
	//    PL=01         中等优先级
	//    TCIE=1        传输完成中断
	//    CIRC=0        普通模式；DBM=0 单缓冲
	DMA2_Stream0->CR = (kAdcDmaChannel << DMA_SxCR_CHSEL_Pos) |
		(0U << DMA_SxCR_DIR_Pos) | DMA_SxCR_MINC |
		DMA_SxCR_PSIZE_0 | DMA_SxCR_MSIZE_0 |
		DMA_SxCR_PL_0 | DMA_SxCR_TCIE;

	// 5. 置 EN 启动。置位后要等它读回 1，确认流真正开始响应请求。
	DMA2_Stream0->CR |= DMA_SxCR_EN;
	while (!(DMA2_Stream0->CR & DMA_SxCR_EN)) {
	}

	++g_rearms;
}

// 驱动与 link 的边界：link 只看到一段 4 个采样的内存，不知道它来自 DMA。
// 真实 link 会在这里把段提交给执行器；本示例直接打印，方便观察。
void link_publish(std::span<const sample_t> frame) noexcept {
	char line[160];
	std::snprintf(line, sizeof line,
		"frame %lu: %u %u %u %u | rearm=%lu max_gap=%lu cycles\n",
		static_cast<unsigned long>(g_frames),
		static_cast<unsigned>(frame[0]), static_cast<unsigned>(frame[1]),
		static_cast<unsigned>(frame[2]), static_cast<unsigned>(frame[3]),
		static_cast<unsigned long>(g_rearms),
		static_cast<unsigned long>(g_max_gap_cycles));
	trace(line);
}

}  // namespace

// DMA2 Stream0 的中断服务函数。名字必须和启动文件向量表里的一致，
// 所以用 extern "C" 禁止 C++ 名字修饰。
extern "C" void DMA2_Stream0_IRQHandler(void) {
	// LISR 覆盖 Stream0..3，是共享寄存器。先整体读一次，再按位判断，
	// 避免清完一个标志后读不到同一次中断里的另一个标志。
	const std::uint32_t sr = DMA2->LISR;

	if (sr & DMA_LISR_TCIF0) {
		// 记录进入中断的时刻，用来量“DMA 停下 → 重新使能”的盲区。
		const std::uint32_t t0 = DWT->CYCCNT;

		// 清 TC 标志。LIFCR 是“写 1 清零”，且只写自己这一位，
		// 千万不要写 0xFFFFFFFF，否则会把同组其它 Stream 的标志一起清掉。
		DMA2->LIFCR = DMA_LIFCR_CTCIF0;

		// 数据内存屏障：保证后面的读发生在“看到 TC 标志”之后，
		// 编译器也不会把 g_dma_buf 的读挪到清标志之前。
		__DMB();

		// 普通模式下硬件已经停了，此刻 g_dma_buf 是完整的一帧。
		// 先拷贝成稳定快照，再重新武装；顺序反过来会让 DMA 在新一帧里
		// 覆盖正在拷贝的内容。
		std::copy(g_dma_buf.begin(), g_dma_buf.end(), g_frame.begin());
		++g_frames;

		// 重新提交。这一句返回前，ADC 产生的新采样都进不了内存。
		dma_arm();

		// 盲区 = 从 TC 到 EN 重新置 1 的周期数（含清标志、拷贝、重装）。
		const std::uint32_t gap = DWT->CYCCNT - t0;
		if (gap > g_max_gap_cycles) {
			g_max_gap_cycles = gap;
		}
	}

	// 传输错误：地址/对齐/总线问题。普通模式下出错后流通常已停，
	// 计数后重新武装，让采集继续；真实驱动应把错误上报给 link。
	if (sr & DMA_LISR_TEIF0) {
		DMA2->LIFCR = DMA_LIFCR_CTEIF0;
		++g_errors;
		dma_arm();
	}

	// 直接模式错误（DMEIF）和 FIFO 错误（FEIF）：本配置不用 FIFO，
	// 但共享标志还是要清，否则会一直进中断。
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
	// 时钟树、SysTick 等由板级初始化/启动文件完成，这里从外设配置开始。
	dwt_init();
	adc_init();

	// NVIC：给 DMA 中断一个中等优先级并打开。优先级数值越小越高；
	// 这里只要不与更高优先级的硬实时中断抢即可。
	NVIC_SetPriority(DMA2_Stream0_IRQn, 5U);
	NVIC_EnableIRQ(DMA2_Stream0_IRQn);

	// 先把 DMA 武装好，再启动 ADC。顺序反了的话，SWSTART 之后到 EN=1
	// 之间的转换结果没人搬，开头就丢一帧。
	dma_arm();
	ADC1->CR2 |= ADC_CR2_SWSTART;

	std::uint32_t shown = 0;
	for (;;) {
		std::array<sample_t, kChannels> frame{};
		std::uint32_t frames;
		{
			// 关中断取快照：g_frame/g_frames 由中断更新，主循环读的时候
			// 不能被 TC 中断打断，否则可能拿到半新半旧的一帧。
			irq_lock lock;
			frame = g_frame;
			frames = g_frames;
		}

		if (frames == shown) {
			// 没有新帧就睡到下一个中断，别空转烧电。
			__WFI();
			continue;
		}
		shown = frames;

		// 这里就是“把一段内存交给 link”的位置：span 只借用，
		// link 返回后 frame 是主循环的局部变量，驱动不再持有它。
		link_publish(frame);
	}
}
