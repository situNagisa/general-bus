// 不在本机编译：STM32F4/F7 的 DMA/USART 是芯片寄存器级代码，依赖 CMSIS 头
// （stm32f4xx.h）和目标板；本机是 Linux/WSL2，没有这些寄存器，也没有串口外设。
//
// ── 这是驱动实现 link 的手段，link 层看不到它 ──
// ibus 的 link 层只认识“一次交一段或两段内存、对应线上一串字节”：提交一次、
// 完成一次，内存由 link 提供或借用。至于串口用 DMA 循环搬、写指针由 NDTR 反推、
// 完成事件来自 IDLE 中断——全在驱动内部，link 层既看不到也管不着。
// 本文件只讲驱动这一半：UART 用循环 DMA 一直往环形缓冲里写，线空闲时算一次
// 写指针，把“上次读指针 → 本次写指针”之间的数据交给 link。
//
// ── 为什么 UART 接收要配 IDLE 中断 ──
// DMA 循环模式只会“搬字节”，它不知道一帧报文什么时候结束：串口线上没有帧边界，
// 你也不知道对方这次发了 3 个字节还是 300 个。USART 的 IDLE 标志在“接收线连续
// 空闲一个字符时间”后置位，正好对应“这一阵发完了”。所以常见组合是：
//   DMA 循环模式负责搬字节（永不停止，不丢字节）；
//   USART IDLE 中断负责划界（线一闲，就把这段时间收到的字节交出去）。
//
// ── 写指针怎么来 ──
// DMA_SxNDTR 初值 = 环大小，每搬一个字节减 1，减到 0 时循环模式自动重装。
// 所以任一时刻：
//   已写入位置 write = kRing - NDTR;
// 它只是“环里的下标”，不是累计字节数：绕回后从 0 重新开始。
// 拿 write 和“上次交给 link 的末端” g_read 比较，就知道这次的数据在环里
// 是一段还是两段。
//
// ── 形态：段数由数据位置决定，执行中才知道 ──
// dma_start() 只提交一次，之后驱动不再提交任何东西；完成事件不是“每字节一次”，
// 而是“线空闲”一次。每次完成交给 link 的是 1 段或 2 段内存：
//   没跨环尾：  [g_read, write)                     → 1 段
//   跨了环尾：  [g_read, kRing) + [0, write)        → 2 段
//   g_read=200, write=40 → [200,256) 和 [0,40)，两段
//   g_read=10,  write=40 → [10,40)，一段
// 提交时根本不知道下一次会跨不跨环尾，只有执行到 IDLE 中断、读了 NDTR 之后
// 才知道。所以 link 的接收接口必须能接受“最多两段”，不能假设数据连续。
// 内存由驱动提供（这里的 g_ring 是静态数组），link 拿到的是借用视图：
// 返回后驱动就可能让 DMA 覆盖它，link 必须同步处理完或拷走。
//
// ── 静默丢数据的地方 ──
// 1) IDLE 只在“线空闲”时来。如果对方连续不断地发、中间没有字符间隔，就永远
//    不进 IDLE，环会被 DMA 绕圈覆盖，而且没有任何错误标志。
// 2) 如果 link 处理太慢，DMA 可能在处理期间绕一整圈，覆盖还没交出去的数据。
// 3) write == g_read 既可能是“没有新数据”，也可能是“正好绕了整整一圈”，
//    IDLE+NDTR 无法区分。要检测这种情况，需要额外用 DMA 的 TC 中断数圈数，
//    或者保证环足够大、每次 IDLE 都及时取走。
// 本文件用 g_events/g_bytes 观察事件，但不假装能检测所有溢出。
//
// ── 硬件/软件条件 ──
// - STM32F4/F7，CMSIS 头；USART1_RX 接 PA10（AF7），波特率 115200。
// - USART1_RX → DMA2 Stream2 通道 4（也可用 Stream5 通道 4）。
// - NDTR 是 16 位，所以环大小不能超过 65535；这里用 256。
// - F7 开 D-Cache 时，g_ring 必须放 non-cacheable 区，或在读之前 invalidate，
//   否则 CPU 可能读到 cache 里的旧数据。F4 无 D-Cache。
// - F7 的 USART 寄存器改名：SR→ISR，DR→RDR，清 IDLE 改成写 USART_ICR_IDLECF；
//   本文件按 F4 的 SR/DR 写法，F7 移植时替换这几处即可。
//
// ── 编译与运行 ──
//   arm-none-eabi-g++ -std=c++26 -O2 -mcpu=cortex-m4 -mthumb \
//       -ffreestanding -fno-exceptions -fno-rtti -T stm32f4.ld \
//       startup_stm32f4xx.s uart_dma_ring.cpp -o uart_dma_ring.elf
// 烧录后从 PC 串口助手发数据，通过 SWO/ITM 看输出。预期输出每次线空闲一行：
//   event 1: 1 segment, 5 bytes (5+0) total=5
//   event 2: 2 segments, 60 bytes (20+40) total=65
// 第二行就是“跨环尾”：第一段 20 字节到环尾，第二段 40 字节从环头开始。

#include "stm32f4xx.h"

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <span>
#include <string_view>

namespace {

constexpr std::size_t kRing = 256;            // 必须是 1..65535，NDTR 是 16 位
constexpr std::uint32_t kUsart1RxChannel = 4; // DMA2 Stream2 CHSEL=4 → USART1_RX
constexpr std::uint32_t kBaud = 115200;
constexpr std::uint32_t kPclk2 = 84000000;    // 按实际 APB2 时钟改；这里假设 84 MHz

// DMA 直接写入的环形缓冲。alignas(32) 便于 F7 做 cache line 维护。
alignas(32) std::array<std::uint8_t, kRing> g_ring;

// 上次交给 link 的末端，即“上次读指针”。只有 IDLE ISR 读写，不需要原子操作。
// 注意它是环内下标 [0, kRing)，不是累计字节数。
std::size_t g_read = 0;

// 诊断用，中断写、主循环读，读的时候关中断。
volatile std::uint32_t g_events = 0;       // IDLE 事件次数
volatile std::uint32_t g_bytes = 0;        // 累计交付字节数
volatile std::uint32_t g_last_len[2]{};    // 最近一次两段各自的长度
volatile std::uint32_t g_last_segments = 0;
volatile std::uint32_t g_uart_errors = 0;  // ORE/FE/NE/PE 次数

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

// 驱动与 link 的边界：一次调用最多给两段。这里是示例实现，只记账；
// 真实 link 会在返回前把 first/second 拷走或处理完，因为 DMA 随时会覆盖环。
void link_receive(std::span<const std::uint8_t> first,
	std::span<const std::uint8_t> second) noexcept {
	const std::uint32_t n0 = static_cast<std::uint32_t>(first.size());
	const std::uint32_t n1 = static_cast<std::uint32_t>(second.size());

	g_last_len[0] = n0;
	g_last_len[1] = n1;
	g_last_segments = second.empty() ? 1U : 2U;
	g_bytes += n0 + n1;
	++g_events;
}

// 在 IDLE 中断里调用：读 NDTR 得到写指针，和上次读指针比较，切成 1 或 2 段。
void publish_from_isr() noexcept {
	const std::uint32_t remaining = DMA2_Stream2->NDTR;

	// NDTR=0 是一个短暂的边界（循环模式马上重装）。把它当成环尾位置 0，
	// 保证 write 始终落在 [0, kRing)，后面的比较不会越界。
	const std::size_t write =
		(remaining == 0) ? 0 : kRing - static_cast<std::size_t>(remaining);

	// 没有新数据就直接返回。注意：如果 DMA 正好绕了整整一圈，
	// write 也会等于 g_read，这里无法区分——这是 NDTR 方案的固有盲点。
	if (write == g_read) {
		return;
	}

	// 让 CPU 看到 DMA 刚写进内存的字节。F4 的 AHB 写对 CPU 可见，
	// 这个屏障主要防止编译器把后面的读挪到读 NDTR 之前。
	__DMB();

	if (write > g_read) {
		// 没跨环尾：数据是连续的一段。
		link_receive({g_ring.data() + g_read, write - g_read}, {});
	} else {
		// 跨了环尾：从上次读指针到环尾是一段，环头到写指针是另一段。
		// 两段在逻辑上是连续的，但物理上不连续；link 必须能接两段。
		link_receive({g_ring.data() + g_read, kRing - g_read},
			{g_ring.data(), write});
	}

	// 更新读指针到本次交付的末端。用刚才捕获的 write，而不是重新读 NDTR：
	// 处理期间新到的字节属于下一次事件，下次再交。
	g_read = write;
}

void usart1_init() noexcept {
	// 开 GPIOA、DMA2、USART1 时钟，读回确保生效。
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_DMA2EN;
	RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
	(void)RCC->AHB1ENR;
	(void)RCC->APB2ENR;

	// PA10 设成复用功能（MODER=10），AFR 选 AF7 = USART1。
	// 每 2 位一个引脚：PA10 → 第 10*2 位；AFR[1] 管 8..15，PA10 占 bit 8..11。
	GPIOA->MODER = (GPIOA->MODER & ~(3U << (10U * 2U))) | (2U << (10U * 2U));
	GPIOA->AFR[1] = (GPIOA->AFR[1] & ~(0xFU << 8)) | (7U << 8);

	// 16 倍过采样下的 BRR：四舍五入后的 fCK/baud。84MHz/115200 = 729。
	USART1->BRR = (kPclk2 + kBaud / 2) / kBaud;

	// RE=1 开接收；IDLEIE=1 开线空闲中断；UE 先不置，等 DMA 武装好再开。
	// 注意不置 RXNEIE：接收由 DMA 负责，CPU 不进“每字节”中断。
	USART1->CR1 = USART_CR1_RE | USART_CR1_IDLEIE;

	// DMAR=1 让 USART1 的接收请求接到 DMA。没有这一位，DMA 永远等不到请求。
	USART1->CR3 = USART_CR3_DMAR;
}

void dma_start() noexcept {
	DMA2_Stream2->CR &= ~DMA_SxCR_EN;
	while (DMA2_Stream2->CR & DMA_SxCR_EN) {
	}

	DMA2_Stream2->PAR = dma_addr(&USART1->DR);
	DMA2_Stream2->M0AR = dma_addr(g_ring.data());
	DMA2_Stream2->NDTR = static_cast<std::uint32_t>(kRing);

	// CHSEL=4 选 USART1_RX；DIR=00 外设→内存；MINC=1 内存地址递增；
	// CIRC=1 循环（到环尾自动绕回，永不停止）；PSIZE/MSIZE=00 字节。
	// 不开 DMA 的 HT/TC 中断：这里只靠 IDLE 划界。
	DMA2_Stream2->CR = (kUsart1RxChannel << DMA_SxCR_CHSEL_Pos) |
		(0U << DMA_SxCR_DIR_Pos) | DMA_SxCR_MINC | DMA_SxCR_CIRC |
		DMA_SxCR_PL_0;

	DMA2_Stream2->CR |= DMA_SxCR_EN;
	while (!(DMA2_Stream2->CR & DMA_SxCR_EN)) {
	}
}

}  // namespace

extern "C" void USART1_IRQHandler(void) {
	const std::uint32_t sr = USART1->SR;

	if (sr & USART_SR_IDLE) {
		// F4 清 IDLE 的固定序列：先读 SR，再读 DR。
		// 第二次读 SR 是为了满足“读 SR 后读 DR”的顺序要求，不是多余的。
		// 注意：如果此刻 RXNE 还挂着（DMA 还没搬走最后一个字节），读 DR 会
		// 把这个字节从 DMA 手里抢走，导致 NDTR 和实际数据错位。IDLE 表示
		// 已经空闲了一个字符时间，正常情况下 DMA 早搬完了，所以可以这样做。
		(void)USART1->SR;
		(void)USART1->DR;

		publish_from_isr();
	} else if (sr & (USART_SR_ORE | USART_SR_FE | USART_SR_NE | USART_SR_PE)) {
		// ORE（溢出）等错误也是读 SR 再读 DR 清除。这里只计数；
		// ORE 说明 DMA 没跟上或环被绕满，真实驱动应上报 link。
		(void)USART1->DR;
		++g_uart_errors;
	}
}

int main() {
	usart1_init();
	dma_start();

	// 最后再开 USART：保证第一个字节到达时 DMA 已经在等请求。
	// 顺序反了，开 UE 到置 EN 之间收到的字节没人搬，可能丢。
	USART1->CR1 |= USART_CR1_UE;

	NVIC_SetPriority(USART1_IRQn, 5U);
	NVIC_EnableIRQ(USART1_IRQn);

	std::uint32_t shown = 0;
	for (;;) {
		std::uint32_t events;
		std::uint32_t bytes;
		std::uint32_t len0;
		std::uint32_t len1;
		std::uint32_t segments;
		{
			irq_lock lock;
			events = g_events;
			bytes = g_bytes;
			len0 = g_last_len[0];
			len1 = g_last_len[1];
			segments = g_last_segments;
		}

		if (events == shown) {
			__WFI();
			continue;
		}
		shown = events;

		char line[160];
		std::snprintf(line, sizeof line,
			"event %lu: %lu segment%s, %lu bytes (%lu+%lu) total=%lu\n",
			static_cast<unsigned long>(events),
			static_cast<unsigned long>(segments), segments == 1 ? "" : "s",
			static_cast<unsigned long>(len0 + len1),
			static_cast<unsigned long>(len0), static_cast<unsigned long>(len1),
			static_cast<unsigned long>(bytes));
		trace(line);
	}
}
