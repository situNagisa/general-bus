// 裸机，UART 接收用 DMA 循环模式：USART2 RX 走 DMA1 流 5（通道 4），DMA 在后台不停地写环。
// 主循环（或 UART 空闲中断）里 communicate(receive_pack()) 取出新到的一段，直接在环里解析，不拷贝。
#include <ibus/stm32f4/uart.hpp>

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>

namespace uart = ibus::stm32f4::uart;

alignas(4) static std::array<std::byte, 256> ring{};

// 把完成值（新数据的一段）交给用户的处理函数。
struct chunk_receiver {
	using receiver_concept = stdexec::receiver_t;
	std::size_t* total;
	void set_value(std::span<std::byte const> chunk) && noexcept { *total += chunk.size(); /* 在这里解析 chunk */ }
};
static_assert(uart::ring_receive<uart::dma_ring_port, chunk_receiver>);

int main() {
	// 时钟、引脚、USART2（波特率、RE、DMAR）由用户配置；DMA 流由用户配置成 CIRC + MINC + 外设→内存并启动：
	DMA1_Stream5->PAR = reinterpret_cast<std::uint32_t>(&USART2->DR);
	DMA1_Stream5->M0AR = reinterpret_cast<std::uint32_t>(ring.data());
	DMA1_Stream5->NDTR = ring.size();
	DMA1_Stream5->CR = (4u << DMA_SxCR_CHSEL_Pos) | DMA_SxCR_MINC | DMA_SxCR_CIRC | DMA_SxCR_EN;

	uart::dma_ring_port rx{DMA1_Stream5, ring.data(), ring.size(), 0};
	std::size_t total = 0;
	for (;;) {
		auto op = stdexec::connect(communicate(rx, receive_pack(rx)), chunk_receiver{&total});
		stdexec::start(op);
		__WFI();  // 等下一个中断（例如 UART 空闲中断）再看
	}
}
