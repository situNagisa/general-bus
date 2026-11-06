// 同场景的手写版本（只用于和 main.cpp 对比 .text 大小，不是示例）。
#include <stm32f4xx.h>

#include <array>
#include <cstddef>
#include <cstdint>

alignas(4) static std::array<std::byte, 256> ring{};

int main() {
	DMA1_Stream5->PAR = reinterpret_cast<std::uint32_t>(&USART2->DR);
	DMA1_Stream5->M0AR = reinterpret_cast<std::uint32_t>(ring.data());
	DMA1_Stream5->NDTR = ring.size();
	DMA1_Stream5->CR = (4u << DMA_SxCR_CHSEL_Pos) | DMA_SxCR_MINC | DMA_SxCR_CIRC | DMA_SxCR_EN;
	std::uint32_t r = 0;
	std::size_t total = 0;
	for (;;) {
		std::uint32_t w = ring.size() - DMA1_Stream5->NDTR;
		if (w == ring.size()) w = 0;
		std::uint32_t end = w >= r ? w : ring.size();
		total += end - r;  // 在这里解析 ring[r, end)
		r = end == ring.size() ? 0 : end;
		__WFI();
	}
}
