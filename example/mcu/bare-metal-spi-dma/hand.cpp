// 同场景的手写版本（只用于和 main.cpp 对比 .text 大小，不是示例）：两路 DMA 各自启动，中断里各置一个完成标志。
#include <stm32f4xx.h>

#include <array>
#include <cstddef>
#include <cstdint>

static bool volatile spi1_finished = false, spi2_finished = false;
static std::uint32_t volatile dma_error = 0;
static std::byte dummy1, dummy2;

static void start(SPI_TypeDef* spi, DMA_Stream_TypeDef* rx, DMA_Stream_TypeDef* tx, std::uint32_t rx_cr, std::uint32_t tx_cr,
	std::byte const* t, std::byte* r, std::size_t n, std::byte* dummy) {
	*dummy = std::byte{0};
	rx->PAR = reinterpret_cast<std::uint32_t>(&spi->DR);
	rx->M0AR = reinterpret_cast<std::uint32_t>(r ? r : dummy);
	rx->NDTR = n;
	rx->CR = rx_cr | (r ? DMA_SxCR_MINC : 0u) | DMA_SxCR_TCIE | DMA_SxCR_TEIE | DMA_SxCR_DMEIE;
	tx->PAR = reinterpret_cast<std::uint32_t>(&spi->DR);
	tx->M0AR = reinterpret_cast<std::uint32_t>(t ? t : dummy);
	tx->NDTR = n;
	tx->CR = tx_cr | (t ? DMA_SxCR_MINC : 0u) | DMA_SxCR_DIR_0;
	rx->CR |= DMA_SxCR_EN;
	tx->CR |= DMA_SxCR_EN;
	spi->CR2 |= SPI_CR2_RXDMAEN | SPI_CR2_TXDMAEN;
}

extern "C" void DMA2_Stream0_IRQHandler() {
	std::uint32_t f = DMA2->LISR & 0x3Du;
	DMA2->LIFCR = f;
	if (f & ((1u << 5) | (1u << 3) | (1u << 2))) {
		SPI1->CR2 &= ~(SPI_CR2_RXDMAEN | SPI_CR2_TXDMAEN);
		if (f & ((1u << 3) | (1u << 2))) dma_error = f;
		spi1_finished = true;
	}
}
extern "C" void DMA1_Stream3_IRQHandler() {
	std::uint32_t f = (DMA1->LISR >> 22) & 0x3Du;
	DMA1->LIFCR = f << 22;
	if (f & ((1u << 5) | (1u << 3) | (1u << 2))) {
		SPI2->CR2 &= ~(SPI_CR2_RXDMAEN | SPI_CR2_TXDMAEN);
		if (f & ((1u << 3) | (1u << 2))) dma_error = f;
		spi2_finished = true;
	}
}

static std::array<std::byte, 1024> frame{};
static std::array<std::byte, 6> sensor{};
static std::array<std::byte, 6> sensor_cmd{std::byte{0x80 | 0x28}};

int main() {
	constexpr std::uint32_t ch3 = 3u << DMA_SxCR_CHSEL_Pos;
	start(SPI1, DMA2_Stream0, DMA2_Stream3, ch3, ch3, frame.data(), nullptr, frame.size(), &dummy1);
	start(SPI2, DMA1_Stream3, DMA1_Stream4, 0, 0, sensor_cmd.data(), sensor.data(), sensor.size(), &dummy2);
	while (!(spi1_finished && spi2_finished)) __WFI();
	for (;;) {}
}
