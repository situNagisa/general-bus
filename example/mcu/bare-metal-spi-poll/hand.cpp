// 同场景的手写版本（只用于和 main.cpp 对比 .text 大小，不是示例）。
#include <stm32f4xx.h>

#include <array>
#include <cstddef>
#include <cstdint>

static void xfer(SPI_TypeDef* spi, std::byte const* tx, std::byte* rx, std::size_t n) {
	auto volatile& dr = *reinterpret_cast<std::uint8_t volatile*>(&spi->DR);
	for (std::size_t i = 0; i < n; ++i) {
		while (!(spi->SR & SPI_SR_TXE)) {}
		dr = tx ? std::to_integer<std::uint8_t>(tx[i]) : std::uint8_t{0};
		while (!(spi->SR & SPI_SR_RXNE)) {}
		std::uint8_t v = dr;
		if (rx) rx[i] = std::byte{v};
	}
}

static void cs(bool active) noexcept { GPIOA->BSRR = active ? GPIO_BSRR_BR4 : GPIO_BSRR_BS4; }

int main() {
	std::array<std::byte, 1> cmd{std::byte{0x9F}};
	std::array<std::byte, 3> id{};
	cs(true);
	xfer(SPI1, cmd.data(), nullptr, cmd.size());
	xfer(SPI1, nullptr, id.data(), id.size());
	cs(false);
	std::array<std::byte, 4> cmd2{std::byte{0x03}, std::byte{0}, std::byte{0}, std::byte{0}};
	std::array<std::byte, 4> rx2{};
	cs(true);
	xfer(SPI1, cmd2.data(), rx2.data(), rx2.size());
	cs(false);
	for (;;) {}
}
