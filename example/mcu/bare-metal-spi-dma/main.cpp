// 裸机，DMA + 中断：SPI1（DMA2 流 0 收、流 3 发，通道 3）往屏幕刷一块数据，同时 SPI2（DMA1 流 3 收、流 4 发，通道 0）读传感器。
// dma_port 在 start 时启动 DMA，RX 流的传输完成中断里完成 sender。when_all 等两路都完成。
// 完成发生在中断上下文：后面接的 then 也在中断里跑，所以这里只置一个标志，主循环 WFI 等它。
#include <ibus/stm32f4/spi.hpp>

#include <array>
#include <cstddef>
#include <cstdint>

using namespace ibus;
namespace spi = ibus::stm32f4::spi;

// 每个外设一个登记处，中断里用它找到在途的传输。
static spi::completion spi1_done;
static spi::completion spi2_done;

// 两路都完成后置位；有一路出错就记下 DMA 标志。
static bool volatile finished = false;
static std::uint32_t volatile dma_error = 0;

struct flag_receiver {
	using receiver_concept = stdexec::receiver_t;
	void set_value() && noexcept { finished = true; }
	void set_error(std::uint32_t flags) && noexcept {
		dma_error = flags;
		finished = true;
	}
	void set_stopped() && noexcept { finished = true; }  // when_all 一路出错时会让另一路停止
};

// 中断：读出流的标志、清掉、右移到流 0 的位置，交给登记处。
extern "C" void DMA2_Stream0_IRQHandler() {
	std::uint32_t f = DMA2->LISR & 0x3Du;  // 流 0 在 LISR 的位 0..5
	DMA2->LIFCR = f;
	if (f & (spi::tcif | spi::teif | spi::dmeif)) spi::complete(spi1_done, f);
}
extern "C" void DMA1_Stream3_IRQHandler() {
	std::uint32_t f = (DMA1->LISR >> 22) & 0x3Du;  // 流 3 在 LISR 的位 22..27
	DMA1->LIFCR = f << 22;
	if (f & (spi::tcif | spi::teif | spi::dmeif)) spi::complete(spi2_done, f);
}

static std::array<std::byte, 1024> frame{};
static std::array<std::byte, 6> sensor{};
static std::array<std::byte, 6> sensor_cmd{std::byte{0x80 | 0x28}};

int main() {
	// 时钟、引脚、SPI CR1、NVIC 使能由用户配置，这里省略。
	constexpr std::uint32_t ch3 = 3u << DMA_SxCR_CHSEL_Pos;
	spi::dma_port lcd{SPI1, DMA2_Stream0, DMA2_Stream3, ch3, ch3, &spi1_done};
	spi::dma_port imu{SPI2, DMA1_Stream3, DMA1_Stream4, 0, 0, &spi2_done};

	// op state 放在 main 的栈上：main 一直等到完成，所以活到中断触发之后。
	auto op = stdexec::connect(
		stdexec::when_all(communicate(lcd, send_pack(lcd, std::span<std::byte const>(frame))),
			communicate(imu, transceive_pack(imu, std::span<std::byte const>(sensor_cmd), std::span(sensor)))),
		flag_receiver{});
	stdexec::start(op);
	while (!finished) __WFI();

	for (;;) {}
}
