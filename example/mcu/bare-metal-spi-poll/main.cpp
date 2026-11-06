// 裸机，轮询：SPI1 读一个 SPI flash 的 JEDEC ID（发 0x9F，再收 3 字节），片选是 PA4。
// ibus::stm32f4::spi::port 在 start 时就地收发完。裸机上没有 sync_wait（它要线程同步原语），
// 所以自己写一个 receiver：connect + start，返回时就已经完成了。
#include <ibus/stm32f4/spi.hpp>

#include <array>
#include <cstddef>

using namespace ibus;
namespace spi = ibus::stm32f4::spi;

// 只接 set_value() 的 receiver：轮询的 port 不会失败，没有错误通道。
struct done_receiver {
	using receiver_concept = stdexec::receiver_t;
	void set_value() && noexcept {}
};
static_assert(spi::transfer<spi::port, done_receiver>);

static void run(auto&& sender) noexcept {
	auto op = stdexec::connect(static_cast<decltype(sender)&&>(sender), done_receiver{});
	stdexec::start(op);
}

static void cs(bool active) noexcept { GPIOA->BSRR = active ? GPIO_BSRR_BR4 : GPIO_BSRR_BS4; }

int main() {
	// 时钟、引脚复用、CR1（主机、分频、8 位帧、SPE）由用户配置，这里省略。
	spi::port p{SPI1};

	std::array<std::byte, 1> cmd{std::byte{0x9F}};
	std::array<std::byte, 3> id{};
	cs(true);
	run(communicate(p, send_pack(p, std::span<std::byte const>(cmd))));
	run(communicate(p, receive_pack(p, std::span(id))));
	cs(false);

	// 全双工：同一段时钟里发 cmd2、收 rx2
	std::array<std::byte, 4> cmd2{std::byte{0x03}, std::byte{0}, std::byte{0}, std::byte{0}};
	std::array<std::byte, 4> rx2{};
	cs(true);
	run(communicate(p, transceive_pack(p, std::span<std::byte const>(cmd2), std::span(rx2))));
	cs(false);

	for (;;) {}
}
