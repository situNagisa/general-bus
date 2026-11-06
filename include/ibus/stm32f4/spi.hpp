#pragma once

#include <ibus/cpo.hpp>

#include <exec/create.hpp>
#include <stdexec/execution.hpp>

#include <cassert>
#include <cstddef>
#include <cstdint>
#include <span>

#include <stm32f4xx.h>

// STM32F4 的 SPI 主机（寄存器级，CMSIS 设备头）。时钟、引脚、CR1（模式、分频、8 位帧）、片选都由用户配置，port 只记下寄存器地址。
//   port：轮询。start 时就地收发完，set_value()。不会失败（没有错误通道）。
//   dma_port：DMA。start 时启动两个 DMA 流，RX 流的传输完成中断里完成 sender；DMA 出错时 set_error(流的中断标志)。
// send_pack(tx) / receive_pack(rx) / transceive_pack(tx, rx)：只发（收回来的丢掉）/ 只收（发 0）/ 全双工等长。
namespace ibus::stm32f4::spi {

// DMA 中断标志，统一成流 0 的位置（LISR / HISR 里每个流的那几位右移到最低位后）。
inline constexpr std::uint32_t feif = 1u << 0;
inline constexpr std::uint32_t dmeif = 1u << 2;
inline constexpr std::uint32_t teif = 1u << 3;
inline constexpr std::uint32_t htif = 1u << 4;
inline constexpr std::uint32_t tcif = 1u << 5;

// transfer 的参考 receiver：完成值为空；错误是 DMA 流的中断标志（轮询的 port 没有错误通道，也能连接）。
struct transfer_receiver {
	using receiver_concept = stdexec::receiver_t;
	void set_value() && noexcept {}
	void set_error(std::uint32_t) && noexcept {}
};

// concept：send_pack(tx) / receive_pack(rx) / transceive_pack(tx, rx)，communicate 各一次传输。
template <class P, class R = transfer_receiver>
concept transfer = requires(P& p, std::span<std::byte const> tx, std::span<std::byte> rx) {
	requires stdexec::sender_to<decltype(communicate(p, send_pack(p, tx))), R>;
	requires stdexec::sender_to<decltype(communicate(p, receive_pack(p, rx))), R>;
	requires stdexec::sender_to<decltype(communicate(p, transceive_pack(p, tx, rx))), R>;
};

// 一次传输：tx 为空时发 0，rx 为空时丢掉收到的。
struct packet {
	std::byte const* tx;
	std::byte* rx;
	std::size_t size;
};

// port：轮询。每个字节：等 TXE、写 DR、等 RXNE、读 DR。
struct port {
	SPI_TypeDef* spi;

	packet send_pack(std::span<std::byte const> tx) const noexcept { return {tx.data(), nullptr, tx.size()}; }
	packet receive_pack(std::span<std::byte> rx) const noexcept { return {nullptr, rx.data(), rx.size()}; }
	packet transceive_pack(std::span<std::byte const> tx, std::span<std::byte> rx) const noexcept {
		assert(tx.size() == rx.size() && "spi: transceive tx/rx length mismatch");
		return {tx.data(), rx.data(), rx.size()};
	}

	auto communicate(packet p) const noexcept {
		return exec::create<stdexec::set_value_t()>([spi = spi, p](auto& ctx) noexcept {
			auto volatile& dr = *reinterpret_cast<std::uint8_t volatile*>(&spi->DR);  // 8 位帧：按字节访问 DR
			for (std::size_t i = 0; i < p.size; ++i) {
				while (!(spi->SR & SPI_SR_TXE)) {}
				dr = p.tx ? std::to_integer<std::uint8_t>(p.tx[i]) : std::uint8_t{0};
				while (!(spi->SR & SPI_SR_RXNE)) {}
				std::uint8_t v = dr;
				if (p.rx) p.rx[i] = std::byte{v};
			}
			stdexec::set_value(std::move(ctx.receiver));
		});
	}
};

// DMA 传输完成的登记处：一个外设一个，由用户持有（中断要能找到它）。在途的传输把自己登记进来。
struct completion {
	void (*fn)(void*, std::uint32_t) noexcept;
	void* self;
};

// 在 RX 流的中断里调用：flags 是这个流的中断标志（已右移到流 0 的位置，并由用户清掉）。
inline void complete(completion& c, std::uint32_t flags) noexcept { c.fn(c.self, flags); }

// dma_port：SPI + 两个 DMA 流（RX：外设→内存，TX：内存→外设）。
// rx_cr / tx_cr：流的 CR 里由用户定的部分（CHSEL 通道、PL 优先级等）；方向、MINC、中断使能由 port 填。
// 同一个外设一次只能有一个传输在途（completion 只有一个）。tx 为空时发 0、rx 为空时收进一个丢弃字节（都不递增地址）。
struct dma_port {
	SPI_TypeDef* spi;
	DMA_Stream_TypeDef* rx_stream;
	DMA_Stream_TypeDef* tx_stream;
	std::uint32_t rx_cr;
	std::uint32_t tx_cr;
	completion* done;

	packet send_pack(std::span<std::byte const> tx) const noexcept { return {tx.data(), nullptr, tx.size()}; }
	packet receive_pack(std::span<std::byte> rx) const noexcept { return {nullptr, rx.data(), rx.size()}; }
	packet transceive_pack(std::span<std::byte const> tx, std::span<std::byte> rx) const noexcept {
		assert(tx.size() == rx.size() && "spi: transceive tx/rx length mismatch");
		return {tx.data(), rx.data(), rx.size()};
	}

private:
	// exec::create 的 fn：存在 op state 里，地址固定到完成，所以直接把自己登记进 completion。
	struct _start {
		dma_port port;
		packet p;
		void* ctx;
		std::byte dummy;  // tx 为空时发的 0 / rx 为空时的丢弃字节

		template <class Ctx>
		void operator()(Ctx& c) noexcept {
			ctx = &c;
			*port.done = {&_done<Ctx>, this};
			dummy = std::byte{0};
			DMA_Stream_TypeDef* rx = port.rx_stream;
			DMA_Stream_TypeDef* tx = port.tx_stream;
			rx->PAR = reinterpret_cast<std::uint32_t>(&port.spi->DR);
			rx->M0AR = reinterpret_cast<std::uint32_t>(p.rx ? p.rx : &dummy);
			rx->NDTR = p.size;
			rx->CR = port.rx_cr | (p.rx ? DMA_SxCR_MINC : 0u) | DMA_SxCR_TCIE | DMA_SxCR_TEIE | DMA_SxCR_DMEIE;
			tx->PAR = reinterpret_cast<std::uint32_t>(&port.spi->DR);
			tx->M0AR = reinterpret_cast<std::uint32_t>(p.tx ? p.tx : &dummy);
			tx->NDTR = p.size;
			tx->CR = port.tx_cr | (p.tx ? DMA_SxCR_MINC : 0u) | DMA_SxCR_DIR_0;
			rx->CR |= DMA_SxCR_EN;
			tx->CR |= DMA_SxCR_EN;
			port.spi->CR2 |= SPI_CR2_RXDMAEN | SPI_CR2_TXDMAEN;
		}

		template <class Ctx>
		static void _done(void* s, std::uint32_t flags) noexcept {
			auto* self = static_cast<_start*>(s);
			self->port.spi->CR2 &= ~(SPI_CR2_RXDMAEN | SPI_CR2_TXDMAEN);
			auto& c = *static_cast<Ctx*>(self->ctx);
			if (flags & (teif | dmeif)) return stdexec::set_error(std::move(c.receiver), flags);
			stdexec::set_value(std::move(c.receiver));
		}
	};

public:
	auto communicate(packet p) const noexcept {
		return exec::create<stdexec::set_value_t(), stdexec::set_error_t(std::uint32_t)>(_start{*this, p, nullptr, {}});
	}
};

static_assert(transfer<port> && transfer<dma_port>);

}  // namespace ibus::stm32f4::spi
