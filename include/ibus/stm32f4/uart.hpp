#pragma once

#include <ibus/cpo.hpp>

#include <exec/create.hpp>
#include <stdexec/execution.hpp>

#include <cstddef>
#include <cstdint>
#include <span>

#include <stm32f4xx.h>

// STM32F4 的 UART 接收，DMA 循环模式：DMA 在后台不停地往用户的环形缓冲里写，port 只读 NDTR 算出写到哪了。
// 当作协议：communicate(receive_pack()) 取出上次之后新写进来的一段（指向环里，不拷贝，不等待），没有新数据就是空 span。
// 写到环尾会绕回：一次只给连续的一段，绕回的部分下次再取。
// UART、DMA 流（CIRC、MINC、外设→内存）的配置和启动都由用户做；DMA 追上读的位置（覆盖未读数据）也由用户保证不会发生。
namespace ibus::stm32f4::uart {

// ring_receive 的参考 receiver：完成值是新数据的那一段；不会失败。
struct ring_receive_receiver {
	using receiver_concept = stdexec::receiver_t;
	void set_value(std::span<std::byte const>) && noexcept {}
};

// concept：receive_pack() 取下一段新数据。
template <class P, class R = ring_receive_receiver>
concept ring_receive = requires(P& p) {
	requires stdexec::sender_to<decltype(communicate(p, receive_pack(p))), R>;
};

struct dma_ring_port {
	DMA_Stream_TypeDef* stream;
	std::byte const* buffer;
	std::uint32_t size;
	std::uint32_t pending;  // 下一次从这里读

	struct read_packet {};

	read_packet receive_pack() const noexcept { return {}; }

	// sender 里记的是 port 的地址：start 之前 port 必须还活着。
	auto communicate(read_packet) noexcept {
		return exec::create<stdexec::set_value_t(std::span<std::byte const>)>([self = this](auto& ctx) noexcept {
			std::uint32_t w = self->size - self->stream->NDTR;  // NDTR 从 size 倒数，写位置 = size - NDTR
			if (w == self->size) w = 0;
			std::uint32_t r = self->pending;
			std::uint32_t end = w >= r ? w : self->size;
			self->pending = end == self->size ? 0 : end;
			stdexec::set_value(std::move(ctx.receiver), std::span(self->buffer + r, end - r));
		});
	}
};

static_assert(ring_receive<dma_ring_port>);

}  // namespace ibus::stm32f4::uart
