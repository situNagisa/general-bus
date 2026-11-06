#pragma once

#include <ibus/cpo.hpp>
#include <ibus/sender.hpp>

#include <atomic>
#include <cassert>
#include <cerrno>
#include <cstddef>
#include <cstdint>
#include <span>

#include <linux/if_xdp.h>
#include <sys/socket.h>

// AF_XDP：四个用户态与内核共享的单生产者 / 单消费者环。当作协议直接收发：
//   communicate(send_pack(...)) 把一个描述直接写进环里的下一个槽位（不发布）；
//   communicate(send_pack(publish)) 推进 producer 发布出去（TX 环还会 sendto 唤醒内核）；
//   communicate(receive_pack()) 取下一个描述：返回指向环里那一项的指针，没有就是 nullptr（不拷贝）；
//   communicate(send_pack(publish)) 在消费者一侧发布 consumer，归还已取出的槽位。
// communicate 都返回 sender，start 时执行；sender 里记的是 port 的地址，start 之前 port 必须还活着。
// port 不持有资源：AF_XDP socket、UMEM 注册、环的 mmap、bind 都由用户做，port 只记下环的位置。
namespace ibus::af_xdp {

// 发布已写进环里的描述（send_pack 的参数）。
struct publish_t {};
inline constexpr publish_t publish{};

namespace detail {
inline std::uint32_t load(std::uint32_t* p) noexcept { return std::atomic_ref(*p).load(std::memory_order_acquire); }
inline void store(std::uint32_t* p, std::uint32_t v) noexcept { std::atomic_ref(*p).store(v, std::memory_order_release); }
}  // namespace detail

// 一个已 mmap 的环（XDP_MMAP_OFFSETS 给出 producer / consumer / desc 偏移），作为生产者一侧的 port。
// pending 是写进去但还没发布的生产者位置（只有用户态写 producer）。
template <class Desc>  // 类模板参数无法省略：描述类型是成员类型
struct producer_port {
	std::uint32_t* producer;
	std::uint32_t* consumer;
	Desc* desc;
	std::uint32_t size;  // 2 的幂
	std::uint32_t pending;

	static producer_port at(void* base, ::xdp_ring_offset const& off, std::uint32_t size) noexcept {
		auto* b = static_cast<std::byte*>(base);
		auto* prod = reinterpret_cast<std::uint32_t*>(b + off.producer);
		return {prod, reinterpret_cast<std::uint32_t*>(b + off.consumer), reinterpret_cast<Desc*>(b + off.desc), size, *prod};
	}

	struct write_packet {
		Desc d;
	};
	struct publish_packet {};

	write_packet send_pack(Desc d) const noexcept { return {d}; }
	publish_packet send_pack(publish_t) const noexcept { return {}; }

	// 写进下一个槽位（不发布）。空位不够是用户的逻辑错误。
	auto communicate(write_packet p) noexcept {
		return ibus::detail::nothrow<void>([self = this, p] noexcept {
			assert(self->size - (self->pending - detail::load(self->consumer)) > 0 && "af_xdp: ring full");
			self->desc[self->pending++ & (self->size - 1)] = p.d;
		});
	}

	// 发布：此后内核才看得到这些描述。
	auto communicate(publish_packet) noexcept {
		return ibus::detail::nothrow<void>([self = this] noexcept { detail::store(self->producer, self->pending); });
	}
};

// 消费者一侧的 port：RX 环、completion 环。pending 是已取出但还没归还的 consumer 位置（只有用户态写 consumer）。
template <class Desc>
struct consumer_port {
	std::uint32_t* producer;
	std::uint32_t* consumer;
	Desc* desc;
	std::uint32_t size;
	std::uint32_t pending;

	static consumer_port at(void* base, ::xdp_ring_offset const& off, std::uint32_t size) noexcept {
		auto* b = static_cast<std::byte*>(base);
		auto* cons = reinterpret_cast<std::uint32_t*>(b + off.consumer);
		return {reinterpret_cast<std::uint32_t*>(b + off.producer), cons, reinterpret_cast<Desc*>(b + off.desc), size, *cons};
	}

	struct read_packet {};
	struct publish_packet {};

	read_packet receive_pack() const noexcept { return {}; }
	publish_packet send_pack(publish_t) const noexcept { return {}; }

	// 取下一个描述：指向环里的那一项，归还（publish）之前一直有效；没有就是 nullptr。
	auto communicate(read_packet) noexcept {
		return ibus::detail::nothrow<Desc const*>([self = this] noexcept -> Desc const* {
			if (self->pending == detail::load(self->producer)) return nullptr;
			return &self->desc[self->pending++ & (self->size - 1)];
		});
	}

	// 归还已取出的槽位：发布 consumer。
	auto communicate(publish_packet) noexcept {
		return ibus::detail::nothrow<void>([self = this] noexcept { detail::store(self->consumer, self->pending); });
	}
};

// TX 环：发布后再 sendto(MSG_DONTWAIT) 唤醒内核。fill 环、completion 环、RX 环直接用 producer_port / consumer_port。
struct tx_port : producer_port<::xdp_desc> {
	int xsk;

	using producer_port::communicate;

	// EAGAIN / EBUSY / ENOBUFS 表示内核已在处理或暂时没空，不算错。
	auto communicate(publish_packet) noexcept {
		return ibus::detail::syscall<void>([self = this] noexcept {
			detail::store(self->producer, self->pending);
			if (::sendto(self->xsk, nullptr, 0, MSG_DONTWAIT, nullptr, 0) < 0 && errno != EAGAIN && errno != EBUSY && errno != ENOBUFS) return -1;
			return 0;
		});
	}
};

using fill_port = producer_port<std::uint64_t>;
using rx_port = consumer_port<::xdp_desc>;
using completion_port = consumer_port<std::uint64_t>;

}  // namespace ibus::af_xdp
