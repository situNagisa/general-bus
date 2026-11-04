#pragma once

#include <ibus/cpo.hpp>

#include <atomic>
#include <cassert>
#include <cerrno>
#include <cstddef>
#include <cstdint>
#include <span>
#include <system_error>

#include <linux/if_xdp.h>
#include <sys/socket.h>

// 先编排、再一次提交的一类：AF_XDP。
// link 不持有资源：AF_XDP socket、UMEM 注册、四个环的 mmap 都由用户做，link 只接收环的位置。
// 四个环都是用户态与内核共享的单生产者 / 单消费者环：写描述、推进生产者索引就是“提交”，读描述、推进消费者索引就是“取完成”。
namespace ibus::af_xdp {

// 一个已 mmap 的环：XDP_MMAP_OFFSETS 给出的 producer / consumer / desc 偏移，加上 mmap 基址和环大小（2 的幂）。
template <class Desc>  // 类模板参数无法省略：描述类型是成员类型
struct ring {
	std::uint32_t* producer;
	std::uint32_t* consumer;
	Desc* desc;
	std::uint32_t size;

	static ring at(void* base, ::xdp_ring_offset const& off, std::uint32_t size) noexcept {
		auto* b = static_cast<std::byte*>(base);
		return {reinterpret_cast<std::uint32_t*>(b + off.producer), reinterpret_cast<std::uint32_t*>(b + off.consumer),
			reinterpret_cast<Desc*>(b + off.desc), size};
	}
};

namespace detail {
inline std::uint32_t load(std::uint32_t* p) noexcept { return std::atomic_ref(*p).load(std::memory_order_acquire); }
inline void store(std::uint32_t* p, std::uint32_t v) noexcept { std::atomic_ref(*p).store(v, std::memory_order_release); }

// 生产者一侧：把 n 个描述写进环，推进 producer。空位不够是用户的逻辑错误。
void produce(auto& r, auto&& descs) noexcept {
	std::uint32_t prod = *r.producer;
	assert(r.size - (prod - load(r.consumer)) >= std::ranges::size(descs) && "af_xdp: ring full");
	for (auto const& d : descs) r.desc[prod++ & (r.size - 1)] = d;
	store(r.producer, prod);
}

// 消费者一侧：最多取 out.size() 个描述，推进 consumer，返回取到的个数。
std::size_t consume(auto& r, auto out) noexcept {
	std::uint32_t cons = *r.consumer;
	std::uint32_t n = load(r.producer) - cons;
	if (n > out.size()) n = static_cast<std::uint32_t>(out.size());
	for (std::uint32_t i = 0; i < n; ++i) out[i] = r.desc[cons++ & (r.size - 1)];
	store(r.consumer, cons);
	return n;
}
}  // namespace detail

// tx_link：TX 环。send_pack 返回 xdp_desc（UMEM 里的偏移 + 长度）；communicate_range 写进 TX 环，
// 再 sendto(MSG_DONTWAIT) 唤醒内核（XDP_USE_NEED_WAKEUP 时只在环标记需要时才唤醒）。发完的 UMEM 块从 completion 环回来。
class tx_link {
public:
	tx_link(int xsk, ring<::xdp_desc> tx) noexcept : xsk_(xsk), tx_(tx) {}

	::xdp_desc send_pack(std::uint64_t umem_offset, std::uint32_t len) const noexcept { return {umem_offset, len, 0}; }

	void communicate(::xdp_desc d) { communicate_range(std::span(&d, 1)); }

	void communicate_range(auto&& descs) {
		detail::produce(tx_, descs);
		if (::sendto(xsk_, nullptr, 0, MSG_DONTWAIT, nullptr, 0) < 0 && errno != EAGAIN && errno != EBUSY && errno != ENOBUFS)
			throw std::system_error(errno, std::generic_category(), "af_xdp sendto");
	}

private:
	int xsk_;
	ring<::xdp_desc> tx_;
};

// fill_link：fill 环。把空闲的 UMEM 块借给内核用来接收；send_pack 返回块的 UMEM 偏移。
class fill_link {
public:
	explicit fill_link(ring<std::uint64_t> fill) noexcept : fill_(fill) {}

	std::uint64_t send_pack(std::uint64_t umem_offset) const noexcept { return umem_offset; }

	void communicate(std::uint64_t a) { communicate_range(std::span(&a, 1)); }
	void communicate_range(auto&& addrs) { detail::produce(fill_, addrs); }

private:
	ring<std::uint64_t> fill_;
};

// rx_link：RX 环。收到的包：UMEM 偏移 + 长度（写在 fill 环借出的块里）。用完的块由用户再交给 fill_link。
class rx_link {
public:
	explicit rx_link(ring<::xdp_desc> rx) noexcept : rx_(rx) {}

	std::span<::xdp_desc> receive_pack(std::span<::xdp_desc> out) const noexcept { return out; }

	// 不等待：最多取满 out，返回取到的个数。
	std::size_t communicate(std::span<::xdp_desc> out) { return detail::consume(rx_, out); }

private:
	ring<::xdp_desc> rx_;
};

// completion_link：completion 环。TX 发完、可复用的 UMEM 块偏移。
class completion_link {
public:
	explicit completion_link(ring<std::uint64_t> comp) noexcept : comp_(comp) {}

	std::span<std::uint64_t> receive_pack(std::span<std::uint64_t> out) const noexcept { return out; }

	std::size_t communicate(std::span<std::uint64_t> out) { return detail::consume(comp_, out); }

private:
	ring<std::uint64_t> comp_;
};

}  // namespace ibus::af_xdp
