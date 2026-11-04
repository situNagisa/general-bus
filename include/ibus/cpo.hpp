#pragma once

#include <utility>

namespace ibus {

// CPO 不加约束，只转发到 link 的成员。
// send_pack / receive_pack 接受什么参数、返回什么类型（可以是空类型）都由 link 自己定；
// link 支持哪些组合由它自己的 communicate / communicate_range 重载决定。
// 同步就能完成的操作直接返回结果；只有真正需要异步完成的才返回 sender（由各 link 自己决定）。
// link 不持有资源：fd 等由用户打开、配置、关闭，link 只接收传入。
// 每次传输的附加属性（flags、速率、超时等）由 link 内部保存，link 对每一项提供 set_xxx()；发送时用当前值。
struct send_pack_t {
	constexpr auto operator()(auto& link, auto&&... args) const
		-> decltype(link.send_pack(std::forward<decltype(args)>(args)...)) {
		return link.send_pack(std::forward<decltype(args)>(args)...);
	}
};
inline constexpr send_pack_t send_pack{};

struct receive_pack_t {
	constexpr auto operator()(auto& link, auto&&... args) const
		-> decltype(link.receive_pack(std::forward<decltype(args)>(args)...)) {
		return link.receive_pack(std::forward<decltype(args)>(args)...);
	}
};
inline constexpr receive_pack_t receive_pack{};

// 一段里同时发和收（例如 SPI 全双工）。
struct transceive_pack_t {
	constexpr auto operator()(auto& link, auto&&... args) const
		-> decltype(link.transceive_pack(std::forward<decltype(args)>(args)...)) {
		return link.transceive_pack(std::forward<decltype(args)>(args)...);
	}
};
inline constexpr transceive_pack_t transceive_pack{};

// 静态 n 段：每段一个 pack，段数在编译期确定。
struct communicate_t {
	constexpr auto operator()(auto& link, auto&&... packs) const
		-> decltype(link.communicate(std::forward<decltype(packs)>(packs)...)) {
		return link.communicate(std::forward<decltype(packs)>(packs)...);
	}
};
inline constexpr communicate_t communicate{};

// 动态 n 段：一个 range（段数运行期决定）一次交给 link。
struct communicate_range_t {
	constexpr auto operator()(auto& link, auto&& range) const
		-> decltype(link.communicate_range(std::forward<decltype(range)>(range))) {
		return link.communicate_range(std::forward<decltype(range)>(range));
	}
};
inline constexpr communicate_range_t communicate_range{};

}  // namespace ibus
