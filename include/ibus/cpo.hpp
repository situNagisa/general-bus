#pragma once

#include <utility>

namespace ibus {

// port：往上面发消息、从上面读消息的对象。不关心是一对一还是一对多，也不持有资源（fd 等由用户打开、关闭）。
// CPO 不加约束，只转发到 port 的成员。
// send_pack / receive_pack / transceive_pack 的参数（包括每次传输的属性：flags、速率、超时等）和返回类型都由 port 自己定；
// port 支持哪些组合由它自己的 communicate / communicate_range 重载决定。
// 同步就能完成的操作直接返回结果；只有真正需要异步完成的才返回 sender。
struct send_pack_t {
	constexpr auto operator()(auto& port, auto&&... args) const
		-> decltype(port.send_pack(std::forward<decltype(args)>(args)...)) {
		return port.send_pack(std::forward<decltype(args)>(args)...);
	}
};
inline constexpr send_pack_t send_pack{};

struct receive_pack_t {
	constexpr auto operator()(auto& port, auto&&... args) const
		-> decltype(port.receive_pack(std::forward<decltype(args)>(args)...)) {
		return port.receive_pack(std::forward<decltype(args)>(args)...);
	}
};
inline constexpr receive_pack_t receive_pack{};

// 一段里同时发和收（例如 SPI 全双工）。
struct transceive_pack_t {
	constexpr auto operator()(auto& port, auto&&... args) const
		-> decltype(port.transceive_pack(std::forward<decltype(args)>(args)...)) {
		return port.transceive_pack(std::forward<decltype(args)>(args)...);
	}
};
inline constexpr transceive_pack_t transceive_pack{};

// 静态 n 段：每段一个 pack，段数在编译期确定。
struct communicate_t {
	constexpr auto operator()(auto& port, auto&&... packs) const
		-> decltype(port.communicate(std::forward<decltype(packs)>(packs)...)) {
		return port.communicate(std::forward<decltype(packs)>(packs)...);
	}
};
inline constexpr communicate_t communicate{};

// 动态 n 段：一个 range（段数运行期决定）一次交给 port；range 之后的参数是这一次调用的属性（例如 flags）。
struct communicate_range_t {
	constexpr auto operator()(auto& port, auto&& range, auto&&... args) const
		-> decltype(port.communicate_range(std::forward<decltype(range)>(range), std::forward<decltype(args)>(args)...)) {
		return port.communicate_range(std::forward<decltype(range)>(range), std::forward<decltype(args)>(args)...);
	}
};
inline constexpr communicate_range_t communicate_range{};

}  // namespace ibus
