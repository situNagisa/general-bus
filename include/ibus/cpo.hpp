#pragma once

#include <concepts>
#include <ranges>
#include <utility>

namespace ibus {

// communicate_range 的参数：元素是 T 的连续、有长度的范围（T 带 const 时也接受只读的范围）。
// 各 port 再用 std::ranges::borrowed_range 分两个重载：
//   不持有元素（左值、span 等视图）：只记地址，元素由用户保证活到完成；
//   持有元素的右值（临时 array / vector）：移进 sender，由 sender 管到完成（array 的移动就是复制）。
template <class R, class T>
concept contiguous_range_of = std::ranges::contiguous_range<R> && std::ranges::sized_range<R>
	&& std::convertible_to<std::add_pointer_t<std::ranges::range_reference_t<R>>, T*>;

// port：往上面发消息、从上面读消息的对象。不关心是一对一还是一对多，也不持有资源（fd 等由用户打开、关闭）。
// CPO 不加约束，只转发到 port 的成员。
// send_pack / receive_pack / transceive_pack 的参数（包括每次传输的属性：flags、速率、超时等）和返回类型都由 port 自己定；
// port 支持哪些组合由它自己的 communicate / communicate_range 重载决定。
// communicate 只接受 send_pack / receive_pack / transceive_pack 的返回值，communicate_range 只接受一个范围，都不接受额外参数；
// 要改的配置（flags 等）是 pack 的参数，或是 port 自己的成员。
// communicate / communicate_range 一律返回 sender：完成值走 set_value，错误走 set_error，不抛异常。
// 错误的类型由各总线自己定（错误来源是 errno 的总线就直接用 errno 的值，int）。
// 同步的操作在 start 时就地完成；调用方要结果就 sync_wait。
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
	constexpr auto operator()(auto& port, auto&& range) const -> decltype(port.communicate_range(std::forward<decltype(range)>(range))) {
		return port.communicate_range(std::forward<decltype(range)>(range));
	}
};
inline constexpr communicate_range_t communicate_range{};

}  // namespace ibus
