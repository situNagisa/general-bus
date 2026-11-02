#pragma once

#include <utility>

namespace ibus {

// ---------- 段的种类（communicate 的模板参数） ----------
namespace kind {
struct send {};
struct receive {};
struct submit {};
}  // namespace kind

// ---------- CPO ----------
// CPO 不加约束，只转发到 link / submission 的成员；约束由各 link 的 concept 表达。
template <class... K>
struct communicate_t {
	template <class Link, class... Args>
	constexpr auto operator()(Link& link, Args&&... args) const
		-> decltype(link.template communicate<K...>(std::forward<Args>(args)...)) {
		return link.template communicate<K...>(std::forward<Args>(args)...);
	}
};
template <class... K>
inline constexpr communicate_t<K...> communicate{};

struct send_t {
	template <class Submission, class... Args>
	constexpr auto operator()(Submission& s, Args&&... args) const
		-> decltype(s.send(std::forward<Args>(args)...)) {
		return s.send(std::forward<Args>(args)...);
	}
};
inline constexpr send_t send{};

struct receive_t {
	template <class Submission, class... Args>
	constexpr auto operator()(Submission& s, Args&&... args) const
		-> decltype(s.receive(std::forward<Args>(args)...)) {
		return s.receive(std::forward<Args>(args)...);
	}
};
inline constexpr receive_t receive{};

}  // namespace ibus
