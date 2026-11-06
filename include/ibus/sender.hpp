#pragma once

#include <exec/create.hpp>
#include <stdexec/execution.hpp>

#include <cerrno>
#include <type_traits>
#include <utility>

// communicate / communicate_range 返回的 sender 都由这里用 exec::create 组合出来：
// start 时同步调用 f（最多一次系统调用），完成值走 set_value。
// syscall 给错误来源是 errno 的总线用：错误走 set_error(int)，值就是 errno。
// f 的返回值类型是 void，所以在 f 里同步完成是安全的（exec::create 在调用 f 之后不再访问 op state）。
namespace ibus::detail {

// f() 返回系统调用的原始返回值 r：r < 0 时 set_error(errno)，否则 set_value(T(r))（T 为 void 时 set_value()）。
template <class T>
auto syscall(auto f) noexcept {
	if constexpr (std::is_void_v<T>) {
		return exec::create<stdexec::set_value_t(), stdexec::set_error_t(int)>(
			[f = std::move(f)](auto& ctx) mutable noexcept {
				if (f() < 0) stdexec::set_error(std::move(ctx.receiver), errno);
				else stdexec::set_value(std::move(ctx.receiver));
			});
	} else {
		return exec::create<stdexec::set_value_t(T), stdexec::set_error_t(int)>(
			[f = std::move(f)](auto& ctx) mutable noexcept {
				auto r = f();
				if (r < 0) stdexec::set_error(std::move(ctx.receiver), errno);
				else stdexec::set_value(std::move(ctx.receiver), static_cast<T>(r));
			});
	}
}

// 不会失败的操作（只读写共享内存）：start 时调用 f，返回值走 set_value（void 时 set_value()）。
template <class T>
auto nothrow(auto f) noexcept {
	if constexpr (std::is_void_v<T>) {
		return exec::create<stdexec::set_value_t()>([f = std::move(f)](auto& ctx) mutable noexcept {
			f();
			stdexec::set_value(std::move(ctx.receiver));
		});
	} else {
		return exec::create<stdexec::set_value_t(T)>(
			[f = std::move(f)](auto& ctx) mutable noexcept { stdexec::set_value(std::move(ctx.receiver), f()); });
	}
}

// 自己决定怎么完成：f(receiver&&) 里调用 set_value / set_error。
template <class... Sigs>
auto complete(auto f) noexcept {
	return exec::create<Sigs...>([f = std::move(f)](auto& ctx) mutable noexcept { f(std::move(ctx.receiver)); });
}

}  // namespace ibus::detail
