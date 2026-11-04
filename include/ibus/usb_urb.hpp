#pragma once

#include <ibus/cpo.hpp>

#include <exec/create.hpp>
#include <stdexec/execution.hpp>

#include <cerrno>
#include <cstddef>
#include <cstdint>
#include <new>
#include <span>
#include <system_error>
#include <type_traits>
#include <utility>

#include <linux/usbdevice_fs.h>
#include <sys/ioctl.h>

namespace ibus::usb_urb {

// link：已打开的 usbfs 设备 fd + 一个端点（接口已由用户 CLAIMINTERFACE）。走异步 URB：
//   提交 USBDEVFS_SUBMITURB，完成要另外 USBDEVFS_REAPURB 收割——真正的异步，所以 communicate 返回 sender。
// 收割（reap）不是某个 URB 自己能做的：REAPURB 返回同一 fd 上任意一个已完成的 URB。
// sender 在 start 时提交；完成由 reaper 驱动（reaper 收到哪个 URB 就完成哪个 sender，放到哪个线程跑由用户定）。
// 用 exec::create 组合：URB 由在途状态持有（地址固定直到完成），usercontext 指回它。
// 已知问题：SUBMITURB 立即失败时在 start 里同步 set_error（exec::create 的 fn 内同步完成）。

namespace detail {
// usercontext 指向它：收到 URB 后由 reaper 调 complete(self)。
// （usbdevfs_urb 末尾是柔性数组，不能放进基类或被拷贝，所以在途状态把它放在最后一个成员，这里只做分发。）
struct urb_header {
	void (*complete)(void*) noexcept;
	void* self;
};
}  // namespace detail

// 收割器：在用户选的线程上调用 run_one() / run()，每收到一个 URB 就完成对应的 sender。
class reaper {
public:
	explicit reaper(int fd) noexcept : fd_(fd) {}

	// 阻塞收割一个已完成的 URB 并完成对应的 sender。
	void run_one() const {
		::usbdevfs_urb* u = nullptr;
		while (::ioctl(fd_, USBDEVFS_REAPURB, &u) < 0)
			if (errno != EINTR) throw std::system_error(errno, std::generic_category(), "ioctl(USBDEVFS_REAPURB)");
		auto* h = static_cast<detail::urb_header*>(u->usercontext);  // usercontext 在提交前设为在途状态的 header
		h->complete(h->self);
	}

private:
	int fd_;
};

class link {
public:
	struct send_packet {
		std::span<std::byte const> data;
	};
	struct receive_packet {
		std::span<std::byte> data;
	};

	// type：USBDEVFS_URB_TYPE_BULK / USBDEVFS_URB_TYPE_INTERRUPT；endpoint：端点地址（IN 端点带 0x80）。
	link(int fd, unsigned char type, unsigned char endpoint) noexcept : fd_(fd), type_(type), endpoint_(endpoint) {}

	void set_flags(unsigned v) noexcept { cfg_.flags = v; }  // USBDEVFS_URB_SHORT_NOT_OK、USBDEVFS_URB_ZERO_PACKET 等

	send_packet send_pack(std::span<std::byte const> d) const noexcept { return {d}; }
	receive_packet receive_pack(std::span<std::byte> d) const noexcept { return {d}; }

private:
	// 一次提交需要的字段（usbdevfs_urb 带柔性数组，不能按值捕获）。
	struct urb_args {
		int fd;
		unsigned char type;
		unsigned char endpoint;
		unsigned flags;
		void* buffer;
		int length;
	};

	// 在途状态：在 exec::create 的 op state 里原地构造（地址固定直到完成），usercontext 指回 header。
	// sender 只有完成后才会析构 op state，所以析构时 URB 已完成。
	template <class Ctx>  // 类模板参数无法省略：要用作成员类型
	struct pending {
		detail::urb_header header;
		Ctx& ctx;
		// usbdevfs_urb 末尾是柔性数组（bulk / interrupt 不用，0 个元素），不能直接作为成员放进 optional，
		// 所以用同样大小、对齐的原始存储，原地构造一个。
		alignas(::usbdevfs_urb) std::byte storage[sizeof(::usbdevfs_urb)];

		::usbdevfs_urb& urb_ref() noexcept { return *std::launder(reinterpret_cast<::usbdevfs_urb*>(storage)); }

		pending(Ctx& c, urb_args const& a) noexcept : header{&done, this}, ctx(c) {
			auto& urb = *::new (storage) ::usbdevfs_urb{};
			urb.type = a.type;
			urb.endpoint = a.endpoint;
			urb.flags = a.flags;
			urb.buffer = a.buffer;
			urb.buffer_length = a.length;
			urb.usercontext = &header;
			if (::ioctl(a.fd, USBDEVFS_SUBMITURB, &urb) < 0) fail(std::move(ctx.receiver), errno);
		}
		pending(pending&&) = delete;

		static void done(void* p) noexcept {
			auto* self = static_cast<pending*>(p);
			auto& urb = self->urb_ref();
			if (urb.status < 0) return fail(std::move(self->ctx.receiver), -urb.status);
			stdexec::set_value(std::move(self->ctx.receiver), static_cast<std::size_t>(urb.actual_length));
		}

		[[gnu::cold, gnu::noinline]] static void fail(auto r, int e) noexcept {
			stdexec::set_error(std::move(r), std::make_exception_ptr(std::system_error(e, std::generic_category(), "usb urb")));
		}
	};

	auto make(void* buf, std::size_t n) const noexcept {
		urb_args a{fd_, type_, endpoint_, cfg_.flags, buf, static_cast<int>(n)};
		return exec::create<stdexec::set_value_t(std::size_t), stdexec::set_error_t(std::exception_ptr)>(
			[a](auto& ctx) noexcept { return pending<std::remove_reference_t<decltype(ctx)>>(ctx, a); });
	}

public:
	// 1 段：一个 URB。sender 完成值：actual_length（实际传输的字节数）；URB status < 0 时 set_error。
	auto communicate(send_packet p) const noexcept { return make(const_cast<std::byte*>(p.data.data()), p.data.size()); }
	auto communicate(receive_packet p) const noexcept { return make(p.data.data(), p.data.size()); }

private:
	struct config {
		unsigned flags = 0;  // USBDEVFS_URB_SHORT_NOT_OK、USBDEVFS_URB_ZERO_PACKET 等
	};

	int fd_;
	unsigned char type_;
	unsigned char endpoint_;
	config cfg_;
};

}  // namespace ibus::usb_urb
