#pragma once

#include <ibus/cpo.hpp>
#include <ibus/sender.hpp>

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

// port：已打开的 usbfs 设备 fd + 一个端点（接口已由用户 CLAIMINTERFACE）。走异步 URB：
//   提交 USBDEVFS_SUBMITURB，完成要另外 USBDEVFS_REAPURB 收割——真正的异步，所以 communicate 返回 sender。
// 收割（reap）不是某个 URB 自己能做的：REAPURB 返回同一 fd 上任意一个已完成的 URB。
// sender 在 start 时提交；完成由 reaper 驱动（reaper 收到哪个 URB 就完成哪个 sender，放到哪个线程跑由用户定）。
// 用 exec::create 组合：URB 由在途状态持有（地址固定直到完成），usercontext 指回它。
// 提交失败（SUBMITURB 返回错误）时在 start 里同步 set_error(errno)。
// 完成：URB status < 0 时 set_error(-status)，否则 set_value(actual_length)。

namespace detail {
// usercontext 指向它：收到 URB 后由 reaper 调 complete(self)。
struct urb_header {
	void (*complete)(void*) noexcept;
	void* self;
};
}  // namespace detail

// 收割器：在用户选的线程上 sync_wait(run_one())，每收到一个 URB 就完成对应的 sender。
struct reaper {
	int fd;

	// 阻塞收割一个已完成的 URB 并完成对应的 sender；完成值为空，REAPURB 失败时 set_error(errno)。
	auto run_one() const noexcept {
		return ibus::detail::complete<stdexec::set_value_t(), stdexec::set_error_t(std::error_code)>([fd = fd](auto&& r) noexcept {
			::usbdevfs_urb* u = nullptr;
			if (::ioctl(fd, USBDEVFS_REAPURB, &u) < 0) return stdexec::set_error(std::move(r), ibus::detail::errno_code());
			auto* h = static_cast<detail::urb_header*>(u->usercontext);  // usercontext 在提交前设为在途状态的 header
			h->complete(h->self);
			stdexec::set_value(std::move(r));
		});
	}
};

// concept：send_pack / receive_pack(data, URB flags)，communicate 是一个 URB，完成值是 actual_length。
// transfer 的参考 receiver：正好接得住 transfer 里各组合发出的完成；用户的 receiver 照这个写，作为 transfer 的第二个参数检查。
struct transfer_receiver {
	using receiver_concept = stdexec::receiver_t;
	void set_value(std::size_t) && noexcept {}
	void set_error(std::error_code) && noexcept {}
};
template <class P, class R = transfer_receiver>
concept transfer = requires(P& p, std::span<std::byte const> out, std::span<std::byte> in, unsigned flags) {
	requires stdexec::sender_to<decltype(communicate(p, send_pack(p, out, flags))), R>;
	requires stdexec::sender_to<decltype(communicate(p, receive_pack(p, in, flags))), R>;
};

// port：一个端点。type：USBDEVFS_URB_TYPE_BULK / USBDEVFS_URB_TYPE_INTERRUPT；endpoint：端点地址（IN 端点带 0x80）。
struct port {
	int fd;
	unsigned char type;
	unsigned char endpoint;

	struct send_packet {
		std::span<std::byte const> data;
		unsigned flags;
	};
	struct receive_packet {
		std::span<std::byte> data;
		unsigned flags;
	};

	// flags：USBDEVFS_URB_SHORT_NOT_OK（短包算错）、USBDEVFS_URB_ZERO_PACKET（整包长度时补一个零长包）等。
	send_packet send_pack(std::span<std::byte const> d, unsigned flags = 0) const noexcept { return {d, flags}; }
	receive_packet receive_pack(std::span<std::byte> d, unsigned flags = 0) const noexcept { return {d, flags}; }

private:
	// exec::create 的 fn：它存在 op state 里，start 之后地址固定直到完成，所以在途状态（URB 等）直接放在它自己身上，
	// URB 原地构造，usercontext 指回 header。fn 返回 void，提交失败时在里面同步 set_error 是安全的。
	// usbdevfs_urb 末尾是柔性数组（bulk / interrupt 不用，0 个元素），所以用同样大小、对齐的原始存储。
	struct _submit {
		int fd;
		unsigned char type;
		unsigned char endpoint;
		unsigned flags;
		void* buffer;
		int length;
		detail::urb_header header;
		void* ctx;
		alignas(::usbdevfs_urb) std::byte storage[sizeof(::usbdevfs_urb)];

		::usbdevfs_urb& urb() noexcept { return *std::launder(reinterpret_cast<::usbdevfs_urb*>(storage)); }

		template <class Ctx>
		void operator()(Ctx& c) && noexcept {
			header = {&done<Ctx>, this};
			ctx = &c;
			auto& u = *::new (storage) ::usbdevfs_urb{};
			u.type = type;
			u.endpoint = endpoint;
			u.flags = flags;
			u.buffer = buffer;
			u.buffer_length = length;
			u.usercontext = &header;
			if (::ioctl(fd, USBDEVFS_SUBMITURB, &u) < 0) stdexec::set_error(std::move(c.receiver), ibus::detail::errno_code());
		}

		template <class Ctx>
		static void done(void* p) noexcept {
			auto* self = static_cast<_submit*>(p);
			auto& c = *static_cast<Ctx*>(self->ctx);
			auto& u = self->urb();
			if (u.status < 0) return stdexec::set_error(std::move(c.receiver), std::error_code(-u.status, std::generic_category()));
			stdexec::set_value(std::move(c.receiver), static_cast<std::size_t>(u.actual_length));
		}
	};

	auto _make(void* buf, std::size_t n, unsigned flags) const noexcept {
		return exec::create<stdexec::set_value_t(std::size_t), stdexec::set_error_t(std::error_code)>(
			_submit{fd, type, endpoint, flags, buf, static_cast<int>(n), {}, nullptr, {}});
	}

public:
	// 1 段：一个 URB。sender 完成值：actual_length（实际传输的字节数）；URB status < 0 时 set_error(-status)。
	auto communicate(send_packet p) const noexcept { return _make(const_cast<std::byte*>(p.data.data()), p.data.size(), p.flags); }
	auto communicate(receive_packet p) const noexcept { return _make(p.data.data(), p.data.size(), p.flags); }
};

static_assert(transfer<port>);

}  // namespace ibus::usb_urb
