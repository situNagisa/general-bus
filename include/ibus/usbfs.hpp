#pragma once

#include <ibus/cpo.hpp>
#include <ibus/sender.hpp>

#include <cassert>
#include <cerrno>
#include <cstddef>
#include <system_error>
#include <span>

#include <endian.h>
#include <linux/usb/ch9.h>
#include <linux/usbdevice_fs.h>
#include <sys/ioctl.h>

namespace ibus::usbfs {

// concept：控制传输。send_pack(setup, timeout_ms) 是 setup 阶段，send_pack(data) / receive_pack(data) 是数据阶段；
// communicate(setup) / (setup, OUT) / (setup, IN) 各一次控制传输，完成值是数据阶段的字节数。
// control 的参考 receiver：正好接得住 control 里各组合发出的完成；用户的 receiver 照这个写，作为 control 的第二个参数检查。
struct control_receiver {
	using receiver_concept = stdexec::receiver_t;
	void set_value(std::size_t) && noexcept {}
	void set_error(std::error_code) && noexcept {}
};
template <class P, class R = control_receiver>
concept control = requires(P& p, ::usb_ctrlrequest const& setup, unsigned timeout_ms, std::span<std::byte const> out, std::span<std::byte> in) {
	requires stdexec::sender_to<decltype(communicate(p, send_pack(p, setup, timeout_ms))), R>;
	requires stdexec::sender_to<decltype(communicate(p, send_pack(p, setup, timeout_ms), send_pack(p, out))), R>;
	requires stdexec::sender_to<decltype(communicate(p, send_pack(p, setup, timeout_ms), receive_pack(p, in))), R>;
};

// port：一个 USB 设备的默认控制端点（已打开的 /dev/bus/usb/BBB/DDD，由用户打开、关闭）。
// USBDEVFS_CONTROL 是同步的：communicate 返回 sender，start 时一次 ioctl，完成值是数据阶段的字节数，失败 set_error(errno)。
struct port {
	int fd;

	struct setup_packet {
		::usb_ctrlrequest setup;
		unsigned timeout_ms;  // 0：一直等
	};
	struct out_packet {
		std::span<std::byte const> data;
	};
	struct in_packet {
		std::span<std::byte> data;
	};

	setup_packet send_pack(::usb_ctrlrequest const& s, unsigned timeout_ms = 1000) const noexcept { return {s, timeout_ms}; }
	out_packet send_pack(std::span<std::byte const> d) const noexcept { return {d}; }
	in_packet receive_pack(std::span<std::byte> d) const noexcept { return {d}; }

private:
	auto _control(setup_packet const& s, void* data) const noexcept {
		return detail::syscall<std::size_t>([fd = fd, s, data] noexcept {
			::usbdevfs_ctrltransfer c{
				.bRequestType = s.setup.bRequestType,
				.bRequest = s.setup.bRequest,
				.wValue = le16toh(s.setup.wValue),
				.wIndex = le16toh(s.setup.wIndex),
				.wLength = le16toh(s.setup.wLength),
				.timeout = s.timeout_ms,
				.data = data,
			};
			return ::ioctl(fd, USBDEVFS_CONTROL, &c);
		});
	}

public:
	// 控制传输，只有 setup 阶段。完成值 0。
	auto communicate(setup_packet s) const noexcept { return _control(s, nullptr); }

	// 控制传输，setup + OUT 数据阶段。完成值是发出的字节数。
	auto communicate(setup_packet s, out_packet d) const noexcept {
		assert((s.setup.bRequestType & USB_DIR_IN) == 0 && "usbfs: OUT data stage but bRequestType is IN");
		assert(d.data.size() == le16toh(s.setup.wLength) && "usbfs: data length != wLength");
		return _control(s, const_cast<std::byte*>(d.data.data()));
	}

	// 控制传输，setup + IN 数据阶段。完成值是收到的字节数。
	auto communicate(setup_packet s, in_packet d) const noexcept {
		assert((s.setup.bRequestType & USB_DIR_IN) != 0 && "usbfs: IN data stage but bRequestType is OUT");
		assert(d.data.size() == le16toh(s.setup.wLength) && "usbfs: data length != wLength");
		return _control(s, d.data.data());
	}
};

static_assert(control<port>);

}  // namespace ibus::usbfs
