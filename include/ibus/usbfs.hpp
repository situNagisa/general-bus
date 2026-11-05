#pragma once

#include <ibus/cpo.hpp>

#include <cassert>
#include <cerrno>
#include <cstddef>
#include <span>
#include <system_error>

#include <endian.h>
#include <linux/usb/ch9.h>
#include <linux/usbdevice_fs.h>
#include <sys/ioctl.h>

namespace ibus::usbfs {

// port：一个 USB 设备的默认控制端点（已打开的 /dev/bus/usb/BBB/DDD，由用户打开、关闭）。
// USBDEVFS_CONTROL 是同步的，直接返回结果。
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

	// 控制传输，只有 setup 阶段。返回 0。
	std::size_t communicate(setup_packet s) const { return _control(s, nullptr); }

	// 控制传输，setup + OUT 数据阶段。返回发出的字节数。
	std::size_t communicate(setup_packet s, out_packet d) const {
		assert((s.setup.bRequestType & USB_DIR_IN) == 0 && "usbfs: OUT data stage but bRequestType is IN");
		assert(d.data.size() == le16toh(s.setup.wLength) && "usbfs: data length != wLength");
		return _control(s, const_cast<std::byte*>(d.data.data()));
	}

	// 控制传输，setup + IN 数据阶段。返回收到的字节数。
	std::size_t communicate(setup_packet s, in_packet d) const {
		assert((s.setup.bRequestType & USB_DIR_IN) != 0 && "usbfs: IN data stage but bRequestType is OUT");
		assert(d.data.size() == le16toh(s.setup.wLength) && "usbfs: data length != wLength");
		return _control(s, d.data.data());
	}

private:
	std::size_t _control(setup_packet const& s, void* data) const {
		::usb_ctrlrequest const& setup = s.setup;
		::usbdevfs_ctrltransfer c{
			.bRequestType = setup.bRequestType,
			.bRequest = setup.bRequest,
			.wValue = le16toh(setup.wValue),
			.wIndex = le16toh(setup.wIndex),
			.wLength = le16toh(setup.wLength),
			.timeout = s.timeout_ms,
			.data = data,
		};
		int r = ::ioctl(fd, USBDEVFS_CONTROL, &c);
		if (r < 0) throw std::system_error(errno, std::generic_category(), "ioctl(USBDEVFS_CONTROL)");
		return static_cast<std::size_t>(r);
	}
};

}  // namespace ibus::usbfs
