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

// link：一个 USB 设备的默认控制端点（已打开的 /dev/bus/usb/BBB/DDD，由用户打开、关闭）。
// USBDEVFS_CONTROL 是同步的，直接返回结果。
class link {
public:
	struct setup_packet {
		::usb_ctrlrequest setup;
	};
	struct out_packet {
		std::span<std::byte const> data;
	};
	struct in_packet {
		std::span<std::byte> data;
	};

	explicit link(int fd) noexcept : fd_(fd) {}

	void set_timeout_ms(unsigned v) noexcept { cfg_.timeout_ms = v; }  // 0：一直等

	setup_packet send_pack(::usb_ctrlrequest const& s) const noexcept { return {s}; }
	out_packet send_pack(std::span<std::byte const> d) const noexcept { return {d}; }
	in_packet receive_pack(std::span<std::byte> d) const noexcept { return {d}; }

	// 控制传输，只有 setup 阶段。返回 0。
	std::size_t communicate(setup_packet s) const { return control(s.setup, nullptr); }

	// 控制传输，setup + OUT 数据阶段。返回发出的字节数。
	std::size_t communicate(setup_packet s, out_packet d) const {
		assert((s.setup.bRequestType & USB_DIR_IN) == 0 && "usbfs: OUT data stage but bRequestType is IN");
		assert(d.data.size() == le16toh(s.setup.wLength) && "usbfs: data length != wLength");
		return control(s.setup, const_cast<std::byte*>(d.data.data()));
	}

	// 控制传输，setup + IN 数据阶段。返回收到的字节数。
	std::size_t communicate(setup_packet s, in_packet d) const {
		assert((s.setup.bRequestType & USB_DIR_IN) != 0 && "usbfs: IN data stage but bRequestType is OUT");
		assert(d.data.size() == le16toh(s.setup.wLength) && "usbfs: data length != wLength");
		return control(s.setup, d.data.data());
	}

private:
	struct config {
		unsigned timeout_ms = 1000;  // 0：一直等
	};

	std::size_t control(::usb_ctrlrequest const& setup, void* data) const {
		::usbdevfs_ctrltransfer c{
			.bRequestType = setup.bRequestType,
			.bRequest = setup.bRequest,
			.wValue = le16toh(setup.wValue),
			.wIndex = le16toh(setup.wIndex),
			.wLength = le16toh(setup.wLength),
			.timeout = cfg_.timeout_ms,
			.data = data,
		};
		int r = ::ioctl(fd_, USBDEVFS_CONTROL, &c);
		if (r < 0) throw std::system_error(errno, std::generic_category(), "ioctl(USBDEVFS_CONTROL)");
		return static_cast<std::size_t>(r);
	}

	int fd_;
	config cfg_;
};

}  // namespace ibus::usbfs
