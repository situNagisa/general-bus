#include <ibus/usbfs.hpp>

// usbfs::port 的 communicate：控制传输（USBDEVFS_CONTROL）。
//   (send_pack(setup), receive_pack(data))  GET_DESCRIPTOR(DEVICE)：setup 阶段发 8 字节，数据阶段收 18 字节
// 用法：./usbfs /dev/bus/usb/BBB/DDD

#include <array>
#include <cstdio>
#include <exception>
#include <span>
#include <system_error>

#include <fcntl.h>
#include <unistd.h>

using namespace ibus;

int main(int argc, char** argv) {
	if (argc < 2) {
		std::printf("usage: %s /dev/bus/usb/BBB/DDD\n", argv[0]);
		return 1;
	}
	try {
		// port 不持有 fd：设备由这里打开、关闭。
		int fd = ::open(argv[1], O_RDWR | O_CLOEXEC);
		if (fd < 0) throw std::system_error(errno, std::generic_category(), argv[1]);
		usbfs::port l{fd};

		std::array<std::byte, USB_DT_DEVICE_SIZE> desc{};
		::usb_ctrlrequest setup{
			.bRequestType = USB_DIR_IN | USB_TYPE_STANDARD | USB_RECIP_DEVICE,
			.bRequest = USB_REQ_GET_DESCRIPTOR,
			.wValue = htole16(USB_DT_DEVICE << 8),
			.wIndex = 0,
			.wLength = htole16(desc.size()),
		};
		auto n = communicate(l, send_pack(l, setup, 1000), receive_pack(l, std::span<std::byte>(desc)));
		auto const* d = reinterpret_cast<::usb_device_descriptor const*>(desc.data());
		std::printf("%zu bytes: idVendor=%04x idProduct=%04x\n", n, le16toh(d->idVendor), le16toh(d->idProduct));
		::close(fd);
	} catch (std::exception const& e) {
		std::printf("error: %s\n", e.what());
	}
}
