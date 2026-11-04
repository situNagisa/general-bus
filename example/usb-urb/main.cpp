#include <ibus/usb_urb.hpp>

// usb_urb::link：异步 bulk / interrupt URB。communicate 返回 sender：start 时 USBDEVFS_SUBMITURB 提交，
// 完成要靠 reaper 收割（USBDEVFS_REAPURB），reaper 放到哪个线程跑由用户定——这里放在一个 jthread 里。
// link 不持有 fd：打开设备、CLAIMINTERFACE 都由这里做。
// 用法：./usb-urb /dev/bus/usb/BBB/DDD 接口号 IN端点（如 0x81）

#include <array>
#include <cstdio>
#include <cstdlib>
#include <exception>
#include <span>
#include <system_error>
#include <thread>

#include <fcntl.h>
#include <sys/ioctl.h>
#include <unistd.h>

int main(int argc, char** argv) {
	using namespace ibus;
	if (argc < 4) {
		std::printf("usage: %s /dev/bus/usb/BBB/DDD interface in-endpoint\n", argv[0]);
		return 1;
	}
	try {
		int fd = ::open(argv[1], O_RDWR | O_CLOEXEC);
		if (fd < 0) throw std::system_error(errno, std::generic_category(), argv[1]);
		unsigned ifc = static_cast<unsigned>(std::strtoul(argv[2], nullptr, 0));
		if (::ioctl(fd, USBDEVFS_CLAIMINTERFACE, &ifc) < 0) throw std::system_error(errno, std::generic_category(), "CLAIMINTERFACE");
		auto ep = static_cast<unsigned char>(std::strtoul(argv[3], nullptr, 0));

		usb_urb::link in(fd, USBDEVFS_URB_TYPE_BULK, ep);
		usb_urb::reaper reaper(fd);
		std::jthread reaping([&] { reaper.run_one(); });  // 收割 1 个 URB

		std::array<std::byte, 512> buf{};
		auto [n] = stdexec::sync_wait(communicate(in, receive_pack(in, std::span(buf)))).value();
		std::printf("bulk in: %zu bytes\n", n);
		reaping.join();
		::ioctl(fd, USBDEVFS_RELEASEINTERFACE, &ifc);
		::close(fd);
	} catch (std::exception const& e) {
		std::printf("error: %s\n", e.what());
	}
}
