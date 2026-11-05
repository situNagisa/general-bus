#include <ibus/ioctl_i2c.hpp>

// 从 0x50 的寄存器 0x10 开始读 16 字节：先 write 寄存器地址，再 read。
// 两次各是一个完整的 START…STOP，中间有 STOP（没有 repeated START）；多数 EEPROM/传感器仍可工作。
// 用法：./ioctl-i2c [/dev/i2c-N]

#include <cstdio>
#include <exception>
#include <span>
#include <system_error>

#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <unistd.h>

int main(int argc, char** argv) {
	using namespace ibus;
	try {
		// port 不持有 fd：打开设备、绑定从机地址都由这里做。
		char const* dev = argc > 1 ? argv[1] : "/dev/i2c-1";
		int fd = ::open(dev, O_RDWR | O_CLOEXEC);
		if (fd < 0) throw std::system_error(errno, std::generic_category(), dev);
		if (::ioctl(fd, I2C_SLAVE, 0x50) < 0) throw std::system_error(errno, std::generic_category(), "ioctl(I2C_SLAVE)");
		ioctl_i2c::port l{fd};
		std::byte reg[1]{std::byte{0x10}}, data[16]{};

		auto w = communicate(l, send_pack(l, std::span<std::byte const>(reg)));
		auto n = communicate(l, receive_pack(l, std::span(data)));
		std::printf("wrote %zu, read %zu bytes\n", w, n);
		::close(fd);
	} catch (std::exception const& e) {
		std::printf("error: %s\n", e.what());
	}
}
