#include <ibus/smbus.hpp>

// smbus::link：ioctl(I2C_SMBUS)，SMBus 的固定格式事务，每种格式是 communicate 的一个重载（静态段数）。
// link 不持有 fd：打开设备、绑定从机地址（I2C_SLAVE）、开关 PEC（I2C_PEC）都由这里做。
// 用法：./smbus [/dev/i2c-N]

#include <array>
#include <cstdio>
#include <exception>
#include <span>
#include <system_error>

#include <fcntl.h>
#include <sys/ioctl.h>
#include <unistd.h>

int main(int argc, char** argv) {
	using namespace ibus;
	try {
		char const* dev = argc > 1 ? argv[1] : "/dev/i2c-1";
		int fd = ::open(dev, O_RDWR | O_CLOEXEC);
		if (fd < 0) throw std::system_error(errno, std::generic_category(), dev);
		if (::ioctl(fd, I2C_SLAVE, 0x50) < 0) throw std::system_error(errno, std::generic_category(), "ioctl(I2C_SLAVE)");
		smbus::link l(fd);

		// read word data：命令 0x10 + 收 2 字节
		std::uint16_t w;
		communicate(l, send_pack(l, smbus::command{0x10}), receive_pack(l, w));
		std::printf("word 0x10 = %#06x\n", w);

		// write byte data：命令 0x20 + 1 字节
		communicate(l, send_pack(l, smbus::command{0x20}), send_pack(l, std::uint8_t{0x5a}));

		// block read：命令 0x30 + 从机给出长度（最多 32）+ 数据
		std::array<std::byte, I2C_SMBUS_BLOCK_MAX> blk{};
		std::size_t n = communicate(l, send_pack(l, smbus::command{0x30}), receive_pack(l, std::span(blk)));
		std::printf("block 0x30: %zu bytes\n", n);
		::close(fd);
	} catch (std::exception const& e) {
		std::printf("error: %s\n", e.what());
	}
}
