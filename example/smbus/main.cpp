#include <ibus/smbus.hpp>

// smbus::port：ioctl(I2C_SMBUS)，SMBus 的固定格式事务，每种格式是 communicate 的一个重载（静态段数）。
// port 不持有 fd：打开设备、绑定从机地址（I2C_SLAVE）、开关 PEC（I2C_PEC）都由这里做。
// 用法：./smbus [/dev/i2c-N]

#include <array>
#include <cstdio>
#include <cstring>
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
		smbus::port l{fd};

		// read word data：命令 0x10 + 收 2 字节
		std::uint16_t w;
		stdexec::sync_wait(communicate(l, send_pack(l, smbus::command{0x10}), receive_pack(l, w)));
		std::printf("word 0x10 = %#06x\n", w);

		// write byte data：命令 0x20 + 1 字节
		stdexec::sync_wait(communicate(l, send_pack(l, smbus::command{0x20}), send_pack(l, std::uint8_t{0x5a})));

		// block read：命令 0x30 + 从机给出长度（最多 32）+ 数据
		::i2c_smbus_data blk{};
		stdexec::sync_wait(communicate(l, send_pack(l, smbus::command{0x30}), receive_pack(l, blk)));
		std::printf("block 0x30: %u bytes\n", blk.block[0]);
		::close(fd);
	} catch (int e) {  // sync_wait 抛出的 errno
		std::printf("error: %s\n", std::strerror(e));
	} catch (std::exception const& e) {
		std::printf("error: %s\n", e.what());
	}
}
