#include <ibus/i2c_rdwr.hpp>

// i2c_rdwr::port：动态 n 条消息，一次同步调用（ioctl(I2C_RDWR)）。
// 依次读 n 个寄存器，每个寄存器“写地址 + 读 1 字节”两条，共 2n 条，相邻之间 repeated START。
// send_pack / receive_pack 返回 i2c_msg，用户自己放进连续内存；条数上限 I2C_RDWR_IOCTL_MAX_MSGS（42）由用户保证。
// 用法：./i2c-rdwr [/dev/i2c-N] [寄存器个数]

#include <array>
#include <cstdio>
#include <tuple>
#include <cstdlib>
#include <exception>
#include <span>
#include <system_error>

#include <fcntl.h>
#include <unistd.h>

// sync_wait 取 sender 唯一的完成值（set_error(error_code) 时 sync_wait 抛 system_error）。
static auto value(auto&& s) { return std::get<0>(stdexec::sync_wait(std::forward<decltype(s)>(s)).value()); }

int main(int argc, char** argv) {
	using namespace ibus;
	try {
		// port 不持有 fd：设备由这里打开、关闭。
		char const* dev = argc > 1 ? argv[1] : "/dev/i2c-1";
		int fd = ::open(dev, O_RDWR | O_CLOEXEC);
		if (fd < 0) throw std::system_error(errno, std::generic_category(), dev);
		i2c_rdwr::port l{fd};
		constexpr std::uint16_t addr = 0x50;

		std::size_t n = argc > 2 ? std::strtoul(argv[2], nullptr, 0) : 3;
		if (n > I2C_RDWR_IOCTL_MAX_MSGS / 2) n = I2C_RDWR_IOCTL_MAX_MSGS / 2;
		std::array<std::byte, I2C_RDWR_IOCTL_MAX_MSGS / 2> regs{}, vals{};
		std::array<::i2c_msg, I2C_RDWR_IOCTL_MAX_MSGS> msgs;
		std::size_t count = 0;
		for (std::size_t i = 0; i < n; ++i) {
			regs[i] = std::byte(0x10 + i);
			msgs[count++] = send_pack(l, addr, std::span<std::byte const>(&regs[i], 1));
			msgs[count++] = receive_pack(l, addr, std::span<std::byte>(&vals[i], 1));
		}
		auto m = value(communicate_range(l, std::span(msgs).first(count)));
		std::printf("%zu of %zu msgs\n", m, count);

		// 每条消息的从机地址、附加 flags 都是 pack 的参数：例如写寄存器地址和写数据用 I2C_M_NOSTART 拼成线上一次写
		std::byte reg[1]{std::byte{0x10}}, data[4]{};
		std::array<::i2c_msg, 2> w;
		w[0] = send_pack(l, addr, std::span<std::byte const>(reg));
		w[1] = send_pack(l, addr, std::span<std::byte const>(data), I2C_M_NOSTART);
		std::printf("nostart: %zu msgs\n", value(communicate_range(l, w)));
		::close(fd);
	} catch (std::exception const& e) {
		std::printf("error: %s\n", e.what());
	}
}
