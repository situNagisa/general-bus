#pragma once

#include <ibus/cpo.hpp>
#include <ibus/sender.hpp>

#include <cerrno>
#include <cstddef>
#include <span>

#include <unistd.h>

namespace ibus::ioctl_i2c {

// port：已打开的 /dev/i2c-N，且已 ioctl(I2C_SLAVE) 绑定从机地址（都由用户做）。
// 每次 communicate 是一次完整的 START…STOP（read / write）。communicate 返回 sender：start 时一次 read / write，完成值是字节数，失败 set_error(errno)。
struct port {
	int fd;

	struct send_packet {
		std::span<std::byte const> data;
	};
	struct receive_packet {
		std::span<std::byte> data;
	};

	send_packet send_pack(std::span<std::byte const> d) const noexcept { return {d}; }
	receive_packet receive_pack(std::span<std::byte> d) const noexcept { return {d}; }

	// 1 段 send：write。返回写出的字节数。
	auto communicate(send_packet p) const noexcept {
		return detail::syscall<std::size_t>([fd = fd, p] noexcept { return ::write(fd, p.data.data(), p.data.size()); });
	}

	// 1 段 receive：read。返回读到的字节数。
	auto communicate(receive_packet p) const noexcept {
		return detail::syscall<std::size_t>([fd = fd, p] noexcept { return ::read(fd, p.data.data(), p.data.size()); });
	}
};

}  // namespace ibus::ioctl_i2c
