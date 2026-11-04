#pragma once

#include <ibus/cpo.hpp>

#include <cerrno>
#include <cstddef>
#include <span>
#include <system_error>

#include <unistd.h>

namespace ibus::ioctl_i2c {

// link：已打开的 /dev/i2c-N，且已 ioctl(I2C_SLAVE) 绑定从机地址（都由用户做）。
// 每次 communicate 是一次完整的 START…STOP（read / write）。同步，直接返回结果。
class link {
public:
	struct send_packet {
		std::span<std::byte const> data;
	};
	struct receive_packet {
		std::span<std::byte> data;
	};

	explicit link(int fd) noexcept : fd_(fd) {}

	send_packet send_pack(std::span<std::byte const> d) const noexcept { return {d}; }
	receive_packet receive_pack(std::span<std::byte> d) const noexcept { return {d}; }

	// 1 段 send：write。返回写出的字节数。
	std::size_t communicate(send_packet p) const {
		::ssize_t r = ::write(fd_, p.data.data(), p.data.size());
		if (r < 0) throw std::system_error(errno, std::generic_category(), "i2c write");
		return static_cast<std::size_t>(r);
	}

	// 1 段 receive：read。返回读到的字节数。
	std::size_t communicate(receive_packet p) const {
		::ssize_t r = ::read(fd_, p.data.data(), p.data.size());
		if (r < 0) throw std::system_error(errno, std::generic_category(), "i2c read");
		return static_cast<std::size_t>(r);
	}

private:
	int fd_;
};

}  // namespace ibus::ioctl_i2c
