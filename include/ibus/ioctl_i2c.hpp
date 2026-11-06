#pragma once

#include <ibus/cpo.hpp>
#include <ibus/sender.hpp>

#include <cerrno>
#include <cstddef>
#include <system_error>
#include <span>

#include <unistd.h>

namespace ibus::ioctl_i2c {

// concept：send_pack(data) 一次 write、receive_pack(data) 一次 read（各是一次完整的 START…STOP），完成值是字节数。
// read_write 的参考 receiver：正好接得住 read_write 里各组合发出的完成；用户的 receiver 照这个写，作为 read_write 的第二个参数检查。
struct read_write_receiver {
	using receiver_concept = stdexec::receiver_t;
	void set_value(std::size_t) && noexcept {}
	void set_error(std::error_code) && noexcept {}
};
template <class P, class R = read_write_receiver>
concept read_write = requires(P& p, std::span<std::byte const> out, std::span<std::byte> in) {
	requires stdexec::sender_to<decltype(communicate(p, send_pack(p, out))), R>;
	requires stdexec::sender_to<decltype(communicate(p, receive_pack(p, in))), R>;
};

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

static_assert(read_write<port>);

}  // namespace ibus::ioctl_i2c
