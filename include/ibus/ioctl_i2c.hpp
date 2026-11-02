#pragma once

#include <ibus/cpo.hpp>

#include <stdexec/execution.hpp>

#include <cerrno>
#include <concepts>
#include <cstddef>
#include <cstdint>
#include <ranges>
#include <system_error>
#include <tuple>
#include <type_traits>
#include <utility>

#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <sys/ioctl.h>
#include <unistd.h>

namespace ibus::ioctl_i2c {

// communicate<kind::submit> 接受的 submission：一段连续的 i2c_msg，原样交给 I2C_RDWR。
template <class S>
concept submission = std::ranges::contiguous_range<S> && std::ranges::sized_range<S>
	&& std::same_as<std::ranges::range_value_t<S>, ::i2c_msg>;

// link：对端（总线 + 从机地址）已确定。
class link {
public:
	link(char const* dev, std::uint16_t addr) : fd_(::open(dev, O_RDWR | O_CLOEXEC)), addr_(addr) {
		if (fd_ < 0) throw std::system_error(errno, std::generic_category(), dev);
	}
	link(link const&) = delete;
	link& operator=(link const&) = delete;
	~link() { ::close(fd_); }

	std::uint16_t address() const noexcept { return addr_; }

	// 一次 ioctl(I2C_RDWR)；完成值是完成的条数。左值转发时只存引用，右值时存值。
	template <class K, class S>
		requires std::same_as<K, kind::submit> && submission<std::remove_cvref_t<S>>
	auto communicate(S&& s) {
		return stdexec::just()
			| stdexec::then([fd = fd_, t = std::tuple<S>(std::forward<S>(s))]() mutable -> std::size_t {
				auto& s = std::get<0>(t);
				::i2c_rdwr_ioctl_data d{
				.msgs = std::ranges::data(s),
				.nmsgs = static_cast<std::uint32_t>(std::ranges::size(s)),
				};
				int r = ::ioctl(fd, I2C_RDWR, &d);
				if (r < 0) throw std::system_error(errno, std::generic_category(), "ioctl(I2C_RDWR)");
				return static_cast<std::size_t>(r);
			});
	}

private:
	int fd_;
	std::uint16_t addr_;
};

}  // namespace ibus::ioctl_i2c
