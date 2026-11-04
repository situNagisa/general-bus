#pragma once

#include <ibus/cpo.hpp>

#include <cerrno>
#include <cstddef>
#include <cstdint>
#include <ranges>
#include <span>
#include <system_error>

#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <sys/ioctl.h>

namespace ibus::i2c_rdwr {

// link：已打开的 /dev/i2c-N（由用户打开、关闭）+ 从机地址，走 ioctl(I2C_RDWR)。同步，直接返回结果。
// send_pack / receive_pack 直接返回 i2c_msg（地址、方向已填好）；用户把任意多个放进一段连续内存，
// communicate_range 一次交给 I2C_RDWR：相邻消息之间 repeated START，最后 STOP。
class link {
public:
	link(int fd, std::uint16_t addr) noexcept : fd_(fd), addr_(addr) {}

	// 打包时附加到每条消息的 flags（打包时读取当前值）：
	//   I2C_M_TEN 10 位地址；I2C_M_NOSTART 不发 START 和地址，接在上一条后面（需 I2C_FUNC_NOSTART）；
	//   I2C_M_IGNORE_NAK 忽略从机 NACK；I2C_M_REV_DIR_ADDR 地址的读写位取反；I2C_M_NO_RD_ACK 读时不发 ACK；
	//   I2C_M_RECV_LEN（只用于接收）读到的第一个字节决定还要读多少。
	void set_send_flags(std::uint16_t v) noexcept { cfg_.send_flags = v; }
	void set_receive_flags(std::uint16_t v) noexcept { cfg_.receive_flags = v; }

	::i2c_msg send_pack(std::span<std::byte const> d) const noexcept { return msg(d, cfg_.send_flags); }
	::i2c_msg receive_pack(std::span<std::byte> d) const noexcept { return msg(d, I2C_M_RD | cfg_.receive_flags); }

	// 动态 n 条：一段连续的 i2c_msg，原地交给一次 I2C_RDWR。返回完成的条数。
	std::size_t communicate_range(auto&& msgs) const {
		::i2c_rdwr_ioctl_data d{
			.msgs = std::ranges::data(msgs),
			.nmsgs = static_cast<std::uint32_t>(std::ranges::size(msgs)),
		};
		int n = ::ioctl(fd_, I2C_RDWR, &d);
		if (n < 0) throw std::system_error(errno, std::generic_category(), "ioctl(I2C_RDWR)");
		return static_cast<std::size_t>(n);
	}

private:
	struct config {
		std::uint16_t send_flags = 0;
		std::uint16_t receive_flags = 0;
	};

	::i2c_msg msg(std::span<std::byte const> d, unsigned flags) const noexcept {
		return {
			.addr = addr_,
			.flags = static_cast<std::uint16_t>(flags),
			.len = static_cast<std::uint16_t>(d.size()),
			.buf = reinterpret_cast<std::uint8_t*>(const_cast<std::byte*>(d.data())),
		};
	}

	int fd_;
	std::uint16_t addr_;
	config cfg_;
};

}  // namespace ibus::i2c_rdwr
