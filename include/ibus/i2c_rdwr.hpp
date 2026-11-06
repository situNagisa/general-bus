#pragma once

#include <ibus/cpo.hpp>
#include <ibus/sender.hpp>

#include <cstddef>
#include <cstdint>
#include <ranges>
#include <span>
#include <type_traits>

#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <sys/ioctl.h>

namespace ibus::i2c_rdwr {

// concept：send_pack / receive_pack(从机地址, 数据, flags) 返回 i2c_msg；communicate_range(i2c_msg 的连续区间) 一次 I2C_RDWR，
// 完成值是完成的消息条数。
// transaction 的参考 receiver：正好接得住 transaction 里各组合发出的完成；用户的 receiver 照这个写，作为 transaction 的第二个参数检查。
struct transaction_receiver {
	using receiver_concept = stdexec::receiver_t;
	void set_value(std::size_t) && noexcept {}
	void set_error(int) && noexcept {}
};
template <class P, class R = transaction_receiver>
concept transaction = requires(P& p, std::uint16_t addr, std::span<std::byte const> out, std::span<std::byte> in, std::uint16_t flags,
	std::span<::i2c_msg> msgs) {
	{ send_pack(p, addr, out, flags) } -> std::same_as<::i2c_msg>;
	{ receive_pack(p, addr, in, flags) } -> std::same_as<::i2c_msg>;
	requires stdexec::sender_to<decltype(communicate_range(p, msgs)), R>;
};

// port：已打开的 /dev/i2c-N（由用户打开、关闭），走 ioctl(I2C_RDWR)。返回 sender：start 时一次 ioctl，失败 set_error(errno)。
// send_pack / receive_pack 直接返回 i2c_msg；从机地址和每条消息的 flags 是它们的参数，同一个事务里可以访问多个从机。
// 用户把任意多个放进一段连续内存，communicate_range 一次交给 I2C_RDWR：相邻消息之间 repeated START，最后 STOP。
// flags：I2C_M_TEN 10 位地址；I2C_M_NOSTART 不发 START 和地址，接在上一条后面（需 I2C_FUNC_NOSTART）；
//        I2C_M_IGNORE_NAK 忽略从机 NACK；I2C_M_REV_DIR_ADDR 地址的读写位取反；I2C_M_NO_RD_ACK 读时不发 ACK；
//        I2C_M_RECV_LEN（只用于接收）读到的第一个字节决定还要读多少。
struct port {
	int fd;

	::i2c_msg send_pack(std::uint16_t addr, std::span<std::byte const> d, std::uint16_t flags = 0) const noexcept { return msg(addr, d, flags); }
	::i2c_msg receive_pack(std::uint16_t addr, std::span<std::byte> d, std::uint16_t flags = 0) const noexcept {
		return msg(addr, d, static_cast<std::uint16_t>(I2C_M_RD | flags));
	}

private:
	auto _rdwr(auto m) const noexcept(std::is_nothrow_move_constructible_v<decltype(m)>) {
		return detail::syscall<std::size_t>([fd = fd, m = std::move(m)] mutable noexcept {
			::i2c_rdwr_ioctl_data d{.msgs = std::ranges::data(m), .nmsgs = static_cast<std::uint32_t>(std::ranges::size(m))};
			return ::ioctl(fd, I2C_RDWR, &d);
		});
	}

public:
	// 动态 n 条：一段连续的 i2c_msg，原地交给一次 I2C_RDWR。完成值是完成的条数。
	// 左值 / 视图：只记地址，元素由用户保证活到完成。
	auto communicate_range(contiguous_range_of<::i2c_msg> auto&& r) const noexcept
		requires std::ranges::borrowed_range<decltype(r)>
	{
		return _rdwr(std::span<::i2c_msg>(r));
	}

	// 持有元素的右值（临时 array / vector）：移进 sender。
	auto communicate_range(contiguous_range_of<::i2c_msg> auto&& r) const
		requires(!std::ranges::borrowed_range<decltype(r)>)
	{
		return _rdwr(std::move(r));
	}

private:
	static ::i2c_msg msg(std::uint16_t addr, std::span<std::byte const> d, std::uint16_t flags) noexcept {
		return {
			.addr = addr,
			.flags = flags,
			.len = static_cast<std::uint16_t>(d.size()),
			.buf = reinterpret_cast<std::uint8_t*>(const_cast<std::byte*>(d.data())),
		};
	}
};

static_assert(transaction<port>);

}  // namespace ibus::i2c_rdwr
