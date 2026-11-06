#pragma once

#include <ibus/cpo.hpp>
#include <ibus/sender.hpp>

#include <array>
#include <cassert>
#include <cerrno>
#include <concepts>
#include <cstddef>
#include <cstdint>
#include <span>
#include <type_traits>

#include <linux/spi/spidev.h>
#include <sys/ioctl.h>
#include <unistd.h>

namespace ibus::spidev {

// 段数在编译期确定的一段 spi_ioc_transfer：std::span<spi_ioc_transfer, N>（N 不是 dynamic_extent）。
template <class S>
concept static_transfers = requires { S::extent; } && (S::extent != std::dynamic_extent)
	&& std::same_as<std::remove_const_t<typename S::element_type>, ::spi_ioc_transfer>;

// concept：send_pack(data) 一次 write、receive_pack(data) 一次 read（各是一次独立片选），完成值是字节数。
// read_write 的参考 receiver：正好接得住 read_write 里各组合发出的完成；用户的 receiver 照这个写，作为 read_write 的第二个参数检查。
struct read_write_receiver {
	using receiver_concept = stdexec::receiver_t;
	void set_value(std::size_t) && noexcept {}
	void set_error(int) && noexcept {}
};
template <class P, class R = read_write_receiver>
concept read_write = requires(P& p, std::span<std::byte const> out, std::span<std::byte> in) {
	requires stdexec::sender_to<decltype(communicate(p, send_pack(p, out))), R>;
	requires stdexec::sender_to<decltype(communicate(p, receive_pack(p, in))), R>;
};

// concept：SPI_IOC_MESSAGE。send_pack / receive_pack / transceive_pack(数据, speed_hz, cs_change, delay_usecs, bits_per_word, nbits, word_delay_usecs)
// 返回 spi_ioc_transfer；communicate_range(span<spi_ioc_transfer, N>) 把 N 段放进同一次片选，完成值是总字节数。
// message 的参考 receiver：正好接得住 message 里各组合发出的完成；用户的 receiver 照这个写，作为 message 的第二个参数检查。
struct message_receiver {
	using receiver_concept = stdexec::receiver_t;
	void set_value(std::size_t) && noexcept {}
	void set_error(int) && noexcept {}
};
template <class P, class R = message_receiver>
concept message = requires(P& p, std::span<std::byte const> out, std::span<std::byte> in, std::uint32_t speed_hz, bool cs_change,
	std::uint16_t delay_usecs, std::uint8_t bits_per_word, std::uint8_t nbits, std::uint8_t word_delay_usecs,
	std::span<::spi_ioc_transfer, 1> one, std::span<::spi_ioc_transfer, 2> two) {
	{ send_pack(p, out, speed_hz, cs_change, delay_usecs, bits_per_word, nbits, word_delay_usecs) } -> std::same_as<::spi_ioc_transfer>;
	{ receive_pack(p, in, speed_hz, cs_change, delay_usecs, bits_per_word, nbits, word_delay_usecs) } -> std::same_as<::spi_ioc_transfer>;
	{ transceive_pack(p, out, in, speed_hz, cs_change, delay_usecs, bits_per_word, nbits, word_delay_usecs) } -> std::same_as<::spi_ioc_transfer>;
	requires stdexec::sender_to<decltype(communicate_range(p, one)), R>;
	requires stdexec::sender_to<decltype(communicate_range(p, two)), R>;
};

// port：一个 SPI 从机（已打开的 /dev/spidevB.C，由用户打开、配置模式和默认速率、关闭）。
// 每次 communicate 是一次独立的片选（read / write）。返回 sender：start 时一次系统调用，完成值是字节数，失败 set_error(errno)。
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

	// 1 段 send：write，MISO 上收回来的丢掉。返回发出的字节数。
	auto communicate(send_packet p) const noexcept {
		return detail::syscall<std::size_t>([fd = fd, p] noexcept { return ::write(fd, p.data.data(), p.data.size()); });
	}

	// 1 段 receive：read，MOSI 上发 0。返回收到的字节数。
	auto communicate(receive_packet p) const noexcept {
		return detail::syscall<std::size_t>([fd = fd, p] noexcept { return ::read(fd, p.data.data(), p.data.size()); });
	}
};

// message_port：同一个 SPI 从机，走 SPI_IOC_MESSAGE(n)。n 是请求号的一部分，必须是编译期常量，
// 所以这是静态 n 段：communicate(pack...) 的每个参数是一段，全部放在同一次片选里，一次 ioctl。
// send_pack / receive_pack / transceive_pack 返回 spi_ioc_transfer；每段的属性是它们的参数（0 表示用 fd 的默认值）：
//   speed_hz          这一段的时钟
//   cs_change         这一段结束后临时释放片选再开始下一段；放在最后一段时，表示 ioctl 返回后保持片选有效（取决于驱动）
//   delay_usecs       这一段结束后等多久再开始下一段 / 改变片选
//   bits_per_word     字长
//   nbits             线数（1：单线，2：双线，4：四线，8：八线）
//   word_delay_usecs  字与字之间的延时
struct message_port {
	int fd;

	::spi_ioc_transfer send_pack(std::span<std::byte const> d, std::uint32_t speed_hz = 0, bool cs_change = false, std::uint16_t delay_usecs = 0,
		std::uint8_t bits_per_word = 0, std::uint8_t nbits = 0, std::uint8_t word_delay_usecs = 0) const noexcept {
		::spi_ioc_transfer x = segment(d.size(), speed_hz, cs_change, delay_usecs, bits_per_word, word_delay_usecs);
		x.tx_buf = reinterpret_cast<std::uintptr_t>(d.data());
		x.tx_nbits = nbits;
		return x;
	}

	::spi_ioc_transfer receive_pack(std::span<std::byte> d, std::uint32_t speed_hz = 0, bool cs_change = false, std::uint16_t delay_usecs = 0,
		std::uint8_t bits_per_word = 0, std::uint8_t nbits = 0, std::uint8_t word_delay_usecs = 0) const noexcept {
		::spi_ioc_transfer x = segment(d.size(), speed_hz, cs_change, delay_usecs, bits_per_word, word_delay_usecs);
		x.rx_buf = reinterpret_cast<std::uintptr_t>(d.data());
		x.rx_nbits = nbits;
		return x;
	}

	// 全双工：同一段时钟里发 tx、收 rx，两者等长。
	::spi_ioc_transfer transceive_pack(std::span<std::byte const> tx, std::span<std::byte> rx, std::uint32_t speed_hz = 0, bool cs_change = false,
		std::uint16_t delay_usecs = 0, std::uint8_t bits_per_word = 0, std::uint8_t nbits = 0, std::uint8_t word_delay_usecs = 0) const noexcept {
		assert(tx.size() == rx.size() && "spidev: transceive tx/rx length mismatch");
		::spi_ioc_transfer x = segment(rx.size(), speed_hz, cs_change, delay_usecs, bits_per_word, word_delay_usecs);
		x.tx_buf = reinterpret_cast<std::uintptr_t>(tx.data());
		x.rx_buf = reinterpret_cast<std::uintptr_t>(rx.data());
		x.tx_nbits = nbits;
		x.rx_nbits = nbits;
		return x;
	}

	// 静态 n 段：用户自己的 spi_ioc_transfer[N]（在里面原地填好），一次 SPI_IOC_MESSAGE(N)，不拷贝。返回总字节数。
	auto communicate_range(static_transfers auto x) const noexcept {
		return detail::syscall<std::size_t>([fd = fd, x] noexcept { return ::ioctl(fd, SPI_IOC_MESSAGE(decltype(x)::extent), x.data()); });
	}

	// 便捷写法：每个参数是一段，放进 sender 里的一个数组（比直接用 communicate_range 多一次复制）。
	auto communicate(std::same_as<::spi_ioc_transfer> auto... x) const noexcept {
		return detail::syscall<std::size_t>([fd = fd, a = std::array<::spi_ioc_transfer, sizeof...(x)>{x...}] noexcept {
			return ::ioctl(fd, SPI_IOC_MESSAGE(sizeof...(x)), a.data());
		});
	}

private:
	static ::spi_ioc_transfer segment(std::size_t len, std::uint32_t speed_hz, bool cs_change, std::uint16_t delay_usecs,
		std::uint8_t bits_per_word, std::uint8_t word_delay_usecs) noexcept {
		::spi_ioc_transfer x{};
		x.len = static_cast<std::uint32_t>(len);
		x.speed_hz = speed_hz;
		x.delay_usecs = delay_usecs;
		x.bits_per_word = bits_per_word;
		x.cs_change = cs_change;
		x.word_delay_usecs = word_delay_usecs;
		return x;
	}
};

static_assert(read_write<port> && message<message_port>);

}  // namespace ibus::spidev
