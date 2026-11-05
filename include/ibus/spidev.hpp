#pragma once

#include <ibus/cpo.hpp>

#include <array>
#include <cassert>
#include <cerrno>
#include <concepts>
#include <cstddef>
#include <cstdint>
#include <span>
#include <system_error>

#include <linux/spi/spidev.h>
#include <sys/ioctl.h>
#include <unistd.h>

namespace ibus::spidev {

// port：一个 SPI 从机（已打开的 /dev/spidevB.C，由用户打开、配置模式和默认速率、关闭）。
// 每次 communicate 是一次独立的片选（read / write）。同步，直接返回结果。
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
	std::size_t communicate(send_packet p) const {
		::ssize_t r = ::write(fd, p.data.data(), p.data.size());
		if (r < 0) throw std::system_error(errno, std::generic_category(), "spidev write");
		return static_cast<std::size_t>(r);
	}

	// 1 段 receive：read，MOSI 上发 0。返回收到的字节数。
	std::size_t communicate(receive_packet p) const {
		::ssize_t r = ::read(fd, p.data.data(), p.data.size());
		if (r < 0) throw std::system_error(errno, std::generic_category(), "spidev read");
		return static_cast<std::size_t>(r);
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

	// 静态 n 段：spi_ioc_transfer[n] 原地构造，一次 SPI_IOC_MESSAGE(n)。返回总字节数。
	std::size_t communicate(std::same_as<::spi_ioc_transfer> auto... x) const {
		std::array<::spi_ioc_transfer, sizeof...(x)> a{x...};
		int r = ::ioctl(fd, SPI_IOC_MESSAGE(sizeof...(x)), a.data());
		if (r < 0) throw std::system_error(errno, std::generic_category(), "ioctl(SPI_IOC_MESSAGE)");
		return static_cast<std::size_t>(r);
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

}  // namespace ibus::spidev
