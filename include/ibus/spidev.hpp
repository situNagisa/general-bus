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

// link：一个 SPI 从机（已打开的 /dev/spidevB.C，由用户打开、配置模式和速率、关闭）。
// 每次 communicate 是一次独立的片选（read / write）。同步，直接返回结果。
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

	// 1 段 send：write，MISO 上收回来的丢掉。返回发出的字节数。
	std::size_t communicate(send_packet p) const {
		::ssize_t r = ::write(fd_, p.data.data(), p.data.size());
		if (r < 0) throw std::system_error(errno, std::generic_category(), "spidev write");
		return static_cast<std::size_t>(r);
	}

	// 1 段 receive：read，MOSI 上发 0。返回收到的字节数。
	std::size_t communicate(receive_packet p) const {
		::ssize_t r = ::read(fd_, p.data.data(), p.data.size());
		if (r < 0) throw std::system_error(errno, std::generic_category(), "spidev read");
		return static_cast<std::size_t>(r);
	}

private:
	int fd_;
};

// message_link：同一个 SPI 从机，走 SPI_IOC_MESSAGE(n)。n 是请求号的一部分，必须是编译期常量，
// 所以这是静态 n 段：communicate(pack...) 的每个参数是一段，全部放在同一次片选里，一次 ioctl。
// send_pack / receive_pack / transceive_pack 返回 spi_ioc_transfer（只填缓冲和长度）；
// 速率、延时等每段属性在发送时按当前 config 填入每一段。
class message_link {
public:
	explicit message_link(int fd) noexcept : fd_(fd) {}

	void set_speed_hz(std::uint32_t v) noexcept { cfg_.speed_hz = v; }  // 0：用 fd 的默认速率
	void set_delay_usecs(std::uint16_t v) noexcept { cfg_.delay_usecs = v; }  // 每段结束后等多久再开始下一段 / 改变片选
	void set_bits_per_word(std::uint8_t v) noexcept { cfg_.bits_per_word = v; }  // 0：用 fd 的默认字长
	void set_tx_nbits(std::uint8_t v) noexcept { cfg_.tx_nbits = v; }  // 发送线数（0/1：单线，2：双线，4：四线，8：八线）
	void set_rx_nbits(std::uint8_t v) noexcept { cfg_.rx_nbits = v; }  // 接收线数
	void set_word_delay_usecs(std::uint8_t v) noexcept { cfg_.word_delay_usecs = v; }  // 字与字之间的延时
	void set_cs_change(bool v) noexcept { cfg_.cs_change = v; }  // 段与段之间临时释放片选
	void set_keep_cs(bool v) noexcept { cfg_.keep_cs = v; }  // ioctl 返回后保持片选有效（最后一段 cs_change=1，取决于驱动）

	::spi_ioc_transfer send_pack(std::span<std::byte const> d) const noexcept {
		::spi_ioc_transfer x{};
		x.tx_buf = reinterpret_cast<std::uintptr_t>(d.data());
		x.len = static_cast<std::uint32_t>(d.size());
		return x;
	}
	::spi_ioc_transfer receive_pack(std::span<std::byte> d) const noexcept {
		::spi_ioc_transfer x{};
		x.rx_buf = reinterpret_cast<std::uintptr_t>(d.data());
		x.len = static_cast<std::uint32_t>(d.size());
		return x;
	}
	// 全双工：同一段时钟里发 tx、收 rx，两者等长。
	::spi_ioc_transfer transceive_pack(std::span<std::byte const> tx, std::span<std::byte> rx) const noexcept {
		assert(tx.size() == rx.size() && "spidev: transceive tx/rx length mismatch");
		::spi_ioc_transfer x{};
		x.tx_buf = reinterpret_cast<std::uintptr_t>(tx.data());
		x.rx_buf = reinterpret_cast<std::uintptr_t>(rx.data());
		x.len = static_cast<std::uint32_t>(rx.size());
		return x;
	}

	// 静态 n 段：spi_ioc_transfer[n] 原地构造（填入 config），一次 SPI_IOC_MESSAGE(n)。返回总字节数。
	std::size_t communicate(std::same_as<::spi_ioc_transfer> auto... x) const {
		std::array<::spi_ioc_transfer, sizeof...(x)> a{x...};
		for (std::size_t i = 0; i < a.size(); ++i) {
			a[i].speed_hz = cfg_.speed_hz;
			a[i].delay_usecs = cfg_.delay_usecs;
			a[i].bits_per_word = cfg_.bits_per_word;
			a[i].tx_nbits = cfg_.tx_nbits;
			a[i].rx_nbits = cfg_.rx_nbits;
			a[i].word_delay_usecs = cfg_.word_delay_usecs;
			a[i].cs_change = i + 1 < a.size() ? cfg_.cs_change : cfg_.keep_cs;
		}
		int r = ::ioctl(fd_, SPI_IOC_MESSAGE(sizeof...(x)), a.data());
		if (r < 0) throw std::system_error(errno, std::generic_category(), "ioctl(SPI_IOC_MESSAGE)");
		return static_cast<std::size_t>(r);
	}

private:
	struct config {
		std::uint32_t speed_hz = 0;        // 0：用 fd 的默认速率
		std::uint16_t delay_usecs = 0;     // 每段结束后等多久再开始下一段 / 改变片选
		std::uint8_t bits_per_word = 0;    // 0：用 fd 的默认字长
		std::uint8_t tx_nbits = 0;         // 发送线数（0/1：单线，2：双线，4：四线，8：八线）
		std::uint8_t rx_nbits = 0;         // 接收线数
		std::uint8_t word_delay_usecs = 0; // 字与字之间的延时
		bool cs_change = false;            // 段与段之间临时释放片选
		bool keep_cs = false;              // ioctl 返回后保持片选有效（最后一段 cs_change=1，取决于驱动）
	};

	int fd_;
	config cfg_;
};

}  // namespace ibus::spidev
