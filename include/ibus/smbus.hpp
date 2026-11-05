#pragma once

#include <ibus/cpo.hpp>

#include <cassert>
#include <cerrno>
#include <cstddef>
#include <cstdint>
#include <span>
#include <system_error>

#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <sys/ioctl.h>

namespace ibus::smbus {

// 命令字节（通常是寄存器号），和 1 字节数据区分开。
struct command {
	std::uint8_t value;
};

// port：已打开的 /dev/i2c-N，且已 ioctl(I2C_SLAVE) 绑定从机地址（都由用户做）。走 ioctl(I2C_SMBUS)。
// SMBus 规定了固定格式的事务，每种格式是 communicate 的一个重载（静态段数）：
//   (send uint8)                            send byte            ：发 1 字节
//   (receive uint8&)                        receive byte         ：收 1 字节
//   (send command, send uint8/uint16)       write byte/word data ：发命令 + 1/2 字节
//   (send command, receive uint8&/uint16&)  read byte/word data  ：发命令 + 收 1/2 字节
//   (send command, send i2c_smbus_data)     block write          ：发命令 + 长度 + 最多 32 字节（block[0] 是长度）
//   (send command, receive i2c_smbus_data&) block read           ：发命令 + 由从机给出长度 + 数据（内核直接写进去）
// PEC 开关是 fd 上的 ioctl(I2C_PEC)，由用户做。同步，直接返回结果。
struct port {
	int fd;

	struct command_packet {
		std::uint8_t command;
	};
	struct byte_out_packet {
		std::uint8_t value;
	};
	struct word_out_packet {
		std::uint16_t value;
	};
	struct block_out_packet {
		::i2c_smbus_data const* data;  // block[0] 是长度（最多 32），block[1..] 是数据，由用户填好
	};
	struct byte_in_packet {
		std::uint8_t* out;
	};
	struct word_in_packet {
		std::uint16_t* out;
	};
	struct block_in_packet {
		::i2c_smbus_data* out;  // 内核写回：block[0] 是长度，block[1..] 是数据
	};

	command_packet send_pack(command c) const noexcept { return {c.value}; }
	byte_out_packet send_pack(std::uint8_t value) const noexcept { return {value}; }
	word_out_packet send_pack(std::uint16_t value) const noexcept { return {value}; }
	block_out_packet send_pack(::i2c_smbus_data const& d) const noexcept { return {&d}; }
	byte_in_packet receive_pack(std::uint8_t& out) const noexcept { return {&out}; }
	word_in_packet receive_pack(std::uint16_t& out) const noexcept { return {&out}; }
	block_in_packet receive_pack(::i2c_smbus_data& out) const noexcept { return {&out}; }

	// send byte：只发 1 字节（放在 command 字段里）。
	void communicate(byte_out_packet v) const { _xfer(I2C_SMBUS_WRITE, v.value, I2C_SMBUS_BYTE, nullptr); }

	// receive byte：只收 1 字节。
	void communicate(byte_in_packet in) const {
		::i2c_smbus_data d{};
		_xfer(I2C_SMBUS_READ, 0, I2C_SMBUS_BYTE, &d);
		*in.out = d.byte;
	}

	// write byte data：命令 + 1 字节。
	void communicate(command_packet c, byte_out_packet v) const {
		::i2c_smbus_data d{};
		d.byte = v.value;
		_xfer(I2C_SMBUS_WRITE, c.command, I2C_SMBUS_BYTE_DATA, &d);
	}

	// write word data：命令 + 2 字节。
	void communicate(command_packet c, word_out_packet v) const {
		::i2c_smbus_data d{};
		d.word = v.value;
		_xfer(I2C_SMBUS_WRITE, c.command, I2C_SMBUS_WORD_DATA, &d);
	}

	// read byte data：命令 + 收 1 字节。
	void communicate(command_packet c, byte_in_packet in) const {
		::i2c_smbus_data d{};
		_xfer(I2C_SMBUS_READ, c.command, I2C_SMBUS_BYTE_DATA, &d);
		*in.out = d.byte;
	}

	// read word data：命令 + 收 2 字节。
	void communicate(command_packet c, word_in_packet in) const {
		::i2c_smbus_data d{};
		_xfer(I2C_SMBUS_READ, c.command, I2C_SMBUS_WORD_DATA, &d);
		*in.out = d.word;
	}

	// block write：命令 + 长度 + 数据（最多 32 字节），数据就在用户的 i2c_smbus_data 里。
	void communicate(command_packet c, block_out_packet b) const {
		assert(b.data->block[0] <= I2C_SMBUS_BLOCK_MAX && "smbus: block longer than 32");
		_xfer(I2C_SMBUS_WRITE, c.command, I2C_SMBUS_BLOCK_DATA, const_cast<::i2c_smbus_data*>(b.data));
	}

	// block read：命令 + 从机给出的长度 + 数据，内核直接写进用户的 i2c_smbus_data。
	void communicate(command_packet c, block_in_packet in) const { _xfer(I2C_SMBUS_READ, c.command, I2C_SMBUS_BLOCK_DATA, in.out); }

private:
	void _xfer(std::uint8_t rw, std::uint8_t command, std::uint32_t size, ::i2c_smbus_data* d) const {
		::i2c_smbus_ioctl_data a{.read_write = rw, .command = command, .size = size, .data = d};
		if (::ioctl(fd, I2C_SMBUS, &a) < 0) throw std::system_error(errno, std::generic_category(), "ioctl(I2C_SMBUS)");
	}

};

}  // namespace ibus::smbus
