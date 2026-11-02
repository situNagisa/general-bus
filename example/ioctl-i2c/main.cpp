#include <ibus/ioctl_i2c.hpp>

// 从 0x50 的寄存器 0x10 开始读 16 字节：先写寄存器地址，再读（一次 I2C_RDWR）。
// 用法：./ioctl-i2c [/dev/i2c-N]

#include <array>
#include <cstdio>
#include <exception>
#include <span>

// 用户自己的 submission：上限 N，运行期计数；满足 ibus::ioctl_i2c::submission。
template <std::size_t N>
class msg_batch {
public:
	explicit msg_batch(std::uint16_t addr) noexcept : addr_(addr) {}

	void send(std::span<std::byte const> d) { push(d, 0); }
	void receive(std::span<std::byte> d) { push(d, I2C_M_RD); }

	::i2c_msg* begin() noexcept { return msgs_.data(); }
	::i2c_msg* end() noexcept { return msgs_.data() + count_; }

private:
	void push(std::span<std::byte const> d, std::uint16_t flags) {
		msgs_[count_++] = ::i2c_msg{
			.addr = addr_,
			.flags = flags,
			.len = static_cast<std::uint16_t>(d.size()),
			.buf = reinterpret_cast<std::uint8_t*>(const_cast<std::byte*>(d.data())),
		};
	}

	std::uint16_t addr_;
	std::array<::i2c_msg, N> msgs_;
	std::size_t count_ = 0;
};

static_assert(ibus::ioctl_i2c::submission<msg_batch<2>>);

int main(int argc, char** argv) {
	using namespace ibus;
	try {
		ioctl_i2c::link l(argc > 1 ? argv[1] : "/dev/i2c-1", 0x50);
		std::byte reg[1]{std::byte{0x10}}, data[16]{};

		msg_batch<2> s(l.address());
		ibus::send(s, std::span<std::byte const>(reg));
		ibus::receive(s, std::span<std::byte>(data));

		auto [n] = stdexec::sync_wait(communicate<kind::submit>(l, s)).value();
		std::printf("done %zu msgs\n", n);
	} catch (std::exception const& e) {
		std::printf("error: %s\n", e.what());
	}
}
