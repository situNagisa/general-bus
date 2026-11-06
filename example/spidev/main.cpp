#include <ibus/spidev.hpp>

// spidev::message_port：communicate(pack...) 把静态 n 段放进同一次片选（SPI_IOC_MESSAGE(n)，n 是编译期常量）。
//   send_pack / receive_pack / transceive_pack（全双工）；速率、cs_change 等每段属性是 pack 的参数。
// port 不持有 fd：设备由这里打开、关闭。
// spidev::port 的 communicate：send_pack 是 write、receive_pack 是 read，每次一个独立片选。
// 注意：SPI flash 读 ID 这类“先发命令、同一片选里再收”的事务，拆成两次会在中间释放片选，
// 器件会丢弃命令；这里只演示调用方式。
// 用法：./spidev [/dev/spidevB.C]

#include <array>
#include <cstdio>
#include <tuple>
#include <exception>
#include <span>
#include <system_error>

#include <fcntl.h>
#include <unistd.h>

using namespace ibus;

// sync_wait 取 sender 唯一的完成值（set_error(error_code) 时 sync_wait 抛 system_error）。
static auto value(auto&& s) { return std::get<0>(stdexec::sync_wait(std::forward<decltype(s)>(s)).value()); }

int main(int argc, char** argv) {
	try {
		char const* dev = argc > 1 ? argv[1] : "/dev/spidev0.0";
		int fd = ::open(dev, O_RDWR | O_CLOEXEC);
		if (fd < 0) throw std::system_error(errno, std::generic_category(), dev);
		spidev::port l{fd};

		std::array<std::byte, 1> cmd{std::byte{0x9F}};
		std::array<std::byte, 3> id{};
		auto w = value(communicate(l, send_pack(l, std::span<std::byte const>(cmd))));
		auto n = value(communicate(l, receive_pack(l, std::span(id))));
		std::printf("sent %zu, received %zu: %02x %02x %02x\n", w, n,
			static_cast<unsigned>(id[0]), static_cast<unsigned>(id[1]), static_cast<unsigned>(id[2]));

		// 静态 2 段同一次片选：读 SPI flash JEDEC ID（发 0x9F，接着收 3 字节），一次 SPI_IOC_MESSAGE(2)
		spidev::message_port ml{fd};
		std::size_t m = value(communicate(ml, send_pack(ml, std::span<std::byte const>(cmd)), receive_pack(ml, std::span(id))));
		std::printf("message: %zu bytes, ID %02x %02x %02x\n", m,
			static_cast<unsigned>(id[0]), static_cast<unsigned>(id[1]), static_cast<unsigned>(id[2]));

		// 静态 3 段：命令 + 一段全双工 + 收，每段的速率是 pack 的参数，可以各不相同；transceive_pack 一段里同时发收
		std::array<std::byte, 4> tx{std::byte{1}, std::byte{2}, std::byte{3}, std::byte{4}}, rx{};
		std::size_t k = value(communicate(ml, send_pack(ml, std::span<std::byte const>(cmd), 1'000'000),
			transceive_pack(ml, std::span<std::byte const>(tx), std::span(rx), 20'000'000), receive_pack(ml, std::span(id), 1'000'000)));
		std::printf("message: %zu bytes\n", k);

		// 不拷贝的写法：段直接放在自己的数组里，communicate_range(span<spi_ioc_transfer, N>)，N 在编译期确定
		std::array<::spi_ioc_transfer, 2> seg{send_pack(ml, std::span<std::byte const>(cmd)), receive_pack(ml, std::span(id))};
		std::printf("message range: %zu bytes\n", value(communicate_range(ml, std::span(seg))));
		::close(fd);
	} catch (std::exception const& e) {
		std::printf("error: %s\n", e.what());
	}
}
