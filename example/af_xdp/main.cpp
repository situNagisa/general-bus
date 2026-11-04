#include <ibus/af_xdp.hpp>

// 先编排、再一次提交：AF_XDP 的四个共享环。
//   fill_link：把空闲 UMEM 块借给内核（接收用）；rx_link：取收到的包（UMEM 偏移 + 长度）
//   tx_link：send_pack 返回 xdp_desc，communicate_range 写进 TX 环 + sendto 唤醒；completion_link：取发完可复用的块
// link 不持有资源：socket、UMEM 注册、环的 setsockopt / mmap、bind 都由这里做。需要 root。
// 用法：sudo ./af_xdp [网卡名，默认 lo]

#include <array>
#include <cstdio>
#include <cstring>
#include <exception>
#include <span>
#include <system_error>

#include <net/if.h>
#include <sys/mman.h>
#include <unistd.h>

int main(int argc, char** argv) {
	using namespace ibus;
	char const* ifname = argc > 1 ? argv[1] : "lo";
	constexpr std::uint32_t frames = 64, frame_size = 2048, ring_size = 64;
	try {
		int xsk = ::socket(AF_XDP, SOCK_RAW | SOCK_CLOEXEC, 0);
		if (xsk < 0) throw std::system_error(errno, std::generic_category(), "socket(AF_XDP)");

		void* umem = ::mmap(nullptr, frames * frame_size, PROT_READ | PROT_WRITE, MAP_PRIVATE | MAP_ANONYMOUS, -1, 0);
		::xdp_umem_reg reg{};
		reg.addr = reinterpret_cast<std::uint64_t>(umem);
		reg.len = frames * frame_size;
		reg.chunk_size = frame_size;
		if (::setsockopt(xsk, SOL_XDP, XDP_UMEM_REG, &reg, sizeof reg) < 0) throw std::system_error(errno, std::generic_category(), "XDP_UMEM_REG");
		std::uint32_t rs = ring_size;
		for (int opt : {XDP_UMEM_FILL_RING, XDP_UMEM_COMPLETION_RING, XDP_RX_RING, XDP_TX_RING})
			if (::setsockopt(xsk, SOL_XDP, opt, &rs, sizeof rs) < 0) throw std::system_error(errno, std::generic_category(), "XDP ring");

		::xdp_mmap_offsets off{};
		::socklen_t ol = sizeof off;
		::getsockopt(xsk, SOL_XDP, XDP_MMAP_OFFSETS, &off, &ol);
		auto map = [&](std::uint64_t pgoff, ::xdp_ring_offset const& o, std::size_t elem) {
			void* p = ::mmap(nullptr, o.desc + ring_size * elem, PROT_READ | PROT_WRITE, MAP_SHARED | MAP_POPULATE, xsk, static_cast<::off_t>(pgoff));
			if (p == MAP_FAILED) throw std::system_error(errno, std::generic_category(), "mmap ring");
			return p;
		};
		void* fill_m = map(XDP_UMEM_PGOFF_FILL_RING, off.fr, sizeof(std::uint64_t));
		void* comp_m = map(XDP_UMEM_PGOFF_COMPLETION_RING, off.cr, sizeof(std::uint64_t));
		void* rx_m = map(XDP_PGOFF_RX_RING, off.rx, sizeof(::xdp_desc));
		void* tx_m = map(XDP_PGOFF_TX_RING, off.tx, sizeof(::xdp_desc));

		::sockaddr_xdp sa{};
		sa.sxdp_family = AF_XDP;
		sa.sxdp_ifindex = ::if_nametoindex(ifname);
		sa.sxdp_queue_id = 0;
		sa.sxdp_flags = XDP_COPY;
		if (::bind(xsk, reinterpret_cast<::sockaddr*>(&sa), sizeof sa) < 0) throw std::system_error(errno, std::generic_category(), "bind(AF_XDP)");

		af_xdp::fill_link fill(af_xdp::ring<std::uint64_t>::at(fill_m, off.fr, ring_size));
		af_xdp::tx_link tx(xsk, af_xdp::ring<::xdp_desc>::at(tx_m, off.tx, ring_size));
		af_xdp::completion_link comp(af_xdp::ring<std::uint64_t>::at(comp_m, off.cr, ring_size));
		af_xdp::rx_link rx(af_xdp::ring<::xdp_desc>::at(rx_m, off.rx, ring_size));

		// 前一半块借给内核接收
		std::array<std::uint64_t, frames / 2> fa;
		for (std::uint32_t i = 0; i < fa.size(); ++i) fa[i] = send_pack(fill, i * frame_size);
		communicate_range(fill, fa);

		// 后一半块用来发：写一个 64 字节的帧（内容不重要，lo 上 XDP_COPY 模式能发出）
		std::uint64_t at = (frames / 2) * frame_size;
		std::memset(static_cast<std::byte*>(umem) + at, 0xff, 64);
		std::array<::xdp_desc, 1> td{send_pack(tx, at, 64)};
		communicate_range(tx, td);

		std::array<std::uint64_t, 4> done{};
		std::size_t k = 0;
		for (int i = 0; i < 1000 && k == 0; ++i) k = communicate(comp, receive_pack(comp, std::span(done)));
		std::printf("tx completed %zu frame(s)\n", k);
		std::array<::xdp_desc, 4> got{};
		std::printf("rx got %zu frame(s)（需要 XDP 程序把包重定向到这个 socket，这里没有加载）\n", communicate(rx, receive_pack(rx, std::span(got))));
		::close(xsk);
	} catch (std::exception const& e) {
		std::printf("error: %s\n", e.what());
	}
}
