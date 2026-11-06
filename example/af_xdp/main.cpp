#include <ibus/af_xdp.hpp>

// 先编排、再一次提交：AF_XDP 的四个共享环。
// 当作协议直接读写环：communicate(send_pack(desc)) 写进下一个槽位，communicate(send_pack(af_xdp::publish)) 发布；
// communicate(receive_pack()) 取下一个描述（指向环里，没有是 nullptr），communicate(send_pack(af_xdp::publish)) 归还。
//   fill_port：把空闲 UMEM 块借给内核（接收用）；rx_port：取收到的包（UMEM 偏移 + 长度）
//   tx_port：写 TX 环，发布时 sendto 唤醒内核；completion_port：取发完可复用的块
// port 不持有资源：socket、UMEM 注册、环的 setsockopt / mmap、bind 都由这里做。需要 root。
// 用法：sudo ./af_xdp [网卡名，默认 lo]

#include <array>
#include <cstdio>
#include <tuple>
#include <cstring>
#include <exception>
#include <span>
#include <system_error>

#include <net/if.h>
#include <sys/mman.h>
#include <unistd.h>

// sync_wait 取 sender 唯一的完成值（set_error(error_code) 时 sync_wait 抛 system_error）。
static auto value(auto&& s) { return std::get<0>(stdexec::sync_wait(std::forward<decltype(s)>(s)).value()); }

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

		auto fill = af_xdp::fill_port::at(fill_m, off.fr, ring_size);
		af_xdp::tx_port tx{af_xdp::producer_port<::xdp_desc>::at(tx_m, off.tx, ring_size), xsk};
		auto comp = af_xdp::completion_port::at(comp_m, off.cr, ring_size);
		auto rx = af_xdp::rx_port::at(rx_m, off.rx, ring_size);

		// 前一半块借给内核接收
		for (std::uint32_t i = 0; i < frames / 2; ++i) stdexec::sync_wait(communicate(fill, send_pack(fill, std::uint64_t{i * frame_size})));
		stdexec::sync_wait(communicate(fill, send_pack(fill, af_xdp::publish)));

		// 后一半块用来发：写一个 64 字节的帧（内容不重要，lo 上 XDP_COPY 模式能发出）
		std::uint64_t at = (frames / 2) * frame_size;
		std::memset(static_cast<std::byte*>(umem) + at, 0xff, 64);
		stdexec::sync_wait(communicate(tx, send_pack(tx, ::xdp_desc{at, 64, 0})));
		stdexec::sync_wait(communicate(tx, send_pack(tx, af_xdp::publish)));

		// completion 环：内核发完会把块的偏移放回来（XDP_COPY 下 sendto 返回时通常已经完成）
		if (auto* done = value(communicate(comp, receive_pack(comp)))) {
			std::printf("tx completed: umem offset %llu\n", static_cast<unsigned long long>(*done));
			stdexec::sync_wait(communicate(comp, send_pack(comp, af_xdp::publish)));
		} else {
			std::printf("tx not completed yet\n");
		}
		std::printf("rx got a frame: %s（需要 XDP 程序把包重定向到这个 socket，这里没有加载）\n", value(communicate(rx, receive_pack(rx))) ? "yes" : "no");
		::close(xsk);
	} catch (std::exception const& e) {
		std::printf("error: %s\n", e.what());
	}
}
