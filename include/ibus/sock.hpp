#pragma once

#include <ibus/cpo.hpp>

#include <cerrno>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <ranges>
#include <span>
#include <system_error>

#include <fcntl.h>
#include <linux/errqueue.h>
#include <netinet/in.h>
#include <netinet/udp.h>
#include <sys/sendfile.h>
#include <sys/socket.h>
#include <sys/uio.h>

// 所有 link 都不持有 fd：socket 由用户创建、连接、关闭，link 只接收传入。
// link 是对端已确定的通道（已连接的 socket），不处理地址。
// 都是同步系统调用（阻不阻塞看 fd 自己），communicate / communicate_range 直接返回结果。
// 每次调用的 flags 等由 link 内部保存，set_xxx() 修改，调用时用当前值。
namespace ibus::sock {

// 文件区间：sendfile 的数据来源，不经过用户内存。
struct file_region {
	int fd;
	::off_t offset;
	std::size_t count;
};

// pipe 区间：splice 的一端（另一端是 socket），数据不经过用户内存。
struct pipe_region {
	int fd;
	std::size_t count;
};

namespace detail {

inline std::size_t check(::ssize_t r, char const* what) {
	if (r < 0) throw std::system_error(errno, std::generic_category(), what);
	return static_cast<std::size_t>(r);
}

inline ::iovec iov(std::span<std::byte const> d) noexcept { return {const_cast<std::byte*>(d.data()), d.size()}; }

}  // namespace detail

// link：send / recv / sendfile / splice。
class link {
public:
	struct send_packet {
		std::span<std::byte const> data;
	};
	struct sendfile_packet {
		file_region region;
	};
	struct splice_out_packet {
		pipe_region from;
	};
	struct receive_packet {
		std::span<std::byte> data;
	};
	struct splice_in_packet {
		pipe_region to;
	};

	explicit link(int fd) noexcept : fd_(fd) {}

	void set_send_flags(int v) noexcept { cfg_.send_flags = v; }  // 例如 MSG_NOSIGNAL、MSG_DONTWAIT、MSG_ZEROCOPY（fd 先 setsockopt(SO_ZEROCOPY)）
	void set_receive_flags(int v) noexcept { cfg_.receive_flags = v; }  // 例如 MSG_PEEK（只看不取）、MSG_DONTWAIT、MSG_WAITALL、MSG_TRUNC
	void set_splice_flags(unsigned v) noexcept { cfg_.splice_flags = v; }  // 例如 SPLICE_F_MOVE、SPLICE_F_NONBLOCK

	send_packet send_pack(std::span<std::byte const> d) const noexcept { return {d}; }
	sendfile_packet send_pack(file_region r) const noexcept { return {r}; }
	splice_out_packet send_pack(pipe_region p) const noexcept { return {p}; }
	receive_packet receive_pack(std::span<std::byte> d) const noexcept { return {d}; }
	splice_in_packet receive_pack(pipe_region p) const noexcept { return {p}; }

	// send(flags = config.send_flags)。返回发出的字节数。
	// MSG_ZEROCOPY 时返回只表示已交给内核；buf 何时可复用看错误队列（errqueue_link），序号由用户自己数。
	std::size_t communicate(send_packet p) const { return detail::check(::send(fd_, p.data.data(), p.data.size(), cfg_.send_flags), "send"); }

	// sendfile：文件 → socket。返回发出的字节数。
	std::size_t communicate(sendfile_packet p) const {
		::off_t off = p.region.offset;
		return detail::check(::sendfile(fd_, p.region.fd, &off, p.region.count), "sendfile");
	}

	// splice：pipe → socket。返回搬运的字节数。
	std::size_t communicate(splice_out_packet p) const {
		return detail::check(::splice(p.from.fd, nullptr, fd_, nullptr, p.from.count, cfg_.splice_flags), "splice");
	}

	// recv(flags = config.receive_flags)。返回收到的字节数（0 怎么解读由用户决定；MSG_TRUNC 时是真实长度）。
	std::size_t communicate(receive_packet p) const { return detail::check(::recv(fd_, p.data.data(), p.data.size(), cfg_.receive_flags), "recv"); }

	// splice：socket → pipe。返回搬运的字节数。
	std::size_t communicate(splice_in_packet p) const {
		return detail::check(::splice(fd_, nullptr, p.to.fd, nullptr, p.to.count, cfg_.splice_flags), "splice");
	}

private:
	struct config {
		int send_flags = 0;    // 例如 MSG_NOSIGNAL、MSG_DONTWAIT、MSG_ZEROCOPY（fd 先 setsockopt(SO_ZEROCOPY)）
		int receive_flags = 0; // 例如 MSG_PEEK（只看不取）、MSG_DONTWAIT、MSG_WAITALL、MSG_TRUNC
		unsigned splice_flags = 0;  // 例如 SPLICE_F_MOVE、SPLICE_F_NONBLOCK
	};

	int fd_;
	config cfg_;
};

// errqueue_link：同一个 socket 的错误队列（MSG_ZEROCOPY 的“可复用”通知、IP_RECVERR 的 ICMP 错误等）。
// recvmsg(MSG_ERRQUEUE) 永不阻塞：队列空时以 EAGAIN 失败（抛异常）。要等，用户自己 poll(POLLERR)。
// 原样交出 sock_extended_err，解读由用户决定。
class errqueue_link {
public:
	struct receive_packet {
		::sock_extended_err* out;
	};

	explicit errqueue_link(int fd) noexcept : fd_(fd) {}

	receive_packet receive_pack(::sock_extended_err& e) const noexcept { return {&e}; }

	void communicate(receive_packet p) const {
		alignas(::cmsghdr) char ctrl[CMSG_SPACE(sizeof(::sock_extended_err))];
		::msghdr m{};
		m.msg_control = ctrl;
		m.msg_controllen = sizeof ctrl;
		detail::check(::recvmsg(fd_, &m, MSG_ERRQUEUE), "recvmsg(MSG_ERRQUEUE)");
		std::memcpy(p.out, CMSG_DATA(CMSG_FIRSTHDR(&m)), sizeof *p.out);
	}

private:
	int fd_;
};

// writev_link / readv_link：动态 n 段合成 1 个单元。
// send_pack / receive_pack 直接返回 iovec；用户把任意多个放进一段连续内存（上限 IOV_MAX 由用户保证）。
// 返回总字节数（只有一个总数，每段填了多少由用户自己算）。iovec 不带方向，所以发、收是两个 link。
class writev_link {
public:
	explicit writev_link(int fd) noexcept : fd_(fd) {}

	::iovec send_pack(std::span<std::byte const> d) const noexcept { return detail::iov(d); }

	std::size_t communicate_range(auto&& iovs) const {
		return detail::check(::writev(fd_, std::ranges::data(iovs), static_cast<int>(std::ranges::size(iovs))), "writev");
	}

private:
	int fd_;
};

class readv_link {
public:
	explicit readv_link(int fd) noexcept : fd_(fd) {}

	::iovec receive_pack(std::span<std::byte> d) const noexcept { return detail::iov(d); }

	std::size_t communicate_range(auto&& iovs) const {
		return detail::check(::readv(fd_, std::ranges::data(iovs), static_cast<int>(std::ranges::size(iovs))), "readv");
	}

private:
	int fd_;
};

// sendmsg_link：sendmsg，1 段或动态 n 段 iovec 合成 1 个单元，可带 UDP GSO。
// config.gso_size != 0 时附带 UDP_SEGMENT cmsg：内核把这一块按 gso_size 切成多个 UDP 包发出。
class sendmsg_link {
public:
	struct send_packet {
		std::span<std::byte const> data;
	};

	explicit sendmsg_link(int fd) noexcept : fd_(fd) {}

	void set_flags(int v) noexcept { cfg_.flags = v; }
	void set_gso_size(std::uint16_t v) noexcept { cfg_.gso_size = v; }

	send_packet send_pack(std::span<std::byte const> d) const noexcept { return {d}; }

	// 1 段。返回发出的字节数。
	std::size_t communicate(send_packet p) const {
		::iovec v = detail::iov(p.data);
		return send(&v, 1);
	}

	// 动态 n 段：一段连续的 iovec。返回发出的字节数。
	std::size_t communicate_range(auto&& iovs) const { return send(std::ranges::data(iovs), std::ranges::size(iovs)); }

private:
	struct config {
		int flags = 0;
		std::uint16_t gso_size = 0;
	};

	std::size_t send(::iovec* v, std::size_t n) const {
		alignas(::cmsghdr) char ctrl[CMSG_SPACE(sizeof(std::uint16_t))];
		::msghdr m{};
		m.msg_iov = v;
		m.msg_iovlen = n;
		if (cfg_.gso_size != 0) {
			m.msg_control = ctrl;
			m.msg_controllen = sizeof ctrl;
			::cmsghdr* c = CMSG_FIRSTHDR(&m);
			c->cmsg_level = IPPROTO_UDP;
			c->cmsg_type = UDP_SEGMENT;
			c->cmsg_len = CMSG_LEN(sizeof(std::uint16_t));
			std::memcpy(CMSG_DATA(c), &cfg_.gso_size, sizeof(std::uint16_t));
		}
		return detail::check(::sendmsg(fd_, &m, cfg_.flags), "sendmsg");
	}

	int fd_;
	config cfg_;
};

// recvmsg_link：recvmsg，1 段或动态 n 段 iovec，交出 msg_flags 和 UDP GRO 的 gso_size。
// 要收 GRO 合并的包，fd 先 setsockopt(UDP_GRO)（用户做）；合并时 gro_size 是每个原始包的大小，用户按它切回。
class recvmsg_link {
public:
	struct result {
		std::size_t size;        // 收到的字节数
		int flags;               // msg_flags：MSG_TRUNC、MSG_CTRUNC 等
		std::uint16_t gro_size;  // 0 表示没有合并
	};

	struct receive_packet {
		std::span<std::byte> data;
	};

	explicit recvmsg_link(int fd) noexcept : fd_(fd) {}

	void set_flags(int v) noexcept { cfg_.flags = v; }

	receive_packet receive_pack(std::span<std::byte> d) const noexcept { return {d}; }

	result communicate(receive_packet p) const {
		::iovec v = detail::iov(p.data);
		return receive(&v, 1);
	}

	result communicate_range(auto&& iovs) const { return receive(std::ranges::data(iovs), std::ranges::size(iovs)); }

private:
	struct config {
		int flags = 0;
	};

	result receive(::iovec* v, std::size_t n) const {
		alignas(::cmsghdr) char ctrl[CMSG_SPACE(sizeof(int))];
		::msghdr m{};
		m.msg_iov = v;
		m.msg_iovlen = n;
		m.msg_control = ctrl;
		m.msg_controllen = sizeof ctrl;
		std::size_t r = detail::check(::recvmsg(fd_, &m, cfg_.flags), "recvmsg");
		std::uint16_t gro = 0;
		for (::cmsghdr* c = CMSG_FIRSTHDR(&m); c; c = CMSG_NXTHDR(&m, c))
			if (c->cmsg_level == IPPROTO_UDP && c->cmsg_type == UDP_GRO) {
				int g;
				std::memcpy(&g, CMSG_DATA(c), sizeof g);
				gro = static_cast<std::uint16_t>(g);
			}
		return {r, m.msg_flags, gro};
	}

	int fd_;
	config cfg_;
};

// sendmmsg_link / recvmmsg_link：动态 n 个数据报一次系统调用。
// send_pack / receive_pack 直接返回 mmsghdr；每个数据报的内存用一段 iovec 描述（iovec 数组由用户持有）。
// communicate_range 返回成功的数据报个数 k（前 k 个有效）；msg_len 是输出：内核写回每个数据报实际的字节数，
// 接收时 msg_hdr.msg_flags 也由内核写回（MSG_TRUNC 等）。
class sendmmsg_link {
public:
	explicit sendmmsg_link(int fd) noexcept : fd_(fd) {}

	void set_flags(int v) noexcept { cfg_.flags = v; }

	::mmsghdr send_pack(std::span<::iovec> iov) const noexcept {
		::mmsghdr h{};
		h.msg_hdr.msg_iov = iov.data();
		h.msg_hdr.msg_iovlen = iov.size();
		return h;
	}

	std::size_t communicate_range(auto&& msgs) const {
		return detail::check(::sendmmsg(fd_, std::ranges::data(msgs), static_cast<unsigned>(std::ranges::size(msgs)), cfg_.flags), "sendmmsg");
	}

private:
	struct config {
		int flags = 0;
	};

	int fd_;
	config cfg_;
};

class recvmmsg_link {
public:
	explicit recvmmsg_link(int fd) noexcept : fd_(fd) {}

	void set_flags(int v) noexcept { cfg_.flags = v; }  // 例如 MSG_WAITFORONE：收到第一个后不再阻塞
	void set_timeout(::timespec* v) noexcept { cfg_.timeout = v; }  // recvmmsg 的 timeout 参数

	::mmsghdr receive_pack(std::span<::iovec> iov) const noexcept {
		::mmsghdr h{};
		h.msg_hdr.msg_iov = iov.data();
		h.msg_hdr.msg_iovlen = iov.size();
		return h;
	}

	std::size_t communicate_range(auto&& msgs) const {
		return detail::check(::recvmmsg(fd_, std::ranges::data(msgs), static_cast<unsigned>(std::ranges::size(msgs)), cfg_.flags, cfg_.timeout), "recvmmsg");
	}

private:
	struct config {
		int flags = 0;                  // 例如 MSG_WAITFORONE：收到第一个后不再阻塞
		::timespec* timeout = nullptr;  // recvmmsg 的 timeout 参数
	};

	int fd_;
	config cfg_;
};

}  // namespace ibus::sock
