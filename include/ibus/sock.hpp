#pragma once

#include <ibus/cpo.hpp>
#include <ibus/sender.hpp>

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

// 所有 port 都不持有 fd：socket 由用户创建、连接、配置、关闭，port 只记下 fd。
// 都是同步系统调用（阻不阻塞看 fd 自己）：communicate / communicate_range 返回 sender，start 时做一次系统调用，
// 完成值走 set_value，失败走 set_error(std::error_code)（errno）。
// 每次调用的 flags 等是 send_pack / receive_pack（或 communicate_range）的参数。
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

inline ::iovec iov(std::span<std::byte const> d) noexcept { return {const_cast<std::byte*>(d.data()), d.size()}; }

}  // namespace detail

// port：send / recv / sendfile / splice。
struct port {
	int fd;

	struct send_packet {
		std::span<std::byte const> data;
		int flags;
	};
	struct sendfile_packet {
		file_region region;
	};
	struct splice_out_packet {
		pipe_region from;
		unsigned flags;
	};
	struct receive_packet {
		std::span<std::byte> data;
		int flags;
	};
	struct splice_in_packet {
		pipe_region to;
		unsigned flags;
	};

	// flags：MSG_NOSIGNAL、MSG_DONTWAIT、MSG_ZEROCOPY（fd 先 setsockopt(SO_ZEROCOPY)）等。
	// MSG_ZEROCOPY 时返回只表示已交给内核；buf 何时可复用看错误队列（errqueue_port），序号由用户自己数。
	send_packet send_pack(std::span<std::byte const> d, int flags = 0) const noexcept { return {d, flags}; }
	sendfile_packet send_pack(file_region r) const noexcept { return {r}; }
	// flags：SPLICE_F_MOVE、SPLICE_F_NONBLOCK 等。
	splice_out_packet send_pack(pipe_region p, unsigned flags = 0) const noexcept { return {p, flags}; }
	// flags：MSG_PEEK（只看不取）、MSG_DONTWAIT、MSG_WAITALL、MSG_TRUNC 等。
	receive_packet receive_pack(std::span<std::byte> d, int flags = 0) const noexcept { return {d, flags}; }
	splice_in_packet receive_pack(pipe_region p, unsigned flags = 0) const noexcept { return {p, flags}; }

	// send。返回发出的字节数。
	auto communicate(send_packet p) const noexcept {
		return ibus::detail::syscall<std::size_t>([fd = fd, p] noexcept { return ::send(fd, p.data.data(), p.data.size(), p.flags); });
	}

	// sendfile：文件 → socket。返回发出的字节数。
	auto communicate(sendfile_packet p) const noexcept {
		return ibus::detail::syscall<std::size_t>([fd = fd, p] noexcept {
			::off_t off = p.region.offset;
			return ::sendfile(fd, p.region.fd, &off, p.region.count);
		});
	}

	// splice：pipe → socket。返回搬运的字节数。
	auto communicate(splice_out_packet p) const noexcept {
		return ibus::detail::syscall<std::size_t>([fd = fd, p] noexcept { return ::splice(p.from.fd, nullptr, fd, nullptr, p.from.count, p.flags); });
	}

	// recv。返回收到的字节数（0 怎么解读由用户决定；MSG_TRUNC 时是真实长度）。
	auto communicate(receive_packet p) const noexcept {
		return ibus::detail::syscall<std::size_t>([fd = fd, p] noexcept { return ::recv(fd, p.data.data(), p.data.size(), p.flags); });
	}

	// splice：socket → pipe。返回搬运的字节数。
	auto communicate(splice_in_packet p) const noexcept {
		return ibus::detail::syscall<std::size_t>([fd = fd, p] noexcept { return ::splice(fd, nullptr, p.to.fd, nullptr, p.to.count, p.flags); });
	}
};

// errqueue_port：一个 socket 的错误队列（MSG_ZEROCOPY 的“可复用”通知、IP_RECVERR 的 ICMP 错误等）。
// recvmsg(MSG_ERRQUEUE) 永不阻塞：队列空时以 EAGAIN 失败（set_error）。要等，用户自己 poll(POLLERR)。
// 控制缓冲由用户给（至少 CMSG_SPACE(sizeof(sock_extended_err))），返回指向其中 sock_extended_err 的指针，解读由用户决定。
struct errqueue_port {
	int fd;

	struct receive_packet {
		std::span<std::byte> control;
	};

	receive_packet receive_pack(std::span<std::byte> control) const noexcept { return {control}; }

	auto communicate(receive_packet p) const noexcept {
		return ibus::detail::complete<stdexec::set_value_t(::sock_extended_err const*), stdexec::set_error_t(std::error_code)>(
			[fd = fd, p](auto&& r) noexcept {
				::msghdr m{};
				m.msg_control = p.control.data();
				m.msg_controllen = p.control.size();
				if (::recvmsg(fd, &m, MSG_ERRQUEUE) < 0) return stdexec::set_error(std::move(r), ibus::detail::errno_code());
				stdexec::set_value(std::move(r), reinterpret_cast<::sock_extended_err const*>(CMSG_DATA(CMSG_FIRSTHDR(&m))));
			});
	}
};

// writev_port / readv_port：动态 n 段合成 1 个单元。
// send_pack / receive_pack 直接返回 iovec；用户把任意多个放进一段连续内存（上限 IOV_MAX 由用户保证）。
// 返回总字节数（只有一个总数，每段填了多少由用户自己算）。iovec 不带方向，所以发、收是两个 port。
struct writev_port {
	int fd;

	::iovec send_pack(std::span<std::byte const> d) const noexcept { return detail::iov(d); }

	auto communicate_range(auto&& iovs) const noexcept {
		return ibus::detail::syscall<std::size_t>([fd = fd, v = std::ranges::data(iovs), n = static_cast<int>(std::ranges::size(iovs))] noexcept {
			return ::writev(fd, v, n);
		});
	}
};

struct readv_port {
	int fd;

	::iovec receive_pack(std::span<std::byte> d) const noexcept { return detail::iov(d); }

	auto communicate_range(auto&& iovs) const noexcept {
		return ibus::detail::syscall<std::size_t>([fd = fd, v = std::ranges::data(iovs), n = static_cast<int>(std::ranges::size(iovs))] noexcept {
			return ::readv(fd, v, n);
		});
	}
};

namespace detail {

// sendmsg：gso_size != 0 时附带 UDP_SEGMENT cmsg，内核把这一块按 gso_size 切成多个 UDP 包发出。
inline ::ssize_t sendmsg(int fd, ::iovec* v, std::size_t n, int flags, std::uint16_t gso_size) noexcept {
	alignas(::cmsghdr) char ctrl[CMSG_SPACE(sizeof(std::uint16_t))];
	::msghdr m{};
	m.msg_iov = v;
	m.msg_iovlen = n;
	if (gso_size != 0) {
		m.msg_control = ctrl;
		m.msg_controllen = sizeof ctrl;
		::cmsghdr* c = CMSG_FIRSTHDR(&m);
		c->cmsg_level = IPPROTO_UDP;
		c->cmsg_type = UDP_SEGMENT;
		c->cmsg_len = CMSG_LEN(sizeof(std::uint16_t));
		std::memcpy(CMSG_DATA(c), &gso_size, sizeof(std::uint16_t));
	}
	return ::sendmsg(fd, &m, flags);
}

}  // namespace detail

// sendmsg_port：sendmsg，1 段或动态 n 段 iovec 合成 1 个单元，可带 UDP GSO。
struct sendmsg_port {
	int fd;

	struct send_packet {
		std::span<std::byte const> data;
		int flags;
		std::uint16_t gso_size;
	};

	send_packet send_pack(std::span<std::byte const> d, int flags = 0, std::uint16_t gso_size = 0) const noexcept { return {d, flags, gso_size}; }

	// 1 段。返回发出的字节数。
	auto communicate(send_packet p) const noexcept {
		return ibus::detail::syscall<std::size_t>([fd = fd, p] noexcept {
			::iovec v = detail::iov(p.data);
			return detail::sendmsg(fd, &v, 1, p.flags, p.gso_size);
		});
	}

	// 动态 n 段：一段连续的 iovec。返回发出的字节数。
	auto communicate_range(auto&& iovs, int flags = 0, std::uint16_t gso_size = 0) const noexcept {
		return ibus::detail::syscall<std::size_t>([fd = fd, v = std::ranges::data(iovs), n = std::ranges::size(iovs), flags, gso_size] noexcept {
			return detail::sendmsg(fd, v, n, flags, gso_size);
		});
	}
};

// recvmsg_port：recvmsg，1 段或动态 n 段 iovec，可带控制缓冲（cmsg：时间戳、IP_PKTINFO、UDP_GRO 的 gso_size 等）。
// cmsg 的解析由用户做（CMSG_FIRSTHDR / CMSG_NXTHDR 遍历用户给的控制缓冲）。
struct recvmsg_port {
	int fd;

	struct result {
		std::size_t size;           // 收到的字节数
		int flags;                  // msg_flags：MSG_TRUNC、MSG_CTRUNC 等
		std::size_t control_size;   // 控制缓冲里实际写了多少
	};

	struct receive_packet {
		std::span<std::byte> data;
		std::span<std::byte> control;
		int flags;
	};

	receive_packet receive_pack(std::span<std::byte> d, std::span<std::byte> control = {}, int flags = 0) const noexcept { return {d, control, flags}; }

private:
	static auto _make(auto f) noexcept { return ibus::detail::complete<stdexec::set_value_t(result), stdexec::set_error_t(std::error_code)>(std::move(f)); }

	static void _receive(auto&& r, int fd, ::iovec* v, std::size_t n, std::span<std::byte> control, int flags) noexcept {
		::msghdr m{};
		m.msg_iov = v;
		m.msg_iovlen = n;
		m.msg_control = control.data();
		m.msg_controllen = control.size();
		::ssize_t k = ::recvmsg(fd, &m, flags);
		if (k < 0) return stdexec::set_error(std::move(r), ibus::detail::errno_code());
		stdexec::set_value(std::move(r), result{static_cast<std::size_t>(k), m.msg_flags, m.msg_controllen});
	}

public:
	auto communicate(receive_packet p) const noexcept {
		return _make([fd = fd, p](auto&& r) noexcept {
			::iovec v = detail::iov(p.data);
			_receive(std::move(r), fd, &v, 1, p.control, p.flags);
		});
	}

	auto communicate_range(auto&& iovs, std::span<std::byte> control = {}, int flags = 0) const noexcept {
		return _make([fd = fd, v = std::ranges::data(iovs), n = std::ranges::size(iovs), control, flags](auto&& r) noexcept {
			_receive(std::move(r), fd, v, n, control, flags);
		});
	}
};

// sendmmsg_port / recvmmsg_port：动态 n 个数据报一次系统调用。
// send_pack / receive_pack 直接返回 mmsghdr；每个数据报的内存用一段 iovec 描述（iovec 数组由用户持有）。
// communicate_range 返回成功的数据报个数 k（前 k 个有效）；msg_len 是输出：内核写回每个数据报实际的字节数，
// 接收时 msg_hdr.msg_flags 也由内核写回（MSG_TRUNC 等）。
struct sendmmsg_port {
	int fd;

	::mmsghdr send_pack(std::span<::iovec> iov) const noexcept {
		::mmsghdr h{};
		h.msg_hdr.msg_iov = iov.data();
		h.msg_hdr.msg_iovlen = iov.size();
		return h;
	}

	auto communicate_range(auto&& msgs, int flags = 0) const noexcept {
		return ibus::detail::syscall<std::size_t>([fd = fd, m = std::ranges::data(msgs), n = static_cast<unsigned>(std::ranges::size(msgs)), flags] noexcept {
			return ::sendmmsg(fd, m, n, flags);
		});
	}
};

struct recvmmsg_port {
	int fd;

	::mmsghdr receive_pack(std::span<::iovec> iov) const noexcept {
		::mmsghdr h{};
		h.msg_hdr.msg_iov = iov.data();
		h.msg_hdr.msg_iovlen = iov.size();
		return h;
	}

	// flags：例如 MSG_WAITFORONE（收到第一个后不再阻塞）；timeout：recvmmsg 的超时参数。
	auto communicate_range(auto&& msgs, int flags = 0, ::timespec* timeout = nullptr) const noexcept {
		return ibus::detail::syscall<std::size_t>(
			[fd = fd, m = std::ranges::data(msgs), n = static_cast<unsigned>(std::ranges::size(msgs)), flags, timeout] noexcept {
				return ::recvmmsg(fd, m, n, flags, timeout);
			});
	}
};

}  // namespace ibus::sock
