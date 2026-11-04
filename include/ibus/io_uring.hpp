#pragma once

#include <ibus/cpo.hpp>

#include <cassert>
#include <cerrno>
#include <cstddef>
#include <cstdint>
#include <ranges>
#include <span>
#include <system_error>
#include <utility>

#include <liburing.h>

// 先编排、再一次提交的一类：io_uring。
// link 不持有资源：io_uring 由用户 io_uring_queue_init / queue_exit，fd 由用户打开、关闭。
// io_uring_submit / io_uring_wait_cqe 都是同步调用，communicate / communicate_range 直接返回结果。
namespace ibus::uring {

// sq_link：往一个 io_uring 的 SQ 里提交对某个 fd 的操作。
// send_pack / receive_pack 直接返回填好的 io_uring_sqe（opcode、fd、buf、len、user_data、flags）；
// 用户把任意多个放进一段连续内存，communicate_range 逐个拷进 SQ，一次 io_uring_submit。
// 结果不在这里：每项各自产生一个 CQE，由 cq_link 取。
class sq_link {
public:
	sq_link(::io_uring& ring, int fd) noexcept : ring_(&ring), fd_(fd) {}

	// 打包时写进每个 SQE 的属性（打包时读取当前值）：
	//   IOSQE_IO_LINK 和下一项串成链按顺序执行；IOSQE_IO_HARDLINK 前项失败也继续；IOSQE_IO_DRAIN 等前面全部完成；
	//   IOSQE_ASYNC 直接交给 io-wq；IOSQE_CQE_SKIP_SUCCESS 成功时不产生 CQE；IOSQE_FIXED_FILE fd 是固定文件下标。
	void set_sqe_flags(std::uint8_t v) noexcept { cfg_.sqe_flags = v; }
	void set_send_flags(int v) noexcept { cfg_.send_flags = v; }        // send 的 msg flags，例如 MSG_NOSIGNAL
	void set_receive_flags(int v) noexcept { cfg_.receive_flags = v; }  // recv 的 msg flags，例如 MSG_WAITALL

	// user_data 原样带到对应的 CQE 里，怎么用由用户决定。
	::io_uring_sqe send_pack(std::span<std::byte const> d, std::uint64_t user_data) const noexcept {
		::io_uring_sqe s = blank();
		::io_uring_prep_send(&s, fd_, d.data(), d.size(), cfg_.send_flags);
		return finish(s, user_data);
	}
	::io_uring_sqe receive_pack(std::span<std::byte> d, std::uint64_t user_data) const noexcept {
		::io_uring_sqe s = blank();
		::io_uring_prep_recv(&s, fd_, d.data(), d.size(), cfg_.receive_flags);
		return finish(s, user_data);
	}

	// 1 项：拷进 SQ，io_uring_submit。返回提交的项数。
	std::size_t communicate(::io_uring_sqe const& s) const {
		push(s);
		return submit();
	}

	// 动态 n 项：一段连续的 io_uring_sqe，逐个拷进 SQ，一次 io_uring_submit。返回提交的项数。
	// SQ 容量（建 ring 时定）至少 n 是用户的逻辑保证。
	std::size_t communicate_range(auto&& sqes) const {
		for (::io_uring_sqe const& s : sqes) push(s);
		return submit();
	}

private:
	struct config {
		std::uint8_t sqe_flags = 0;
		int send_flags = 0;
		int receive_flags = 0;
	};

	// SQE 存在用户的内存里，不经过 io_uring_get_sqe，所以要和它一样先把其余字段清零（prep_* 只填几个字段）。
	static ::io_uring_sqe blank() noexcept {
		::io_uring_sqe s;
		::io_uring_initialize_sqe(&s);
		return s;
	}
	::io_uring_sqe finish(::io_uring_sqe s, std::uint64_t user_data) const noexcept {
		s.flags = cfg_.sqe_flags;
		::io_uring_sqe_set_data64(&s, user_data);
		return s;
	}

	void push(::io_uring_sqe const& s) const noexcept {
		::io_uring_sqe* q = ::io_uring_get_sqe(ring_);
		assert(q && "uring: SQ full");
		*q = s;
	}

	std::size_t submit() const {
		int r = ::io_uring_submit(ring_);
		if (r < 0) throw std::system_error(-r, std::generic_category(), "io_uring_submit");
		return static_cast<std::size_t>(r);
	}

	::io_uring* ring_;
	int fd_;
	config cfg_;
};

// 一个已收到的 CQE；析构时 io_uring_cqe_seen 归还。只可移动。
class cqe {
public:
	cqe(::io_uring* ring, ::io_uring_cqe* c) noexcept : ring_(ring), cqe_(c) {}
	cqe(cqe&& o) noexcept : ring_(o.ring_), cqe_(std::exchange(o.cqe_, nullptr)) {}
	cqe& operator=(cqe&& o) noexcept {
		if (this != &o) {
			reset();
			ring_ = o.ring_;
			cqe_ = std::exchange(o.cqe_, nullptr);
		}
		return *this;
	}
	cqe(cqe const&) = delete;
	cqe& operator=(cqe const&) = delete;
	~cqe() { reset(); }

	::io_uring_cqe const& operator*() const noexcept { return *cqe_; }
	::io_uring_cqe const* operator->() const noexcept { return cqe_; }

private:
	void reset() noexcept {
		if (cqe_) ::io_uring_cqe_seen(ring_, cqe_);
	}

	::io_uring* ring_;
	::io_uring_cqe* cqe_;
};

// cq_link：从一个 io_uring 的 CQ 取完成项。CQ 里的 CQE 来自这个 ring 上所有 sq_link 提交的项，靠 user_data 区分。
class cq_link {
public:
	struct wait_packet {};  // 等一个 CQE：io_uring_wait_cqe

	explicit cq_link(::io_uring& ring) noexcept : ring_(&ring) {}

	wait_packet receive_pack() const noexcept { return {}; }

	// 等一个 CQE，返回它（res 不解读：< 0 是 -errno，由用户判断）。
	cqe communicate(wait_packet) const {
		::io_uring_cqe* c;
		int r;
		while ((r = ::io_uring_wait_cqe(ring_, &c)) == -EINTR) {}
		if (r < 0) throw std::system_error(-r, std::generic_category(), "io_uring_wait_cqe");
		return {ring_, c};
	}

private:
	::io_uring* ring_;
};

}  // namespace ibus::uring
