#pragma once

#include <ibus/cpo.hpp>

#include <stdexec/execution.hpp>

#include <cassert>
#include <cerrno>
#include <concepts>
#include <cstddef>
#include <cstdint>
#include <ranges>
#include <system_error>
#include <tuple>
#include <type_traits>
#include <utility>

#include <liburing.h>
#include <unistd.h>

namespace ibus::uring {

// communicate<kind::submit> 接受的 submission：一串已填好的 io_uring_sqe，逐个拷进 SQ。
template <class S>
concept submission = std::ranges::input_range<S> && std::ranges::sized_range<S>
	&& std::convertible_to<std::ranges::range_reference_t<S>, ::io_uring_sqe const&>;

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

// link：对端已确定的 socket；持有 fd，使用（不拥有）一个 io_uring。
class link {
public:
	link(::io_uring& ring, int fd) noexcept : ring_(ring), fd_(fd) {}
	link(link const&) = delete;
	link& operator=(link const&) = delete;
	~link() { ::close(fd_); }

	int fd() const noexcept { return fd_; }

	// 一次 io_uring_submit；完成只表示已提交。左值转发时只存引用，右值时存值。
	template <class K, class S>
		requires std::same_as<K, kind::submit> && submission<std::remove_cvref_t<S>>
	auto communicate(S&& s) {
		return stdexec::just()
			| stdexec::then([ring = &ring_, t = std::tuple<S>(std::forward<S>(s))]() mutable {
				for (::io_uring_sqe const& e : std::get<0>(t)) {
					::io_uring_sqe* sqe = ::io_uring_get_sqe(ring);
					assert(sqe && "uring: SQ full");
					*sqe = e;
				}
				int r = ::io_uring_submit(ring);
				if (r < 0) throw std::system_error(-r, std::generic_category(), "io_uring_submit");
			});
	}

	// 等一个 CQE；完成值是它本身（不解读 res）。
	template <class K>
		requires std::same_as<K, kind::receive>
	auto communicate() {
		return stdexec::just() | stdexec::then([ring = &ring_]() -> cqe {
			::io_uring_cqe* c;
			int r;
			while ((r = ::io_uring_wait_cqe(ring, &c)) == -EINTR) {}
			if (r < 0) throw std::system_error(-r, std::generic_category(), "io_uring_wait_cqe");
			return {ring, c};
		});
	}

private:
	::io_uring& ring_;
	int fd_;
};

}  // namespace ibus::uring
