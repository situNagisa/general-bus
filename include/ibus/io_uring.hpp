#pragma once

#include <ibus/cpo.hpp>

#include <atomic>
#include <cassert>
#include <cerrno>
#include <cstddef>
#include <cstdint>
#include <span>
#include <system_error>
#include <utility>

#include <linux/io_uring.h>
#include <sys/syscall.h>
#include <unistd.h>

// 先编排、再一次提交的一类：io_uring，直接用原生接口（io_uring_setup / io_uring_enter + mmap 的共享环），不用 liburing。
// link 不持有资源：io_uring_setup、mmap、munmap、close 都由用户做；link 只拿环的视图（uring::ring）。
// 要求：SQ 带索引数组（不设 IORING_SETUP_NO_SQARRAY），SQE 64 字节、CQE 16 字节（不设 SQE128 / CQE32）。
// io_uring_enter 是同步调用，communicate / communicate_range 直接返回结果。
namespace ibus::uring {

namespace detail {

inline std::uint32_t load_relaxed(std::uint32_t* p) noexcept { return std::atomic_ref(*p).load(std::memory_order_relaxed); }
inline std::uint32_t load_acquire(std::uint32_t* p) noexcept { return std::atomic_ref(*p).load(std::memory_order_acquire); }
inline void store_release(std::uint32_t* p, std::uint32_t v) noexcept { std::atomic_ref(*p).store(v, std::memory_order_release); }

inline int enter(int ring_fd, unsigned to_submit, unsigned min_complete, unsigned flags) noexcept {
	return static_cast<int>(::syscall(__NR_io_uring_enter, ring_fd, to_submit, min_complete, flags, nullptr, 0));
}

}  // namespace detail

// 一个已建好、已 mmap 的 io_uring 的视图（不持有）。
struct ring {
	int fd;
	unsigned setup_flags;
	// SQ：内核从 head 取，用户往 tail 放；放的是 SQE 数组的下标（array），SQE 本身在 sqes 里
	std::uint32_t* sq_head;
	std::uint32_t* sq_tail;
	std::uint32_t* sq_flags;
	std::uint32_t* sq_array;
	std::uint32_t sq_mask;
	std::uint32_t sq_entries;
	::io_uring_sqe* sqes;
	// CQ：内核往 tail 放，用户从 head 取
	std::uint32_t* cq_head;
	std::uint32_t* cq_tail;
	std::uint32_t cq_mask;
	::io_uring_cqe* cqes;

	// sq_map：IORING_OFF_SQ_RING 处的映射；cq_map：IORING_OFF_CQ_RING 处的映射（IORING_FEAT_SINGLE_MMAP 时就是 sq_map）；
	// sqes_map：IORING_OFF_SQES 处的映射。偏移都来自 io_uring_setup 填回的 params。
	static ring at(int fd, ::io_uring_params const& p, void* sq_map, void* cq_map, void* sqes_map) noexcept {
		auto* sq = static_cast<std::byte*>(sq_map);
		auto* cq = static_cast<std::byte*>(cq_map);
		auto u32 = [](std::byte* b, std::uint32_t off) { return reinterpret_cast<std::uint32_t*>(b + off); };
		return {
			.fd = fd,
			.setup_flags = p.flags,
			.sq_head = u32(sq, p.sq_off.head),
			.sq_tail = u32(sq, p.sq_off.tail),
			.sq_flags = u32(sq, p.sq_off.flags),
			.sq_array = u32(sq, p.sq_off.array),
			.sq_mask = *u32(sq, p.sq_off.ring_mask),
			.sq_entries = *u32(sq, p.sq_off.ring_entries),
			.sqes = static_cast<::io_uring_sqe*>(sqes_map),
			.cq_head = u32(cq, p.cq_off.head),
			.cq_tail = u32(cq, p.cq_off.tail),
			.cq_mask = *u32(cq, p.cq_off.ring_mask),
			.cqes = reinterpret_cast<::io_uring_cqe*>(cq + p.cq_off.cqes),
		};
	}
};

// sq_link：往一个 io_uring 的 SQ 提交对某个 fd 的操作。
// send_pack / receive_pack 直接在共享内存里的 SQE 上原地填写（没有拷贝），返回 SQE 下标；SQE 槽位由用户分配
// （下标 < sq_entries，同一时刻不能被两次编排占用）。communicate_range 把这些下标写进 SQ 索引环、推进 tail，
// 一次 io_uring_enter 提交——只提交传进来的这些下标，其它已填写但没交上来的 SQE 内核看不到。
// 非 SQPOLL 时，communicate_range 返回后，前“返回值”个下标的 SQE 已被内核读走，槽位可以复用。
// 结果不在这里：每项各自产生一个 CQE，由 cq_link 取。
class sq_link {
public:
	sq_link(ring const& r, int fd) noexcept : r_(&r), fd_(fd) {}

	// 编排时写进每个 SQE 的属性（编排时读取当前值）：
	//   IOSQE_IO_LINK 和下一项串成链按顺序执行；IOSQE_IO_HARDLINK 前项失败也继续；IOSQE_IO_DRAIN 等前面全部完成；
	//   IOSQE_ASYNC 直接交给 io-wq；IOSQE_CQE_SKIP_SUCCESS 成功时不产生 CQE；IOSQE_FIXED_FILE fd 是固定文件下标。
	void set_sqe_flags(std::uint8_t v) noexcept { cfg_.sqe_flags = v; }
	void set_send_flags(std::uint32_t v) noexcept { cfg_.send_flags = v; }        // send 的 msg flags，例如 MSG_NOSIGNAL
	void set_receive_flags(std::uint32_t v) noexcept { cfg_.receive_flags = v; }  // recv 的 msg flags，例如 MSG_WAITALL

	// user_data 原样带到对应的 CQE 里，怎么用由用户决定。
	std::uint32_t send_pack(std::uint32_t slot, std::span<std::byte const> d, std::uint64_t user_data) const noexcept {
		fill(slot, IORING_OP_SEND, d.data(), d.size(), cfg_.send_flags, user_data);
		return slot;
	}
	std::uint32_t receive_pack(std::uint32_t slot, std::span<std::byte> d, std::uint64_t user_data) const noexcept {
		fill(slot, IORING_OP_RECV, d.data(), d.size(), cfg_.receive_flags, user_data);
		return slot;
	}

	// 1 项。返回内核接受的项数。
	std::size_t communicate(std::uint32_t slot) const { return publish(std::span(&slot, 1)); }

	// 动态 n 项：一段 SQE 下标，按顺序进 SQ，一次 io_uring_enter。返回内核接受的项数。
	// SQ 索引环的空位够 n 个是用户的逻辑保证。
	std::size_t communicate_range(auto&& slots) const { return publish(slots); }

private:
	struct config {
		std::uint8_t sqe_flags = 0;
		std::uint32_t send_flags = 0;
		std::uint32_t receive_flags = 0;
	};

	// 和 liburing 的 io_uring_initialize_sqe + io_uring_prep_rw 写的字段完全一样。
	void fill(std::uint32_t slot, std::uint8_t op, void const* p, std::size_t n, std::uint32_t msg_flags, std::uint64_t user_data) const noexcept {
		assert(slot < r_->sq_entries && "uring: slot out of range");
		::io_uring_sqe& s = r_->sqes[slot];
		s.opcode = op;
		s.flags = cfg_.sqe_flags;
		s.ioprio = 0;
		s.fd = fd_;
		s.off = 0;
		s.addr = reinterpret_cast<std::uintptr_t>(p);
		s.len = static_cast<std::uint32_t>(n);
		s.msg_flags = msg_flags;
		s.user_data = user_data;
		s.buf_index = 0;
		s.personality = 0;
		s.file_index = 0;
		s.addr3 = 0;
		s.__pad2[0] = 0;
	}

	std::size_t publish(auto&& slots) const {
		ring const& r = *r_;
		std::uint32_t tail = detail::load_relaxed(r.sq_tail);  // 只有用户态写 tail
		std::uint32_t n = 0;
		for (std::uint32_t i : slots) {
			r.sq_array[tail++ & r.sq_mask] = i;
			++n;
		}
		assert(tail - detail::load_acquire(r.sq_head) <= r.sq_entries && "uring: SQ full");
		detail::store_release(r.sq_tail, tail);  // 发布：此后内核才看得到这些 SQE

		if (r.setup_flags & IORING_SETUP_SQPOLL) {
			// 内核线程在轮询；它睡着了才需要叫醒。tail 的写和 flags 的读之间要全屏障。
			std::atomic_thread_fence(std::memory_order_seq_cst);
			if ((detail::load_relaxed(r.sq_flags) & IORING_SQ_NEED_WAKEUP) && detail::enter(r.fd, 0, 0, IORING_ENTER_SQ_WAKEUP) < 0)
				throw std::system_error(errno, std::generic_category(), "io_uring_enter(SQ_WAKEUP)");
			return n;
		}
		int k = detail::enter(r.fd, n, 0, 0);
		if (k < 0) throw std::system_error(errno, std::generic_category(), "io_uring_enter");
		return static_cast<std::size_t>(k);
	}

	ring const* r_;
	int fd_;
	config cfg_;
};

// 一个已收到的 CQE（直接指向共享内存里的那一项）；析构时推进 CQ head 归还。只可移动。
// CQ 按顺序归还：同一时刻只持有一个，或按取到的顺序析构。
class cqe {
public:
	cqe(std::uint32_t* head, std::uint32_t pos, ::io_uring_cqe const* c) noexcept : head_(head), pos_(pos), cqe_(c) {}
	cqe(cqe&& o) noexcept : head_(std::exchange(o.head_, nullptr)), pos_(o.pos_), cqe_(o.cqe_) {}
	cqe& operator=(cqe&&) = delete;
	cqe(cqe const&) = delete;
	cqe& operator=(cqe const&) = delete;
	~cqe() {
		if (head_) detail::store_release(head_, pos_ + 1);
	}

	::io_uring_cqe const& operator*() const noexcept { return *cqe_; }
	::io_uring_cqe const* operator->() const noexcept { return cqe_; }

private:
	std::uint32_t* head_;
	std::uint32_t pos_;
	::io_uring_cqe const* cqe_;
};

// cq_link：从一个 io_uring 的 CQ 取完成项。CQ 里的 CQE 来自这个 ring 上所有 sq_link 提交的项，靠 user_data 区分。
class cq_link {
public:
	struct wait_packet {};

	explicit cq_link(ring const& r) noexcept : r_(&r) {}

	wait_packet receive_pack() const noexcept { return {}; }

	// 等一个 CQE：CQ 里有就直接返回；没有就 io_uring_enter(GETEVENTS) 等至少 1 个。res 不解读：< 0 是 -errno。
	// 上一个 cqe 没析构前再调用，会拿到同一个 CQE。
	cqe communicate(wait_packet) const {
		ring const& r = *r_;
		for (;;) {
			std::uint32_t head = detail::load_relaxed(r.cq_head);
			if (head != detail::load_acquire(r.cq_tail)) return {r.cq_head, head, &r.cqes[head & r.cq_mask]};
			if (detail::enter(r.fd, 0, 1, IORING_ENTER_GETEVENTS) < 0 && errno != EINTR)
				throw std::system_error(errno, std::generic_category(), "io_uring_enter(GETEVENTS)");
		}
	}

private:
	ring const* r_;
};

}  // namespace ibus::uring
