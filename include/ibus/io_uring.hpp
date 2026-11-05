#pragma once

#include <ibus/cpo.hpp>

#include <atomic>
#include <cassert>
#include <cerrno>
#include <cstddef>
#include <cstdint>
#include <span>
#include <system_error>

#include <linux/io_uring.h>
#include <sys/syscall.h>
#include <unistd.h>

// io_uring，原生接口（io_uring_setup / io_uring_enter + mmap 的共享环），不用 liburing。
// 把 io_uring 当作一个协议：往 SQ 写 SQE、提交，都是往内核“发”；从 CQ 取 CQE 是“收”。
//   communicate(sq, send_pack(sq, uring::send / uring::receive, ...))   取 SQ 的下一个槽位，原地填 SQE、写下标（不发布）
//   communicate(sq, send_pack(sq, uring::submit[, wait_nr]))  发布 tail，一次 io_uring_enter（wait_nr > 0 时顺带等至少这么多个 CQE）
//   communicate(cq, receive_pack(cq))       取下一个 CQE：返回指向共享内存的指针，没有就是 nullptr（不等待、不拷贝、无系统调用）
//   communicate(cq, send_pack(cq, uring::release))  归还已取出的 CQE：发布 head（无系统调用）
// 每个对外函数最多一次系统调用；出错（包括 EINTR）直接抛异常，重试由用户决定。
// port 不持有资源：io_uring_setup、mmap、munmap、close 都由用户做。
// 要求：SQ 带索引数组（不设 IORING_SETUP_NO_SQARRAY），SQE 64 字节、CQE 16 字节（不设 SQE128 / CQE32）。
namespace ibus::uring {

// send_pack 的第一个参数：要往内核发的是什么请求。
struct send_t {};     // IORING_OP_SEND：对 fd 发数据
struct receive_t {};  // IORING_OP_RECV：从 fd 收数据（对内核来说也是“发”一个请求）
struct submit_t {};   // 提交已写进 SQ 的项（可顺带等待完成）
struct release_t {};  // 归还已取出的 CQE（发布 CQ head）
inline constexpr send_t send{};
inline constexpr receive_t receive{};
inline constexpr submit_t submit{};
inline constexpr release_t release{};

namespace detail {

inline std::uint32_t load_acquire(std::uint32_t* p) noexcept { return std::atomic_ref(*p).load(std::memory_order_acquire); }
inline std::uint32_t load_relaxed(std::uint32_t* p) noexcept { return std::atomic_ref(*p).load(std::memory_order_relaxed); }
inline void store_release(std::uint32_t* p, std::uint32_t v) noexcept { std::atomic_ref(*p).store(v, std::memory_order_release); }

inline int enter(int ring_fd, unsigned to_submit, unsigned min_complete, unsigned flags) noexcept {
	return static_cast<int>(::syscall(__NR_io_uring_enter, ring_fd, to_submit, min_complete, flags, nullptr, 0));
}

}  // namespace detail

// sq_port：一个 io_uring 的 SQ。
// SQE 槽位就按 tail 顺序取（槽位 = tail & mask），SQE 原地填写、下标原地写进索引环，提交时只发布 tail。
// pending 是写进去但还没发布的 tail（只有用户态写 tail）。
struct sq_port {
	int ring_fd;
	unsigned setup_flags;
	std::uint32_t* head;
	std::uint32_t* tail;
	std::uint32_t* flags;
	std::uint32_t* array;
	std::uint32_t mask;
	std::uint32_t entries;
	::io_uring_sqe* sqes;
	std::uint32_t pending;
	std::uint32_t published;

	// sq_map：IORING_OFF_SQ_RING 处的映射；sqes_map：IORING_OFF_SQES 处的映射。偏移来自 io_uring_setup 填回的 params。
	static sq_port at(int ring_fd, ::io_uring_params const& p, void* sq_map, void* sqes_map) noexcept {
		auto* b = static_cast<std::byte*>(sq_map);
		auto u32 = [b](std::uint32_t off) { return reinterpret_cast<std::uint32_t*>(b + off); };
		std::uint32_t t = *u32(p.sq_off.tail);
		return {ring_fd, p.flags, u32(p.sq_off.head), u32(p.sq_off.tail), u32(p.sq_off.flags), u32(p.sq_off.array),
			*u32(p.sq_off.ring_mask), *u32(p.sq_off.ring_entries), static_cast<::io_uring_sqe*>(sqes_map), t, t};
	}

	// 一项操作的 SQE 内容（除了 opcode 还要填的字段）。
	struct sqe_packet {
		std::uint8_t opcode;
		std::uint8_t sqe_flags;
		int fd;
		void const* addr;
		std::uint32_t len;
		std::uint32_t op_flags;  // msg_flags 等，按 opcode 解释
		std::uint64_t user_data;
	};
	struct submit_packet {
		std::uint32_t wait_nr;
	};

	// send_pack(uring::send, fd, data, user_data[, sqe_flags, msg_flags])：IORING_OP_SEND。
	// sqe_flags：IOSQE_IO_LINK 和下一项串成链按顺序执行；IOSQE_IO_HARDLINK 前项失败也继续；IOSQE_IO_DRAIN 等前面全部完成；
	//   IOSQE_ASYNC 直接交给 io-wq；IOSQE_CQE_SKIP_SUCCESS 成功时不产生 CQE；IOSQE_FIXED_FILE fd 是固定文件下标。
	// msg_flags：MSG_NOSIGNAL 等。user_data 原样带到对应的 CQE 里。
	sqe_packet send_pack(send_t, int fd, std::span<std::byte const> d, std::uint64_t user_data, std::uint8_t sqe_flags = 0, std::uint32_t msg_flags = 0) const noexcept {
		return {IORING_OP_SEND, sqe_flags, fd, d.data(), static_cast<std::uint32_t>(d.size()), msg_flags, user_data};
	}
	// send_pack(uring::receive, fd, buf, user_data[, sqe_flags, msg_flags])：IORING_OP_RECV。对内核来说也是“发”一个请求，所以也是 send_pack。
	sqe_packet send_pack(receive_t, int fd, std::span<std::byte> d, std::uint64_t user_data, std::uint8_t sqe_flags = 0, std::uint32_t msg_flags = 0) const noexcept {
		return {IORING_OP_RECV, sqe_flags, fd, d.data(), static_cast<std::uint32_t>(d.size()), msg_flags, user_data};
	}
	submit_packet send_pack(submit_t, std::uint32_t wait_nr = 0) const noexcept { return {wait_nr}; }

	// 取下一个槽位，原地填 SQE、写下标（不发布）。SQ 满是用户的逻辑错误（用户自己先提交腾位置）。
	void communicate(sqe_packet p) noexcept {
		assert(pending - detail::load_acquire(head) < entries && "uring: SQ full");
		std::uint32_t slot = pending & mask;
		::io_uring_sqe& s = sqes[slot];
		// 和 liburing 的 io_uring_initialize_sqe + io_uring_prep_rw 写的字段完全一样。
		s.opcode = p.opcode;
		s.flags = p.sqe_flags;
		s.ioprio = 0;
		s.fd = p.fd;
		s.off = 0;
		s.addr = reinterpret_cast<std::uintptr_t>(p.addr);
		s.len = p.len;
		s.msg_flags = p.op_flags;
		s.user_data = p.user_data;
		s.buf_index = 0;
		s.personality = 0;
		s.file_index = 0;
		s.addr3 = 0;
		s.__pad2[0] = 0;
		array[slot] = slot;
		++pending;
	}

	// 发布 tail，最多一次 io_uring_enter。返回内核接受的项数（SQPOLL 时是发布的项数）。
	// 非 SQPOLL：enter(n, wait_nr, wait_nr ? GETEVENTS : 0)；SQPOLL：只有内核线程睡着、或要等待时才 enter。
	std::size_t communicate(submit_packet p) {
		std::uint32_t n = pending - published;
		detail::store_release(tail, pending);
		published = pending;
		unsigned f = p.wait_nr ? IORING_ENTER_GETEVENTS : 0u;
		if (setup_flags & IORING_SETUP_SQPOLL) {
			// tail 的写和 flags 的读之间要全屏障。
			std::atomic_thread_fence(std::memory_order_seq_cst);
			if (detail::load_relaxed(flags) & IORING_SQ_NEED_WAKEUP) f |= IORING_ENTER_SQ_WAKEUP;
			if (f && detail::enter(ring_fd, 0, p.wait_nr, f) < 0) throw std::system_error(errno, std::generic_category(), "io_uring_enter");
			return n;
		}
		int k = detail::enter(ring_fd, n, p.wait_nr, f);
		if (k < 0) throw std::system_error(errno, std::generic_category(), "io_uring_enter");
		return static_cast<std::size_t>(k);
	}
};

// cq_port：一个 io_uring 的 CQ。CQ 里的 CQE 来自这个 ring 上提交的所有项，靠 user_data 区分。
// pending 是已取出但还没归还的 head（只有用户态写 head）。
struct cq_port {
	std::uint32_t* head;
	std::uint32_t* tail;
	std::uint32_t mask;
	::io_uring_cqe* cqes;
	std::uint32_t pending;

	// cq_map：IORING_OFF_CQ_RING 处的映射（IORING_FEAT_SINGLE_MMAP 时就是 SQ 那段映射）。
	static cq_port at(::io_uring_params const& p, void* cq_map) noexcept {
		auto* b = static_cast<std::byte*>(cq_map);
		auto u32 = [b](std::uint32_t off) { return reinterpret_cast<std::uint32_t*>(b + off); };
		std::uint32_t* h = u32(p.cq_off.head);
		return {h, u32(p.cq_off.tail), *u32(p.cq_off.ring_mask), reinterpret_cast<::io_uring_cqe*>(b + p.cq_off.cqes), *h};
	}

	struct cqe_packet {};
	struct release_packet {};

	cqe_packet receive_pack() const noexcept { return {}; }
	release_packet send_pack(release_t) const noexcept { return {}; }

	// 取下一个 CQE：指向共享内存里的那一项，归还（release）之前一直有效；没有就是 nullptr。
	// res 不解读：< 0 是 -errno，由用户判断。
	::io_uring_cqe const* communicate(cqe_packet) noexcept {
		if (pending == detail::load_acquire(tail)) return nullptr;
		return &cqes[pending++ & mask];
	}

	// 归还已取出的 CQE：发布 head，内核才能复用这些槽位。
	void communicate(release_packet) noexcept { detail::store_release(head, pending); }
};

}  // namespace ibus::uring
