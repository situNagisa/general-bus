#pragma once

#include <ibus/cpo.hpp>
#include <ibus/sender.hpp>

#include <cerrno>
#include <cstddef>
#include <cstdint>
#include <span>
#include <system_error>
#include <utility>

#include <infiniband/verbs.h>

// 先编排、再一次提交的一类：RDMA verbs（libibverbs）。
// port 不持有资源：设备、PD、MR、CQ、QP 及其状态迁移都由用户建、管、销毁，port 只记下句柄。
// ibv_post_send / ibv_post_recv / ibv_poll_cq 都是同步调用（只入队 / 只取出，不等待）：communicate 返回 sender，start 时执行。
namespace ibus::rdma {

// 一段已注册内存：地址 + 长度 + MR 的 lkey。
struct region {
	std::span<std::byte> data;
	std::uint32_t lkey;
};
struct const_region {
	std::span<std::byte const> data;
	std::uint32_t lkey;
};

// 对端的一段注册内存（RDMA WRITE / READ 的目标）：远端地址 + rkey，由用户带外交换得到。
struct remote {
	std::uint64_t addr;
	std::uint32_t rkey;
};

namespace detail {
inline ::ibv_sge sge(void const* p, std::size_t n, std::uint32_t lkey) noexcept {
	return {reinterpret_cast<std::uint64_t>(p), static_cast<std::uint32_t>(n), lkey};
}
}  // namespace detail

// 提交（send_pack 的参数）。
struct submit_t {};
inline constexpr submit_t submit{};

// WR 放在用户的内存里（地址固定，直到提交返回）：ibv_*_wr 加它自己的一个 sge。
struct send_wr {
	::ibv_send_wr wr;
	::ibv_sge sge;
};
struct recv_wr {
	::ibv_recv_wr wr;
	::ibv_sge sge;
};

// concept：发送队列。send_pack(wr, 本地内存, [remote,] wr_id, send_flags) 编排 SEND / WRITE / READ，完成值为空；
// send_pack(rdma::submit) 一次提交，完成值为空。
// send_queue 的参考 receiver：正好接得住 send_queue 里各组合发出的完成；用户的 receiver 照这个写，作为 send_queue 的 R 参数检查。
struct send_queue_receiver {
	using receiver_concept = stdexec::receiver_t;
	void set_value() && noexcept {}
	void set_error(std::error_code) && noexcept {}
};
template <class P, class R = send_queue_receiver>
concept send_queue = requires(P& p, send_wr& w, const_region out, region in, remote r, std::uint64_t wr_id, unsigned send_flags) {
	requires stdexec::sender_to<decltype(communicate(p, send_pack(p, w, out, wr_id, send_flags))), R>;
	requires stdexec::sender_to<decltype(communicate(p, send_pack(p, w, out, r, wr_id, send_flags))), R>;
	requires stdexec::sender_to<decltype(communicate(p, send_pack(p, w, in, r, wr_id, send_flags))), R>;
	requires stdexec::sender_to<decltype(communicate(p, send_pack(p, submit))), R>;
};

// concept：接收队列。send_pack(wr, 本地内存, wr_id) 挂一个接收缓冲，send_pack(rdma::submit) 一次提交，完成值都为空。
// receive_queue 的参考 receiver：正好接得住 receive_queue 里各组合发出的完成；用户的 receiver 照这个写，作为 receive_queue 的 R 参数检查。
struct receive_queue_receiver {
	using receiver_concept = stdexec::receiver_t;
	void set_value() && noexcept {}
	void set_error(std::error_code) && noexcept {}
};
template <class P, class R = receive_queue_receiver>
concept receive_queue = requires(P& p, recv_wr& w, region in, std::uint64_t wr_id) {
	requires stdexec::sender_to<decltype(communicate(p, send_pack(p, w, in, wr_id))), R>;
	requires stdexec::sender_to<decltype(communicate(p, send_pack(p, submit))), R>;
};

// concept：CQ。receive_pack(ibv_wc 数组) 一次最多取满，完成值是取到的个数。
// completion_queue 的参考 receiver：正好接得住 completion_queue 里各组合发出的完成；用户的 receiver 照这个写，作为 completion_queue 的 R 参数检查。
struct completion_queue_receiver {
	using receiver_concept = stdexec::receiver_t;
	void set_value(std::size_t) && noexcept {}
	void set_error(std::error_code) && noexcept {}
};
template <class P, class R = completion_queue_receiver>
concept completion_queue = requires(P& p, std::span<::ibv_wc> out) {
	requires stdexec::sender_to<decltype(communicate(p, receive_pack(p, out))), R>;
};

// send_port：一个 QP 的发送队列。当作协议：
//   communicate(sq, send_pack(sq, wr, ...))     在用户的 send_wr 上原地填写，接到待提交链表尾部（无系统调用）
//   communicate(sq, send_pack(sq, rdma::submit)) 一次 ibv_post_send 提交整条链表
// 每个 WR 的完成（设了 IBV_SEND_SIGNALED 的）是 CQ 里的一个 ibv_wc，由 cq_port 取，靠 wr_id 区分。
// send_flags：IBV_SEND_SIGNALED 产生完成项；IBV_SEND_FENCE 等前面的 READ / 原子操作完成；
//   IBV_SEND_INLINE 数据直接拷进 WQE；IBV_SEND_SOLICITED 对端产生 solicited 事件。
struct send_port {
	::ibv_qp* qp;
	::ibv_send_wr* head;   // 待提交链表
	::ibv_send_wr** tail;  // 指向链表最后一个 next（空链表时指向 head）

	static send_port at(::ibv_qp* qp) noexcept { return {qp, nullptr, nullptr}; }

	struct wr_packet {
		send_wr* w;
	};
	struct submit_packet {};

	// SEND：对端用 post_recv 挂的缓冲接收。
	wr_packet send_pack(send_wr& w, const_region r, std::uint64_t wr_id, unsigned send_flags = IBV_SEND_SIGNALED) const noexcept {
		return _fill(w, IBV_WR_SEND, r.data.data(), r.data.size(), r.lkey, wr_id, {}, send_flags);
	}
	// RDMA WRITE：直接写进对端内存，对端 CPU 不参与、不产生接收完成。
	wr_packet send_pack(send_wr& w, const_region r, remote dst, std::uint64_t wr_id, unsigned send_flags = IBV_SEND_SIGNALED) const noexcept {
		return _fill(w, IBV_WR_RDMA_WRITE, r.data.data(), r.data.size(), r.lkey, wr_id, dst, send_flags);
	}
	// RDMA READ：从对端内存读进本地。对 QP 来说仍是往发送队列投递一个请求，所以也是 send_pack。
	wr_packet send_pack(send_wr& w, region r, remote src, std::uint64_t wr_id, unsigned send_flags = IBV_SEND_SIGNALED) const noexcept {
		return _fill(w, IBV_WR_RDMA_READ, r.data.data(), r.data.size(), r.lkey, wr_id, src, send_flags);
	}
	submit_packet send_pack(submit_t) const noexcept { return {}; }

	// 接到链表尾部。
	auto communicate(wr_packet p) noexcept {
		return ibus::detail::nothrow<void>([self = this, p] noexcept {
			if (!self->head) self->tail = &self->head;
			*self->tail = &p.w->wr;
			self->tail = &p.w->wr.next;
		});
	}

	// 一次 ibv_post_send 提交整条链表。
	// 完成值为空；失败 set_error(ibv_post_send 返回的错误码)。
	auto communicate(submit_packet) noexcept {
		return ibus::detail::complete<stdexec::set_value_t(), stdexec::set_error_t(std::error_code)>([self = this](auto&& r) noexcept {
			::ibv_send_wr* h = std::exchange(self->head, nullptr);
			::ibv_send_wr* bad = nullptr;
			if (int e = ::ibv_post_send(self->qp, h, &bad)) return stdexec::set_error(std::move(r), std::error_code(e, std::generic_category()));
			stdexec::set_value(std::move(r));
		});
	}

private:
	static wr_packet _fill(send_wr& w, ::ibv_wr_opcode op, void const* p, std::size_t n, std::uint32_t lkey, std::uint64_t wr_id, remote r, unsigned send_flags) noexcept {
		w.sge = detail::sge(p, n, lkey);
		w.wr = {};
		w.wr.wr_id = wr_id;
		w.wr.sg_list = &w.sge;
		w.wr.num_sge = 1;
		w.wr.opcode = op;
		w.wr.send_flags = send_flags;
		w.wr.wr.rdma.remote_addr = r.addr;
		w.wr.wr.rdma.rkey = r.rkey;
		return {&w};
	}
};

// recv_port：一个 QP 的接收队列（对端 SEND 到达时写进挂上的缓冲）。和 send_port 一样当作协议：
//   communicate(rq, send_pack(rq, wr, region, wr_id))  在用户的 recv_wr 上原地填写，接到待提交链表尾部
//   communicate(rq, send_pack(rq, rdma::submit))       一次 ibv_post_recv
// 挂缓冲是往 QP “发”一个请求，所以用 send_pack；数据到达的结果从 cq_port 收。
struct recv_port {
	::ibv_qp* qp;
	::ibv_recv_wr* head;
	::ibv_recv_wr** tail;

	static recv_port at(::ibv_qp* qp) noexcept { return {qp, nullptr, nullptr}; }

	struct wr_packet {
		recv_wr* w;
	};
	struct submit_packet {};

	wr_packet send_pack(recv_wr& w, region r, std::uint64_t wr_id) const noexcept {
		w.sge = detail::sge(r.data.data(), r.data.size(), r.lkey);
		w.wr = {};
		w.wr.wr_id = wr_id;
		w.wr.sg_list = &w.sge;
		w.wr.num_sge = 1;
		return {&w};
	}
	submit_packet send_pack(submit_t) const noexcept { return {}; }

	auto communicate(wr_packet p) noexcept {
		return ibus::detail::nothrow<void>([self = this, p] noexcept {
			if (!self->head) self->tail = &self->head;
			*self->tail = &p.w->wr;
			self->tail = &p.w->wr.next;
		});
	}

	// 完成值为空；失败 set_error(ibv_post_recv 返回的错误码)。
	auto communicate(submit_packet) noexcept {
		return ibus::detail::complete<stdexec::set_value_t(), stdexec::set_error_t(std::error_code)>([self = this](auto&& r) noexcept {
			::ibv_recv_wr* h = std::exchange(self->head, nullptr);
			::ibv_recv_wr* bad = nullptr;
			if (int e = ::ibv_post_recv(self->qp, h, &bad)) return stdexec::set_error(std::move(r), std::error_code(e, std::generic_category()));
			stdexec::set_value(std::move(r));
		});
	}
};

// cq_port：从一个 CQ 取完成项。ibv_poll_cq 不等待：没有就返回 0 个。
// receive_pack 交出用户的 ibv_wc 数组；communicate 一次最多取满它，返回取到的个数。
struct cq_port {
	::ibv_cq* cq;

	struct poll_packet {
		std::span<::ibv_wc> out;
	};

	poll_packet receive_pack(std::span<::ibv_wc> out) const noexcept { return {out}; }

	// 完成值是取到的个数；ibv_poll_cq 失败（返回负数，不设 errno）时 set_error(EIO)。
	auto communicate(poll_packet p) const noexcept {
		return ibus::detail::complete<stdexec::set_value_t(std::size_t), stdexec::set_error_t(std::error_code)>([cq = cq, p](auto&& r) noexcept {
			int n = ::ibv_poll_cq(cq, static_cast<int>(p.out.size()), p.out.data());
			if (n < 0) return stdexec::set_error(std::move(r), std::error_code(EIO, std::generic_category()));
			stdexec::set_value(std::move(r), static_cast<std::size_t>(n));
		});
	}
};

static_assert(send_queue<send_port> && receive_queue<recv_port> && completion_queue<cq_port>);

}  // namespace ibus::rdma
