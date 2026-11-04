#pragma once

#include <ibus/cpo.hpp>

#include <cerrno>
#include <cstddef>
#include <cstdint>
#include <ranges>
#include <span>
#include <system_error>

#include <infiniband/verbs.h>

// 先编排、再一次提交的一类：RDMA verbs（libibverbs）。
// link 不持有资源：设备、PD、MR、CQ、QP 及其状态迁移都由用户建、管、销毁，link 只接收传入。
// ibv_post_send / ibv_post_recv / ibv_poll_cq 都是同步调用（只入队 / 只取出，不等待），直接返回结果。
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

// send_link：往一个 QP 的发送队列投递 WR。
// send_pack 返回 send_wr：一个 ibv_send_wr 加它自己的一个 sge（单段）；用户把任意多个放进一段连续内存，
// communicate_range 把它们串成链表（next 指针），一次 ibv_post_send。
// 每个 WR 的完成（设了 IBV_SEND_SIGNALED 的）是 CQ 里的一个 ibv_wc，由 cq_link 取，靠 wr_id 区分。
class send_link {
public:
	// ibv_send_wr 里 sg_list 指向 sge，二者放在一起；拷贝 / 移动后由 communicate_range 重新指向。
	struct send_wr {
		::ibv_send_wr wr;
		::ibv_sge sge;
	};

	explicit send_link(::ibv_qp* qp) noexcept : qp_(qp) {}

	// 打包时写进每个 WR 的 send_flags（打包时读取当前值）：
	//   IBV_SEND_SIGNALED 产生完成项；IBV_SEND_FENCE 等前面的 READ / 原子操作完成；IBV_SEND_INLINE 数据直接拷进 WQE；
	//   IBV_SEND_SOLICITED 对端产生 solicited 事件。
	void set_send_flags(unsigned v) noexcept { cfg_.send_flags = v; }

	// SEND：对端用 post_recv 挂的缓冲接收。
	send_wr send_pack(const_region r, std::uint64_t wr_id) const noexcept { return make(IBV_WR_SEND, r.data.data(), r.data.size(), r.lkey, wr_id, {}); }
	// RDMA WRITE：直接写进对端内存，对端 CPU 不参与、不产生接收完成。
	send_wr send_pack(const_region r, remote dst, std::uint64_t wr_id) const noexcept { return make(IBV_WR_RDMA_WRITE, r.data.data(), r.data.size(), r.lkey, wr_id, dst); }
	// RDMA READ：从对端内存读进本地。
	send_wr receive_pack(region r, remote src, std::uint64_t wr_id) const noexcept { return make(IBV_WR_RDMA_READ, r.data.data(), r.data.size(), r.lkey, wr_id, src); }

	// 1 个 WR。
	void communicate(send_wr w) const {
		w.wr.sg_list = &w.sge;
		w.wr.next = nullptr;
		post(&w.wr);
	}

	// 动态 n 个 WR：串成链表，一次 ibv_post_send。出错时异常信息里带失败起点的 wr_id。
	void communicate_range(auto&& wrs) const {
		::ibv_send_wr* head = nullptr;
		::ibv_send_wr** tail = &head;
		for (send_wr& w : wrs) {
			w.wr.sg_list = &w.sge;
			w.wr.next = nullptr;
			*tail = &w.wr;
			tail = &w.wr.next;
		}
		post(head);
	}

private:
	struct config {
		unsigned send_flags = IBV_SEND_SIGNALED;
	};

	send_wr make(::ibv_wr_opcode op, void const* p, std::size_t n, std::uint32_t lkey, std::uint64_t wr_id, remote r) const noexcept {
		send_wr w{};
		w.sge = detail::sge(p, n, lkey);
		w.wr.wr_id = wr_id;
		w.wr.num_sge = 1;
		w.wr.opcode = op;
		w.wr.send_flags = cfg_.send_flags;
		w.wr.wr.rdma.remote_addr = r.addr;
		w.wr.wr.rdma.rkey = r.rkey;
		return w;
	}

	void post(::ibv_send_wr* head) const {
		::ibv_send_wr* bad = nullptr;
		if (int e = ::ibv_post_send(qp_, head, &bad))
			throw std::system_error(e, std::generic_category(), "ibv_post_send");
	}

	::ibv_qp* qp_;
	config cfg_;
};

// recv_link：往一个 QP 的接收队列挂缓冲（对端 SEND 到达时写进去）。
// receive_pack 返回 recv_wr；用户把任意多个放进一段连续内存，communicate_range 串成链表，一次 ibv_post_recv。
class recv_link {
public:
	struct recv_wr {
		::ibv_recv_wr wr;
		::ibv_sge sge;
	};

	explicit recv_link(::ibv_qp* qp) noexcept : qp_(qp) {}

	recv_wr receive_pack(region r, std::uint64_t wr_id) const noexcept {
		recv_wr w{};
		w.sge = detail::sge(r.data.data(), r.data.size(), r.lkey);
		w.wr.wr_id = wr_id;
		w.wr.num_sge = 1;
		return w;
	}

	void communicate(recv_wr w) const {
		w.wr.sg_list = &w.sge;
		w.wr.next = nullptr;
		post(&w.wr);
	}

	void communicate_range(auto&& wrs) const {
		::ibv_recv_wr* head = nullptr;
		::ibv_recv_wr** tail = &head;
		for (recv_wr& w : wrs) {
			w.wr.sg_list = &w.sge;
			w.wr.next = nullptr;
			*tail = &w.wr;
			tail = &w.wr.next;
		}
		post(head);
	}

private:
	void post(::ibv_recv_wr* head) const {
		::ibv_recv_wr* bad = nullptr;
		if (int e = ::ibv_post_recv(qp_, head, &bad))
			throw std::system_error(e, std::generic_category(), "ibv_post_recv");
	}

	::ibv_qp* qp_;
};

// cq_link：从一个 CQ 取完成项。ibv_poll_cq 不等待：没有就返回 0 个。
// receive_pack 交出用户的 ibv_wc 数组；communicate 一次最多取满它，返回取到的个数。
class cq_link {
public:
	struct poll_packet {
		std::span<::ibv_wc> out;
	};

	explicit cq_link(::ibv_cq* cq) noexcept : cq_(cq) {}

	poll_packet receive_pack(std::span<::ibv_wc> out) const noexcept { return {out}; }

	std::size_t communicate(poll_packet p) const {
		int n = ::ibv_poll_cq(cq_, static_cast<int>(p.out.size()), p.out.data());
		if (n < 0) throw std::system_error(EIO, std::generic_category(), "ibv_poll_cq");
		return static_cast<std::size_t>(n);
	}

private:
	::ibv_cq* cq_;
};

}  // namespace ibus::rdma
