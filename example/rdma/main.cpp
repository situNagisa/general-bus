#include <ibus/rdma.hpp>

// 先编排、再一次提交：RDMA verbs。
//   send_link：send_pack(SEND / RDMA WRITE) / receive_pack(RDMA READ) 返回 WR，communicate_range 串成链表一次 ibv_post_send
//   recv_link：receive_pack 返回 recv WR，communicate_range 一次 ibv_post_recv
//   cq_link：communicate(receive_pack(wc 数组)) 取完成项，靠 wr_id 区分
// link 不持有资源：设备、PD、MR、CQ、QP（含 INIT→RTR→RTS 自连）都由这里建、销毁。
// 本机没有 RDMA 设备时打印提示退出。完整建链流程见 example/native/rdma/post_send_recv.cpp。

#include <array>
#include <cstdio>
#include <exception>
#include <span>
#include <vector>

int main() {
	using namespace ibus;
	int n = 0;
	::ibv_device** devs = ::ibv_get_device_list(&n);
	if (!devs || n == 0) {
		std::printf("no RDMA device (try rdma_rxe / soft-RoCE)\n");
		if (devs) ::ibv_free_device_list(devs);
		return 1;
	}
	try {
		::ibv_context* ctx = ::ibv_open_device(devs[0]);
		::ibv_pd* pd = ::ibv_alloc_pd(ctx);
		::ibv_cq* cqh = ::ibv_create_cq(ctx, 16, nullptr, nullptr, 0);
		::ibv_qp_init_attr qa{};
		qa.send_cq = qa.recv_cq = cqh;
		qa.qp_type = IBV_QPT_RC;
		qa.cap = {.max_send_wr = 8, .max_recv_wr = 8, .max_send_sge = 1, .max_recv_sge = 1, .max_inline_data = 0};
		::ibv_qp* qp = ::ibv_create_qp(pd, &qa);
		// （此处省略 QP 状态迁移 INIT→RTR→RTS，见 native 示例；未迁移时 post 会失败，作为出错路径演示）

		static std::array<std::byte, 64> buf{};
		::ibv_mr* mr = ::ibv_reg_mr(pd, buf.data(), buf.size(), IBV_ACCESS_LOCAL_WRITE);

		rdma::recv_link rl(qp);
		rdma::send_link sl(qp);
		rdma::cq_link cq(cqh);

		std::vector<rdma::recv_link::recv_wr> rw{receive_pack(rl, rdma::region{std::span(buf).first(32), mr->lkey}, 1)};
		communicate_range(rl, rw);
		std::vector<rdma::send_link::send_wr> sw{send_pack(sl, rdma::const_region{std::span(buf).last(32), mr->lkey}, 2)};
		communicate_range(sl, sw);

		std::array<::ibv_wc, 4> wc;
		std::printf("polled %zu\n", communicate(cq, receive_pack(cq, std::span(wc))));
		::ibv_dereg_mr(mr);
		::ibv_destroy_qp(qp);
		::ibv_destroy_cq(cqh);
		::ibv_dealloc_pd(pd);
		::ibv_close_device(ctx);
	} catch (std::exception const& e) {
		std::printf("error: %s\n", e.what());
	}
	::ibv_free_device_list(devs);
}
