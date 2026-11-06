#include <ibus/rdma.hpp>

// 先编排、再一次提交：RDMA verbs。
//   send_port：send_pack(wr, SEND / RDMA WRITE / RDMA READ) 在用户的 WR 上原地填写并接到链表，send_pack(submit) 一次 ibv_post_send
//   recv_port：send_pack(wr, region) 同上，send_pack(submit) 一次 ibv_post_recv
//   cq_port：communicate(receive_pack(wc 数组)) 取完成项，靠 wr_id 区分
// port 不持有资源：设备、PD、MR、CQ、QP（含 INIT→RTR→RTS 自连）都由这里建、销毁。
// 本机没有 RDMA 设备时打印提示退出。完整建链流程见 example/native/rdma/post_send_recv.cpp。

#include <array>
#include <cstdio>
#include <tuple>
#include <exception>
#include <span>

// sync_wait 取 sender 唯一的完成值（set_error(error_code) 时 sync_wait 抛 system_error）。
static auto value(auto&& s) { return std::get<0>(stdexec::sync_wait(std::forward<decltype(s)>(s)).value()); }

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

		auto rl = rdma::recv_port::at(qp);
		auto sl = rdma::send_port::at(qp);
		rdma::cq_port cq{cqh};

		// WR 放在用户的内存里，原地填写、接成链表，submit 一次 post
		rdma::recv_wr rw;
		stdexec::sync_wait(communicate(rl, send_pack(rl, rw, rdma::region{std::span(buf).first(32), mr->lkey}, 1)));
		stdexec::sync_wait(communicate(rl, send_pack(rl, rdma::submit)));
		std::array<rdma::send_wr, 2> sw;
		stdexec::sync_wait(communicate(sl, send_pack(sl, sw[0], rdma::const_region{std::span(buf).last(32), mr->lkey}, 2)));
		stdexec::sync_wait(communicate(sl, send_pack(sl, sw[1], rdma::const_region{std::span(buf).last(32), mr->lkey}, 3)));
		stdexec::sync_wait(communicate(sl, send_pack(sl, rdma::submit)));

		std::array<::ibv_wc, 4> wc;
		std::printf("polled %zu\n", value(communicate(cq, receive_pack(cq, std::span(wc)))));
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
