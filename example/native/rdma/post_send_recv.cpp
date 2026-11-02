/*
* post_send_recv.cpp —— libibverbs 的 ibv_post_send / ibv_post_recv 最小示例
*
* 【这是什么】
* ibv_post_send() / ibv_post_recv() 是 libibverbs 里最底层的“提交”动作：
* 把一组工作请求（WR，Work Request）挂到某个 QP（Queue Pair）的发送队列 SQ
* 或接收队列 RQ 的尾部。之后由网卡（HCA）异步地去执行它们，执行完把完成项
* （CQE）写进用户创建的 CQ（Completion Queue），用户再用 ibv_poll_cq() 收。
*
* 它解决的问题：绕过内核协议栈，把“发一段内存 / 收一段内存”这件事一次性、
* 批量地交给网卡，用户态直接下发，零拷贝（网卡 DMA 读写已注册的 MR）。
*
* 【形态（非常重要，决定了它的性能）】
*   - 一次调用提交“一整条链表”：wr 参数是链头，每个 WR 的 next 指向下一个。
*     链上每一项都是一次独立的 RDMA 操作（这里是 SEND，对应线上一个消息单元）。
*   - 每个 WR 里又可以有多个 SGE（sg_list/num_sge），即“一个线上消息由几段
*     不连续的内存拼成”。本示例每项只用 1 个 SGE。
*   - 提交 N 个 WR 只需要 1 次系统调用（严格说现代 libibverbs 是用户态直接
*     写 doorbell，根本不进内核）。
*   - 完成项不是每个 WR 都有：只有置了 IBV_SEND_SIGNALED 的发送 WR 才产生
*     CQE（不置则“静默完成”，用来省 CQ 空间）；接收 WR 每完成一个必产生一个
*     CQE。CQE 通过 wc.wr_id 和你提交时的 wr_id 对应。
*   - 内存由用户提供：先 ibv_reg_mr() 把一段用户内存注册成 MR（Memory
*     Region），拿到 lkey/rkey，网卡才能 DMA。提交时 SGE.addr 必须是已注册
*     内存的地址。
*   - 出错时：调用本身失败返回 errno，且 bad_wr 指向“第一个不合法的 WR”；
*     它之前的 WR 已经成功入队（这是规范行为，示例里专门演示了这一点）。
*
* 【需要的内核/硬件条件】
*   - 一块真正的 RDMA 网卡（mlx5 等），或内核的 soft-RoCE 模块 rdma_rxe
*     （modprobe rdma_rxe，再用 rdma link add rxe0 type rxe netdev <网卡>）。
*   - WSL2 里通常既没有 RDMA 设备也没有 rxe，本程序会打印提示后退出。
*
* 【本示例做什么】
*   在同一个进程里建一个 RC QP，并把它连到它自己（self loopback）：
*   自己 SEND，自己 RECV。完整走一遍：打开设备 → query port → PD → 注册 MR
*   → 建 CQ → 建 QP → INIT → RTR → RTS → 投递 4 个 RECV + 3 个 SEND（都走
*   链表）→ 轮询 6 个完成项 → 再用一条“第 2 项非法”的 SEND 链演示 bad_wr。
*
* 【编译】
*   g++ -std=c++26 -O2 -Wall -Wextra -o /tmp/native_post_send_recv \
*       post_send_recv.cpp -libverbs
* 【运行】/tmp/native_post_send_recv
* 【预期输出】有可用设备时依次打印设备/端口信息、6 个完成项（3 个
*   IBV_WC_SEND + 3 个 IBV_WC_RECV，按 wr_id 对应）、收到的三段消息，最后
*   打印 bad_wr 演示结果；没有设备时打印一行提示，以失败退出。
*/

#include <infiniband/verbs.h>

#include <array>
#include <cerrno>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <span>
#include <thread>

namespace {

// ---------------------------------------------------------------------------
// 一、RAII 包装
//
// libibverbs 的对象全是裸指针 + 对应的 destroy 函数，手工释放很容易漏。
// 这里用 unique_ptr + 自定义 deleter 把“创建即拥有、离开作用域自动销毁”
// 的语义补上。注意销毁顺序必须和创建顺序相反（QP 依赖 CQ/PD，CQ 依赖
// context……），所以下面 main 里的变量声明顺序是“父先子后”，析构时自动
// 反序，正好正确。
// ---------------------------------------------------------------------------

struct ContextDeleter {
	void operator()(ibv_context *p) const noexcept { ibv_close_device(p); }
};
struct PdDeleter {
	void operator()(ibv_pd *p) const noexcept { ibv_dealloc_pd(p); }
};
struct MrDeleter {
	void operator()(ibv_mr *p) const noexcept { ibv_dereg_mr(p); }
};
struct CqDeleter {
	void operator()(ibv_cq *p) const noexcept { ibv_destroy_cq(p); }
};
struct QpDeleter {
	void operator()(ibv_qp *p) const noexcept { ibv_destroy_qp(p); }
};

using Context = std::unique_ptr<ibv_context, ContextDeleter>;
using Pd = std::unique_ptr<ibv_pd, PdDeleter>;
using Mr = std::unique_ptr<ibv_mr, MrDeleter>;
using Cq = std::unique_ptr<ibv_cq, CqDeleter>;
using Qp = std::unique_ptr<ibv_qp, QpDeleter>;

// 设备列表是 ibv_device* 的数组 + 计数，用 ibv_free_device_list 释放。
// 包成一个小类，暴露 std::span 方便遍历。
class DeviceList {
	public:
	DeviceList() : list_(ibv_get_device_list(&count_)) {}

	~DeviceList()
	{
		if (list_ != nullptr)
			ibv_free_device_list(list_);
	}

	DeviceList(const DeviceList &) = delete;
	DeviceList &operator=(const DeviceList &) = delete;

	// ibv_get_device_list 失败返回 NULL（errno 有值）；成功但没有设备
	// 返回非 NULL 但 count_ == 0，这两种情况要分开处理。
	[[nodiscard]] bool failed() const { return list_ == nullptr; }
	[[nodiscard]] std::span<ibv_device *> devices() const
	{
		return {list_, static_cast<std::size_t>(count_)};
	}

	private:
	ibv_device **list_ = nullptr;
	int count_ = 0;
};

// ---------------------------------------------------------------------------
// 二、小工具
// ---------------------------------------------------------------------------

// CQ 里的 opcode 没有官方的字符串化函数（只有 ibv_wc_status_str），自己写。
const char *wc_opcode_name(ibv_wc_opcode op)
{
	switch (op) {
	case IBV_WC_SEND:
		return "IBV_WC_SEND";
	case IBV_WC_RDMA_WRITE:
		return "IBV_WC_RDMA_WRITE";
	case IBV_WC_RDMA_READ:
		return "IBV_WC_RDMA_READ";
	case IBV_WC_RECV:
		return "IBV_WC_RECV";
	default:
		return "(其它)";
	}
}

// 轮询 CQ 直到拿到 need 个完成项或超时。
// ibv_poll_cq 返回值：>0 表示本次取到的 CQE 个数；0 表示当前 CQ 为空；
// <0 表示出错（不进内核，一般只有 CQ 被销毁之类的情况才会发生）。
// 返回实际取到的个数，出错返回 -1。
int poll_until(ibv_cq *cq, int need, std::span<ibv_wc> out, int timeout_ms)
{
	int got = 0;
	const auto deadline =
		std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);

	while (got < need) {
		const int space = static_cast<int>(out.size()) - got;
		const int n = ibv_poll_cq(cq, space, out.data() + got);
		if (n < 0) {
			std::fprintf(stderr, "ibv_poll_cq 出错（返回值 %d）\n", n);
			return -1;
		}
		got += n;
		if (got >= need)
			break;
		if (std::chrono::steady_clock::now() >= deadline)
			break;
		// 完成项是异步来的，空转只会烧 CPU；让出一下再试。
		std::this_thread::sleep_for(std::chrono::microseconds(50));
	}
	return got;
}

// 打印一个完成项，并返回它的 wr_id，方便调用方做匹配检查。
void print_wc(const ibv_wc &wc)
{
	// status != IBV_WC_SUCCESS 时，byte_len、opcode 等字段的内容都不可信。
	// vendor_err 是厂商自定义错误码，排查硬件问题时才有意义。
	std::printf("  CQE: wr_id=%llu status=%s(%d) opcode=%s byte_len=%u qp_num=%u%s\n",
			static_cast<unsigned long long>(wc.wr_id),
			ibv_wc_status_str(wc.status), static_cast<int>(wc.status),
			wc_opcode_name(wc.opcode), wc.byte_len, wc.qp_num,
			(wc.wc_flags & IBV_WC_WITH_IMM) ? " WITH_IMM" : "");
}

constexpr std::uint8_t kPort = 1;	 // 单口网卡用 1 号端口
constexpr std::size_t kMsgLen = 64;	 // 每个消息 64 字节（远小于 MTU）
constexpr int kSendCount = 3;		 // 主流程投递 3 个 SEND
constexpr int kRecvCount = 4;		 // 多投 1 个给 bad_wr 演示用
constexpr int kCqDepth = 16;

} // namespace

int main()
{
	// -----------------------------------------------------------------
	// 1. 找设备
	// -----------------------------------------------------------------
	DeviceList devs;
	if (devs.failed()) {
		std::fprintf(stderr, "ibv_get_device_list 失败: %s\n", std::strerror(errno));
		return EXIT_FAILURE;
	}

	const auto list = devs.devices();
	if (list.empty()) {
		std::fprintf(stderr,
			"没有找到任何 RDMA 设备（/dev/infiniband 不存在）。\n"
			"WSL2 里一般没有 RDMA 网卡，也无法加载 rdma_rxe；\n"
			"在有硬件/Linux 主机上，可用软 RoCE 验证：\n"
			"  modprobe rdma_rxe\n"
			"  rdma link add rxe0 type rxe netdev <网卡名>\n"
			"程序到此退出。\n");
		return EXIT_FAILURE;
	}

	// 通常直接用第一块。ibv_get_device_name 返回的是内核设备名（如 mlx5_0）。
	ibv_device *dev = list.front();
	std::printf("设备: %s\n", ibv_get_device_name(dev));

	Context ctx{ibv_open_device(dev)};
	if (!ctx) {
		std::fprintf(stderr, "ibv_open_device 失败: %s\n", std::strerror(errno));
		return EXIT_FAILURE;
	}

	// 设备能力：max_qp_wr 是 QP 队列深度上限，max_cqe 是 CQ 深度上限，
	// max_sge 是单个 WR 能挂的 SGE 数上限。这些都要用来“按需申请”，
	// 申请超过上限时 ibv_create_qp 会返回 EINVAL。
	ibv_device_attr dev_attr{};
	if (ibv_query_device(ctx.get(), &dev_attr) != 0) {
		std::fprintf(stderr, "ibv_query_device 失败: %s\n", std::strerror(errno));
		return EXIT_FAILURE;
	}
	std::printf("能力: max_qp_wr=%d max_cqe=%d max_sge=%d\n", dev_attr.max_qp_wr,
			dev_attr.max_cqe, dev_attr.max_sge);

	// -----------------------------------------------------------------
	// 2. 查询端口状态
	//
	// RC QP 的“地址向量”（AV）里要填 LID 或 GID，都来自这里。
	// InfiniBand 用 LID（16 位本地地址）；RoCE 是以太网链路，没有 LID，
	// 必须用 is_global=1 的 GID 路由（即 AV 里带 GRH）。
	// -----------------------------------------------------------------
	ibv_port_attr port{};
	if (ibv_query_port(ctx.get(), kPort, &port) != 0) {
		std::fprintf(stderr, "ibv_query_port 失败: %s\n", std::strerror(errno));
		return EXIT_FAILURE;
	}
	const bool roce = port.link_layer == IBV_LINK_LAYER_ETHERNET;
	std::printf("端口 %u: state=%s lid=0x%x link_layer=%s active_mtu=%u 字节\n",
			kPort, ibv_port_state_str(port.state), port.lid,
			roce ? "Ethernet(RoCE)" : "InfiniBand",
			1u << (static_cast<unsigned>(port.active_mtu) + 7));

	// 端口必须 ACTIVE，否则 RTR 迁移一定会失败（对端不可达）。
	if (port.state != IBV_PORT_ACTIVE) {
		std::fprintf(stderr, "端口未处于 ACTIVE 状态，无法建链，退出。\n");
		return EXIT_FAILURE;
	}

	// -----------------------------------------------------------------
	// 3. PD + MR
	//
	// PD（Protection Domain）是权限隔离的边界：MR、QP 都属于某个 PD，
	// 不同 PD 的 MR 不能互相访问（相当于一把“钥匙串”)。
	//
	// ibv_reg_mr 把用户虚拟地址固定（pin 住）并交给网卡做 DMA 映射，
	// 返回的 MR 里 lkey 给“本地 SGE”用，rkey 给“对端访问我”用。
	// 注册一次的内存可以被任意多个 WR 引用，不要频繁注册/注销。
	// -----------------------------------------------------------------
	Pd pd{ibv_alloc_pd(ctx.get())};
	if (!pd) {
		std::fprintf(stderr, "ibv_alloc_pd 失败: %s\n", std::strerror(errno));
		return EXIT_FAILURE;
	}

	// 一块 8 KiB 的“竞技场”：前 3 KiB 当发送缓冲，后 4 KiB 切成 4 个
	// 1024 字节的接收槽。只用一次 reg_mr 覆盖全部，lkey 共用。
	std::array<char, 8192> arena{};
	constexpr std::size_t kSendArea = 0;
	constexpr std::size_t kRecvArea = 4096;
	constexpr std::size_t kRecvSlot = 1024;

	// 只做 SEND/RECV 的话，注册 IBV_ACCESS_LOCAL_WRITE 就够了：接收到的
	// 数据是网卡“写入”本地内存，所以接收缓冲必须带 LOCAL_WRITE。
	// （RDMA WRITE 才需要 REMOTE_WRITE，RDMA READ 需要 REMOTE_READ。）
	Mr mr{ibv_reg_mr(pd.get(), arena.data(), arena.size(), IBV_ACCESS_LOCAL_WRITE)};
	if (!mr) {
		std::fprintf(stderr, "ibv_reg_mr 失败: %s\n", std::strerror(errno));
		return EXIT_FAILURE;
	}
	std::printf("MR: addr=%p len=%zu lkey=0x%x\n", static_cast<void *>(arena.data()),
			arena.size(), mr->lkey);

	// -----------------------------------------------------------------
	// 4. CQ
	//
	// CQ 是“完成项数组”，由用户态创建，网卡往里面写 CQE。cqe 是容量，
	// 必须 >= 同时在途的 WR 数，否则队列会溢出（OVERFLOW，完成项丢失）。
	// 发送和接收共用同一个 CQ 在简单场景下完全可行，靠 wr_id 区分。
	// -----------------------------------------------------------------
	Cq cq{ibv_create_cq(ctx.get(), kCqDepth, nullptr, nullptr, 0)};
	if (!cq) {
		std::fprintf(stderr, "ibv_create_cq 失败: %s\n", std::strerror(errno));
		return EXIT_FAILURE;
	}

	// -----------------------------------------------------------------
	// 5. 建 QP
	//
	// IBV_QPT_RC = Reliable Connection，一对一、可靠、按序，最常用。
	// cap 里声明自己需要的队列深度：max_send_wr/max_recv_wr 是 SQ/RQ
	// 能挂的 WR 数；max_send_sge/max_recv_sge 是单个 WR 最多挂几个 SGE。
	// send_cq 和 recv_cq 可以是同一个（这里就是）。
	// sq_sig_all=0 表示“只对置了 IBV_SEND_SIGNALED 的 WR 产生完成项”，
	// 这正是批量提交时省 CQE 的关键。
	// -----------------------------------------------------------------
	ibv_qp_init_attr qp_ia{};
	qp_ia.send_cq = cq.get();
	qp_ia.recv_cq = cq.get();
	qp_ia.qp_type = IBV_QPT_RC;
	qp_ia.sq_sig_all = 0;
	qp_ia.cap.max_send_wr = 8;
	qp_ia.cap.max_recv_wr = 8;
	qp_ia.cap.max_send_sge = 1;
	qp_ia.cap.max_recv_sge = 1;
	qp_ia.cap.max_inline_data = 0;	// 不用 inline，数据走 MR

	Qp qp{ibv_create_qp(pd.get(), &qp_ia)};
	if (!qp) {
		std::fprintf(stderr, "ibv_create_qp 失败: %s\n", std::strerror(errno));
		return EXIT_FAILURE;
	}
	std::printf("QP: qpn=%u（本示例把它连到自己，构成 loopback）\n", qp->qp_num);

	// 下面三段是 RC QP 固定的状态机：RESET → INIT → RTR → RTS。
	// 每次 ibv_modify_qp 都要用 mask 指出“这次改了哪些字段”，没改的字段
	// 即使填了也会被忽略。

	// (a) INIT：设置端口、PKEY 索引、本端权限。
	//     qp_access_flags 是本端允许对端做什么；SEND/RECV 用不到，但填上
	//     本地写权限是常规做法（RDMA 示例会用到）。
	{
		ibv_qp_attr attr{};
		attr.qp_state = IBV_QPS_INIT;
		attr.pkey_index = 0;	// 默认分区 0xffff
		attr.port_num = kPort;
		attr.qp_access_flags = IBV_ACCESS_LOCAL_WRITE | IBV_ACCESS_REMOTE_WRITE |
						IBV_ACCESS_REMOTE_READ;

		const int mask = IBV_QP_STATE | IBV_QP_PKEY_INDEX | IBV_QP_PORT |
					IBV_QP_ACCESS_FLAGS;
		if (ibv_modify_qp(qp.get(), &attr, mask) != 0) {
			std::fprintf(stderr, "modify_qp(INIT) 失败: %s\n",
				std::strerror(errno));
			return EXIT_FAILURE;
		}
	}

	// (b) RTR（Ready To Receive）：告诉 QP “对端是谁”。
	//     dest_qp_num 是对端 QPN（loopback 时就是自己）；
	//     rq_psn / sq_psn 是包序号起点，两端必须一致（这里都用 0）；
	//     path_mtu 取端口当前 MTU；min_rnr_timer 是收到 RNR NAK 后至少等
	//     多久（编码值 12 ≈ 0.64 ms，是常见默认）；
	//     max_dest_rd_atomic 是本端允许对端同时发起的 RDMA READ/ATOMIC 数。
	//     ah_attr 是“地址向量”：RoCE 必须用 GRH（is_global=1 + dgid），
	//     IB 用 LID（is_global=0 + dlid）。
	{
		ibv_qp_attr attr{};
		attr.qp_state = IBV_QPS_RTR;
		attr.dest_qp_num = qp->qp_num;
		attr.path_mtu = port.active_mtu;
		attr.rq_psn = 0;
		attr.max_dest_rd_atomic = 1;
		attr.min_rnr_timer = 12;
		attr.ah_attr.port_num = kPort;
		attr.ah_attr.sl = 0;	  // 服务级别，0 即可
		attr.ah_attr.src_path_bits = 0;
		attr.ah_attr.is_global = 0;

		ibv_gid gid{};
		if (roce) {
			// RoCE：LID 无意义，必须用 GID 路由。这里取端口 1 的
			// GID 表第 0 项，并把 is_global 置 1 表示带 GRH。
			if (ibv_query_gid(ctx.get(), kPort, 0, &gid) != 0) {
				std::fprintf(stderr, "ibv_query_gid 失败: %s\n",
				std::strerror(errno));
				return EXIT_FAILURE;
			}
			attr.ah_attr.is_global = 1;
			attr.ah_attr.grh.dgid = gid;
			attr.ah_attr.grh.sgid_index = 0;
			attr.ah_attr.grh.hop_limit = 64;
			attr.ah_attr.grh.traffic_class = 0;
			attr.ah_attr.grh.flow_label = 0;
		} else {
			// InfiniBand：直接用本地 LID 当目的 LID（loopback）。
			attr.ah_attr.dlid = port.lid;
		}

		const int mask = IBV_QP_STATE | IBV_QP_AV | IBV_QP_PATH_MTU |
					IBV_QP_DEST_QPN | IBV_QP_RQ_PSN |
					IBV_QP_MAX_DEST_RD_ATOMIC | IBV_QP_MIN_RNR_TIMER;
		if (ibv_modify_qp(qp.get(), &attr, mask) != 0) {
			std::fprintf(stderr, "modify_qp(RTR) 失败: %s\n",
				std::strerror(errno));
			return EXIT_FAILURE;
		}
	}

	// (c) RTS（Ready To Send）：可以发数据了。
	//     timeout 是 ACK 超时（编码值，14 ≈ 4.096s * 2^14？实际是按
	//     4.096us * 2^timeout 的粒度，越大等越久）；
	//     retry_cnt / rnr_retry 是重传次数，7 表示无限重试；
	//     max_rd_atomic 是本端能同时在途的 RDMA READ/ATOMIC 数。
	{
		ibv_qp_attr attr{};
		attr.qp_state = IBV_QPS_RTS;
		attr.sq_psn = 0;
		attr.timeout = 14;
		attr.retry_cnt = 7;
		attr.rnr_retry = 7;
		attr.max_rd_atomic = 1;

		const int mask = IBV_QP_STATE | IBV_QP_SQ_PSN | IBV_QP_TIMEOUT |
					IBV_QP_RETRY_CNT | IBV_QP_RNR_RETRY |
					IBV_QP_MAX_QP_RD_ATOMIC;
		if (ibv_modify_qp(qp.get(), &attr, mask) != 0) {
			std::fprintf(stderr, "modify_qp(RTS) 失败: %s\n",
				std::strerror(errno));
			return EXIT_FAILURE;
		}
	}

	// -----------------------------------------------------------------
	// 6. 先投递接收 WR（RX 必须先于数据到达，否则对端收到 RNR NAK）
	//
	// 这里演示链表：recv_wr[i].next = &recv_wr[i+1]，只需要一次
	// ibv_post_recv 就交 4 个。每个接收 WR 指定“数据落地到哪段本地内存”，
	// 目的地址由接收方决定（SEND 是“写到一个本地缓冲”的语义）。
	// -----------------------------------------------------------------
	std::array<ibv_sge, kRecvCount> recv_sge{};
	std::array<ibv_recv_wr, kRecvCount> recv_wr{};
	std::array<int, kRecvCount> recv_slot{};

	for (int i = 0; i < kRecvCount; ++i) {
		const std::size_t off = kRecvArea + static_cast<std::size_t>(i) * kRecvSlot;
		recv_sge[i].addr = reinterpret_cast<std::uintptr_t>(arena.data() + off);
		recv_sge[i].length = kRecvSlot;
		recv_sge[i].lkey = mr->lkey;

		recv_wr[i].wr_id = 100u + static_cast<std::uint64_t>(i);  // 自定义标记
		recv_wr[i].next = (i + 1 < kRecvCount) ? &recv_wr[i + 1] : nullptr;
		recv_wr[i].sg_list = &recv_sge[i];
		recv_wr[i].num_sge = 1;
		recv_slot[i] = i;
	}

	ibv_recv_wr *bad_recv = nullptr;
	// 返回值：0 成功；非 0 是 errno（如 EINVAL/ENOMEM），bad_recv 指向第一个
	// 被拒绝的 WR。注意：bad_recv 之前的 WR 已经挂在 RQ 上了，不会回滚。
	const int rc_recv = ibv_post_recv(qp.get(), &recv_wr[0], &bad_recv);
	if (rc_recv != 0) {
		const long idx = bad_recv ? static_cast<long>(bad_recv - recv_wr.data()) : -1;
		std::fprintf(stderr, "ibv_post_recv 失败: %s，bad_wr 下标=%ld\n",
				std::strerror(rc_recv), idx);
		return EXIT_FAILURE;
	}
	std::printf("已投递 %d 个 RECV WR（wr_id=100..103，一条链一次调用）\n",
			kRecvCount);

	// -----------------------------------------------------------------
	// 7. 投递发送 WR
	//
	// 每个 SEND 从发送区取 64 字节。IBV_SEND_SIGNALED 让这一项产生 CQE；
	// 如果只关心“整批都发完了”，可以只给最后一个置 SIGNALED（RC 上按序
	// 执行，最后一个完成即前面都完成），示例为了让每个 wr_id 都能对上，
	// 全部置 SIGNALED。
	// -----------------------------------------------------------------
	std::array<ibv_sge, kSendCount> send_sge{};
	std::array<ibv_send_wr, kSendCount> send_wr{};

	for (int i = 0; i < kSendCount; ++i) {
		char *p = arena.data() + kSendArea + static_cast<std::size_t>(i) * kMsgLen;
		// 每段内容不同，方便接收侧验证“按序到达、没有串包”。
		std::snprintf(p, kMsgLen, "message #%d from ibv_post_send", i + 1);

		send_sge[i].addr = reinterpret_cast<std::uintptr_t>(p);
		send_sge[i].length = kMsgLen;
		send_sge[i].lkey = mr->lkey;

		send_wr[i].wr_id = 1u + static_cast<std::uint64_t>(i);
		send_wr[i].next = (i + 1 < kSendCount) ? &send_wr[i + 1] : nullptr;
		send_wr[i].sg_list = &send_sge[i];
		send_wr[i].num_sge = 1;
		send_wr[i].opcode = IBV_WR_SEND;   // 双向消息，对端要有 RECV WR
		send_wr[i].send_flags = IBV_SEND_SIGNALED;
	}

	ibv_send_wr *bad_send = nullptr;
	const int rc_send = ibv_post_send(qp.get(), &send_wr[0], &bad_send);
	if (rc_send != 0) {
		const long idx = bad_send ? static_cast<long>(bad_send - send_wr.data()) : -1;
		std::fprintf(stderr, "ibv_post_send 失败: %s，bad_wr 下标=%ld\n",
			std::strerror(rc_send), idx);
		return EXIT_FAILURE;
	}
	std::printf("已投递 %d 个 SEND WR（wr_id=1..3，全部 SIGNALED）\n", kSendCount);

	// -----------------------------------------------------------------
	// 8. 收完成项
	//
	// 3 个 SEND + 3 个 RECV = 6 个 CQE。发送完成表示“对端已确认（RC 的
	// ACK）”，接收完成表示“数据已写进本地缓冲”。
	// wc.byte_len 对 RECV 是有效数据长度，对 SEND 无意义。
	// -----------------------------------------------------------------
	std::array<ibv_wc, kCqDepth> wcs{};
	const int total = kSendCount + kSendCount;
	const int got = poll_until(cq.get(), total, std::span{wcs}.first(total), 2000);
	if (got < 0)
		return EXIT_FAILURE;

	std::printf("\n轮询到 %d 个完成项：\n", got);
	bool all_ok = true;
	int recv_done = 0;
	for (int i = 0; i < got; ++i) {
		print_wc(wcs[i]);
		if (wcs[i].status != IBV_WC_SUCCESS) {
			all_ok = false;
			continue;
		}
		// IBV_WC_RECV 是置了 bit7 的值，用 & 判断“这是不是接收完成”。
		if (wcs[i].opcode & IBV_WC_RECV) {
			++recv_done;
			const std::size_t off =
				kRecvArea + static_cast<std::size_t>(recv_slot[(wcs[i].wr_id - 100u)]) *
						kRecvSlot;
			// 接收缓冲不保证以 '\0' 结尾，用显式长度打印。
			std::printf("        收到 %u 字节: \"%.*s\"\n", wcs[i].byte_len,
					static_cast<int>(wcs[i].byte_len),
					arena.data() + off);
		}
	}
	if (got != total)
		std::fprintf(stderr, "超时：只拿到 %d/%d 个完成项\n", got, total);

	// -----------------------------------------------------------------
	// 9. 演示 bad_wr
	//
	// QP 的 max_send_sge = 1。下面这条链第一项合法、第二项故意挂 2 个 SGE。
	// 规范行为：第一项已经入队（会正常发出并产生完成项），第二项被拒绝，
	// ibv_post_send 返回 EINVAL，bad_wr 指向第二项。
	// 注意：也有厂商在用户态先整链校验再整体入队，那样第一项不会发出；
	// 下面的轮询带超时，就是为了两种情况都能收场。
	// -----------------------------------------------------------------
	std::printf("\n--- bad_wr 演示：第 2 项 num_sge=2 超过 QP 的 max_send_sge=1 ---\n");

	std::array<ibv_sge, 2> bad_sge{};
	std::array<ibv_send_wr, 2> bad_chain{};
	char *demo_msg = arena.data() + kSendArea + 3 * kMsgLen;
	std::snprintf(demo_msg, kMsgLen, "before the bad WR");

	bad_sge[0].addr = reinterpret_cast<std::uintptr_t>(demo_msg);
	bad_sge[0].length = kMsgLen;
	bad_sge[0].lkey = mr->lkey;
	// 第二项复用同一个 SGE 数组的两个元素（内容不重要，反正会被拒）。
	bad_sge[1] = bad_sge[0];

	bad_chain[0].wr_id = 900;
	bad_chain[0].sg_list = &bad_sge[0];
	bad_chain[0].num_sge = 1;
	bad_chain[0].opcode = IBV_WR_SEND;
	bad_chain[0].send_flags = IBV_SEND_SIGNALED;
	bad_chain[0].next = &bad_chain[1];

	bad_chain[1].wr_id = 901;
	bad_chain[1].sg_list = &bad_sge[0];
	bad_chain[1].num_sge = 2;			// ← 非法：超过 1
	bad_chain[1].opcode = IBV_WR_SEND;
	bad_chain[1].send_flags = IBV_SEND_SIGNALED;
	bad_chain[1].next = nullptr;

	ibv_send_wr *rejected = nullptr;
	const int rc_bad = ibv_post_send(qp.get(), &bad_chain[0], &rejected);
	if (rc_bad == 0) {
		std::fprintf(stderr, "意外：非法的 WR 链竟然提交成功了\n");
	} else {
		const long idx = rejected ? static_cast<long>(rejected - bad_chain.data()) : -1;
		std::printf("ibv_post_send 返回 %d（%s），bad_wr 指向第 %ld 项（wr_id=%s）\n",
				rc_bad, std::strerror(rc_bad), idx,
				(rejected && rejected->wr_id == 901) ? "901，符合预期" : "未知");
	}

	// 第一项（wr_id=900）已被发出，配第 4 个 RECV（wr_id=103）。正常会收到
	// 2 个 CQE；若驱动先整链校验，则一个都没有，靠超时结束。
	std::array<ibv_wc, 4> extra{};
	const int got2 = poll_until(cq.get(), 2, extra, 300);
	std::printf("bad_wr 演示后轮询到 %d 个完成项%s\n", got2,
			got2 == 0 ? "（该驱动整链校验后才入队，属正常差异）" : "");
	for (int i = 0; i < got2; ++i)
		print_wc(extra[i]);

	// -----------------------------------------------------------------
	// 10. 收尾
	//
	// 这里的资源都是 RAII 对象，析构顺序由声明顺序决定（反序）：
	// QP → CQ → MR → PD → context。QP 必须先于 CQ/MR 销毁，否则
	// ibv_destroy_cq/ibv_dereg_mr 会返回 EBUSY。
	// 演示程序结束即释放，真实程序里 QP 一般要长期复用。
	// -----------------------------------------------------------------
	std::printf("\n完成：%s\n", (all_ok && recv_done == kSendCount) ? "全部成功" : "有失败项");
	return (all_ok && recv_done == kSendCount) ? EXIT_SUCCESS : EXIT_FAILURE;
}
