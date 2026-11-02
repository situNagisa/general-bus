/*
rdma_write_read.cpp —— RDMA 的两把“远程内存直写/直读”：IBV_WR_RDMA_WRITE / IBV_WR_RDMA_READ

【这是什么】
RDMA（Remote Direct Memory Access）和 SEND/RECV 最大的区别：SEND 是“把消息
塞进对端的接收队列，对端软件必须提前投递 RECV WR 并处理完成项”；而
IBV_WR_RDMA_WRITE / IBV_WR_RDMA_READ 是“直接按地址读写对端进程已经注册好的
内存”，对端 CPU/软件完全不参与：
	- RDMA_WRITE：把本地内存的一批字节写到对端的 [remote_addr, +len)。
	- RDMA_READ ：把对端的 [remote_addr, +len) 读回本地内存。
发起方只需要知道对端 MR 的“远程地址 + rkey”（一般通过带外交换，比如先用
TCP socket 交换这两个数，或者用 rdma_cm 建连时拿到）。对端不需要发布任何
RECV WR，也收不到任何“接收完成项”。

【形态】
	- 一次 ibv_post_send 仍然可以交一整条 WR 链；每个 WR 是一次独立的
	WRITE 或 READ，对应线上一个 RDMA 消息（会被拆成多个 MTU 大小的包）。
	- 每个 WR 可带多个 SGE：
	WRITE：SGE 描述本地要发出的若干段（scatter/gather），写到对端的是
		从 remote_addr 开始的一段连续内存。
	READ ：SGE 描述本地接收数据的若干段，从对端的 remote_addr 开始连续读取。
	- 完成：只有本地 CQ 上一个完成项（需 IBV_SEND_SIGNALED），opcode 为
	IBV_WC_RDMA_WRITE / IBV_WC_RDMA_READ；对端没有任何完成项、没有中断。
	READ 的 CQE 里 byte_len 是实际读到的字节数；WRITE 的 byte_len 无意义。
	- 内存：两端都得在各自的 PD 里注册 MR。对端的 MR 必须带
	IBV_ACCESS_REMOTE_WRITE（被 WRITE）/ IBV_ACCESS_REMOTE_READ（被 READ）；
	否则请求会在对端被拒绝，发起方拿到 IBV_WC_REM_ACCESS_ERR。
	- WRITE 完成（收到 ACK）即意味着数据已经落在对端内存里；但对端 CPU 要
	“看见”它仍需同步（通常用另一个 SEND 送一个“已完成”标记，或轮询标志位）。

【需要的内核/硬件条件】
	- 真实 RDMA 网卡，或 soft-RoCE（rdma_rxe）。WSL2 一般两者都没有，本程序
	会在 ibv_get_device_list 为空时打印提示退出。

【本示例做什么】
	在同一个进程里建一个 RC QP 并自连（loopback），把一块注册为“对端内存”
	的缓冲区当作远端：
	1) RDMA_WRITE 把自己的字符串写进去（不发 RECV WR，验证照样完成）；
	2) RDMA_READ 读回来，校验内容；
	3) 故意 READ 一块只注册了本地权限的 MR，观察 IBV_WC_REM_ACCESS_ERR，
		以及 QP 进入 IBV_QPS_ERR。

【编译】
	g++ -std=c++26 -O2 -Wall -Wextra -o /tmp/native_rdma_write_read \
		rdma_write_read.cpp -libverbs
【运行】/tmp/native_rdma_write_read
【预期输出】设备/端口信息；WRITE 的 1 个 IBV_WC_RDMA_WRITE 完成项；对端缓冲区
内容与发送内容一致；READ 的 1 个 IBV_WC_RDMA_READ 完成项且 byte_len 正确、内容
一致；最后 READ 只读 MR 时出现一个 IBV_WC_REM_ACCESS_ERR，QP 变为 IBV_QPS_ERR。
无设备时只打印提示，以失败退出。
*/

#include <infiniband/verbs.h>

#include <array>
#include <cerrno>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <span>
#include <thread>

namespace {

// ---------------------------------------------------------------------------
// 一、RAII 包装（和 post_send_recv 示例同样的思路）
//
// libibverbs 全是裸指针；用 unique_ptr + deleter 管理，声明顺序保证析构
// 顺序为 QP → CQ → MR → PD → context（子对象先于父对象销毁）。
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

	// NULL 表示调用失败（errno 有原因）；非 NULL 但 count==0 表示没有设备。
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

// 轮询 CQ 直到拿到 need 个完成项或超时。返回实际个数，出错返回 -1。
// 注意 CQ 为空时 ibv_poll_cq 返回 0（不是错误），必须自己循环/让出 CPU。
int poll_until(ibv_cq *cq, int need, std::span<ibv_wc> out, int timeout_ms)
{
	int got = 0;
	const auto deadline =
		std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);

	while (got < need) {
		const int n = ibv_poll_cq(cq, static_cast<int>(out.size()) - got, out.data() + got);
		if (n < 0) {
			std::fprintf(stderr, "ibv_poll_cq 出错（返回值 %d）\n", n);
			return -1;
		}
		got += n;
		if (got >= need || std::chrono::steady_clock::now() >= deadline)
			break;
		std::this_thread::sleep_for(std::chrono::microseconds(50));
	}
	return got;
}

void print_wc(const ibv_wc &wc)
{
	// status 非 SUCCESS 时，opcode/byte_len 内容不可信，只能看 status。
	std::printf("  CQE: wr_id=%llu status=%s(%d) opcode=%s byte_len=%u\n",
		static_cast<unsigned long long>(wc.wr_id), ibv_wc_status_str(wc.status),
		static_cast<int>(wc.status), wc_opcode_name(wc.opcode), wc.byte_len);
}

constexpr std::uint8_t kPort = 1;
constexpr std::size_t kLen = 48;	// 每次远程访问的字节数
constexpr int kCqDepth = 16;

} // namespace

int main()
{
	// -----------------------------------------------------------------
	// 1. 打开设备、查端口
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

	ibv_device *dev = list.front();
	std::printf("设备: %s\n", ibv_get_device_name(dev));

	Context ctx{ibv_open_device(dev)};
	if (!ctx) {
		std::fprintf(stderr, "ibv_open_device 失败: %s\n", std::strerror(errno));
		return EXIT_FAILURE;
	}

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
	if (port.state != IBV_PORT_ACTIVE) {
		std::fprintf(stderr, "端口未处于 ACTIVE 状态，无法建链，退出。\n");
		return EXIT_FAILURE;
	}

	// -----------------------------------------------------------------
	// 2. PD、MR、CQ
	//
	// 三个 MR，权限刻意不同：
	//   local  ：本端写/读（RDMA_WRITE 的源、RDMA_READ 的目的），只要 LOCAL_WRITE
	//   peer   ：扮演“对端内存”，必须 REMOTE_WRITE + REMOTE_READ 才能被远程读写
	//   readonly：只注册 LOCAL_WRITE，最后用它演示远程访问被拒绝
	// 真实程序里 peer 位于另一台机器，这里在同一进程里模拟，rkey 的语义完全一样。
	// -----------------------------------------------------------------
	Pd pd{ibv_alloc_pd(ctx.get())};
	if (!pd) {
		std::fprintf(stderr, "ibv_alloc_pd 失败: %s\n", std::strerror(errno));
		return EXIT_FAILURE;
	}

	std::array<char, 8192> local{};	  // [0, 4096) 写源；[4096, 8192) 读回目的地
	std::array<char, 4096> peer{};
	std::array<char, 4096> readonly{};
	constexpr std::size_t kReadbackOff = 4096;

	// IBV_ACCESS_LOCAL_WRITE：允许网卡把收到的数据写进这块内存（接收/RDMA_READ）。
	Mr local_mr{ibv_reg_mr(pd.get(), local.data(), local.size(), IBV_ACCESS_LOCAL_WRITE)};
	if (!local_mr) {
		std::fprintf(stderr, "reg_mr(local) 失败: %s\n", std::strerror(errno));
		return EXIT_FAILURE;
	}

	// 对端要能被 RDMA_WRITE 和 RDMA_READ，所以两个 REMOTE_* 权限都要。
	// REMOTE_WRITE 是最危险的一项权限：拿到 rkey 的人可以直接改你的内存。
	Mr peer_mr{ibv_reg_mr(pd.get(), peer.data(), peer.size(),
		IBV_ACCESS_LOCAL_WRITE | IBV_ACCESS_REMOTE_WRITE |
		IBV_ACCESS_REMOTE_READ)};
	if (!peer_mr) {
		std::fprintf(stderr, "reg_mr(peer) 失败: %s\n", std::strerror(errno));
		return EXIT_FAILURE;
	}

	Mr ro_mr{ibv_reg_mr(pd.get(), readonly.data(), readonly.size(), IBV_ACCESS_LOCAL_WRITE)};
	if (!ro_mr) {
		std::fprintf(stderr, "reg_mr(readonly) 失败: %s\n", std::strerror(errno));
		return EXIT_FAILURE;
	}

	std::printf("peer MR: addr=%p len=%zu lkey=0x%x rkey=0x%x\n",
		static_cast<void *>(peer.data()), peer.size(), peer_mr->lkey, peer_mr->rkey);
	std::printf("      这两个数（addr + rkey）在真实程序里要发给对端才能被远程访问。\n");

	Cq cq{ibv_create_cq(ctx.get(), kCqDepth, nullptr, nullptr, 0)};
	if (!cq) {
		std::fprintf(stderr, "ibv_create_cq 失败: %s\n", std::strerror(errno));
		return EXIT_FAILURE;
	}

	// -----------------------------------------------------------------
	// 3. 建 RC QP 并自连（RESET → INIT → RTR → RTS）
	//
	// 这里只需要一个 QP：发起 WRITE/READ 的是它，被访问的“远端”也是它自己
	// 所属的设备/端口。RDMA_WRITE 落到 peer_mr 时不会再产生任何接收完成，
	// 所以本次连一个 RECV WR 都不投递。
	// -----------------------------------------------------------------
	ibv_qp_init_attr qp_ia{};
	qp_ia.send_cq = cq.get();
	qp_ia.recv_cq = cq.get();
	qp_ia.qp_type = IBV_QPT_RC;
	qp_ia.sq_sig_all = 0;	// 只让置了 IBV_SEND_SIGNALED 的 WR 产生 CQE
	qp_ia.cap.max_send_wr = 8;
	qp_ia.cap.max_recv_wr = 8;
	qp_ia.cap.max_send_sge = 1;
	qp_ia.cap.max_recv_sge = 1;

	Qp qp{ibv_create_qp(pd.get(), &qp_ia)};
	if (!qp) {
		std::fprintf(stderr, "ibv_create_qp 失败: %s\n", std::strerror(errno));
		return EXIT_FAILURE;
	}
	std::printf("QP: qpn=%u\n", qp->qp_num);

	// INIT：本端权限里必须包含对端将来要用的远程权限吗？不需要——那是被访问
	// 一端 MR 的权限；qp_access_flags 描述“本 QP 允许对端对我做什么”，示例里
	// 填全，便于后续扩展。
	{
		ibv_qp_attr attr{};
		attr.qp_state = IBV_QPS_INIT;
		attr.pkey_index = 0;
		attr.port_num = kPort;
		attr.qp_access_flags = IBV_ACCESS_LOCAL_WRITE | IBV_ACCESS_REMOTE_WRITE |
			IBV_ACCESS_REMOTE_READ;
		if (ibv_modify_qp(qp.get(), &attr,
			IBV_QP_STATE | IBV_QP_PKEY_INDEX | IBV_QP_PORT |
			IBV_QP_ACCESS_FLAGS) != 0) {
			std::fprintf(stderr, "modify_qp(INIT) 失败: %s\n", std::strerror(errno));
			return EXIT_FAILURE;
		}
	}

	// RTR：dest_qp_num 指向自己（loopback）；RoCE 用 GID+GRH，IB 用 LID。
	// max_dest_rd_atomic ≥ 1 是 RDMA_READ 能工作的前提之一（READ 是请求-响应
	// 型操作，要占对端的 RD-ATOMIC 额度）。
	{
		ibv_qp_attr attr{};
		attr.qp_state = IBV_QPS_RTR;
		attr.dest_qp_num = qp->qp_num;
		attr.path_mtu = port.active_mtu;
		attr.rq_psn = 0;
		attr.max_dest_rd_atomic = 1;
		attr.min_rnr_timer = 12;
		attr.ah_attr.port_num = kPort;
		attr.ah_attr.sl = 0;
		attr.ah_attr.is_global = 0;

		ibv_gid gid{};
		if (roce) {
			// RoCE 没有 LID，必须用 GID 路由：is_global=1 让 AV 携带 GRH。
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
		} else {
			attr.ah_attr.dlid = port.lid;
		}

		if (ibv_modify_qp(qp.get(), &attr,
			IBV_QP_STATE | IBV_QP_AV | IBV_QP_PATH_MTU | IBV_QP_DEST_QPN |
			IBV_QP_RQ_PSN | IBV_QP_MAX_DEST_RD_ATOMIC |
			IBV_QP_MIN_RNR_TIMER) != 0) {
			std::fprintf(stderr, "modify_qp(RTR) 失败: %s\n", std::strerror(errno));
			return EXIT_FAILURE;
		}
	}

	// RTS：max_rd_atomic 是本端同时在途的 READ/ATOMIC 数，至少 1。
	{
		ibv_qp_attr attr{};
		attr.qp_state = IBV_QPS_RTS;
		attr.sq_psn = 0;
		attr.timeout = 14;
		attr.retry_cnt = 7;
		attr.rnr_retry = 7;
		attr.max_rd_atomic = 1;
		if (ibv_modify_qp(qp.get(), &attr,
			IBV_QP_STATE | IBV_QP_SQ_PSN | IBV_QP_TIMEOUT |
			IBV_QP_RETRY_CNT | IBV_QP_RNR_RETRY |
			IBV_QP_MAX_QP_RD_ATOMIC) != 0) {
			std::fprintf(stderr, "modify_qp(RTS) 失败: %s\n", std::strerror(errno));
			return EXIT_FAILURE;
		}
	}

	std::array<ibv_wc, kCqDepth> wcs{};

	// =================================================================
	// 4. RDMA_WRITE：本地 local[0..kLen) → 对端 peer[0..kLen)
	//
	// 关键字段：
	//   opcode       = IBV_WR_RDMA_WRITE
	//   send_flags   = IBV_SEND_SIGNALED（要 CQE 就必须置；不置则彻底静默）
	//   sge.addr/length 描述本地源数据（必须在已注册的本地内存里）
	//   wr.rdma.remote_addr = 对端缓冲区地址（这里是 peer.data()）
	//   wr.rdma.rkey        = 对端 MR 的 rkey（来自 peer_mr->rkey）
	// 对端不需要任何 RECV WR——这正是与 SEND 的根本区别。
	// =================================================================
	std::snprintf(local.data(), kLen, "hello via RDMA_WRITE, no peer CPU involved!");
	std::printf("\n--- 1) RDMA_WRITE %zu 字节 到 peer[0] ---\n", kLen);

	ibv_sge write_sge{};
	write_sge.addr = reinterpret_cast<std::uintptr_t>(local.data());
	write_sge.length = kLen;
	write_sge.lkey = local_mr->lkey;	// 本地源内存的 lkey

	ibv_send_wr write_wr{};
	write_wr.wr_id = 1;
	write_wr.sg_list = &write_sge;
	write_wr.num_sge = 1;
	write_wr.opcode = IBV_WR_RDMA_WRITE;
	write_wr.send_flags = IBV_SEND_SIGNALED;
	write_wr.next = nullptr;
	write_wr.wr.rdma.remote_addr = reinterpret_cast<std::uintptr_t>(peer.data());
	write_wr.wr.rdma.rkey = peer_mr->rkey;	// 没有它会被对端拒绝（REM_ACCESS_ERR）

	ibv_send_wr *bad = nullptr;
	int rc = ibv_post_send(qp.get(), &write_wr, &bad);
	if (rc != 0) {
		// 这里出错只说明 WR 没进队列（参数非法/队列满），返回 errno，
		// bad 指向被拒的那一项。
		std::fprintf(stderr, "post_send(RDMA_WRITE) 失败: %s\n", std::strerror(rc));
		return EXIT_FAILURE;
	}

	int got = poll_until(cq.get(), 1, std::span{wcs}.first(1), 2000);
	if (got != 1) {
		std::fprintf(stderr, "等 RDMA_WRITE 完成项超时（%d/1）\n", got);
		return EXIT_FAILURE;
	}
	print_wc(wcs[0]);
	if (wcs[0].status != IBV_WC_SUCCESS) {
		std::fprintf(stderr, "RDMA_WRITE 失败: %s\n", ibv_wc_status_str(wcs[0].status));
		return EXIT_FAILURE;
	}

	// WRITE 的完成意味着数据已写入对端内存，这里直接在本进程里查 peer 内容。
	// （真实跨机场景对端 CPU 看到数据需要额外同步，见文件头说明。）
	const bool write_ok = std::memcmp(peer.data(), local.data(), kLen) == 0;
	std::printf("  peer 内容: \"%.*s\"\n", static_cast<int>(kLen), peer.data());
	std::printf("  与写入内容一致: %s（对端没有收到任何 CQE）\n", write_ok ? "是" : "否");

	// =================================================================
	// 5. RDMA_READ：对端 peer[0..kLen) → 本地 local[4096..)
	//
	// READ 是“发起方主动拉取”，同样只需要 remote_addr + rkey：
	//   - sg_list 指向本地接收缓冲（必须注册，且 lkey 正确）
	//   - 完成项里 byte_len 是实际读到的字节数，可以拿来做校验
	// 对端 CPU 同样不参与、不产生完成项。
	// =================================================================
	std::printf("\n--- 2) RDMA_READ %zu 字节 从 peer[0] ---\n", kLen);
	std::memset(local.data() + kReadbackOff, 0, kLen);	// 先清空，便于判断是否真读到

	ibv_sge read_sge{};
	read_sge.addr = reinterpret_cast<std::uintptr_t>(local.data() + kReadbackOff);
	read_sge.length = kLen;
	read_sge.lkey = local_mr->lkey;

	ibv_send_wr read_wr{};
	read_wr.wr_id = 2;
	read_wr.sg_list = &read_sge;
	read_wr.num_sge = 1;
	read_wr.opcode = IBV_WR_RDMA_READ;
	read_wr.send_flags = IBV_SEND_SIGNALED;
	read_wr.next = nullptr;
	read_wr.wr.rdma.remote_addr = reinterpret_cast<std::uintptr_t>(peer.data());
	read_wr.wr.rdma.rkey = peer_mr->rkey;

	rc = ibv_post_send(qp.get(), &read_wr, &bad);
	if (rc != 0) {
		std::fprintf(stderr, "post_send(RDMA_READ) 失败: %s\n", std::strerror(rc));
		return EXIT_FAILURE;
	}

	got = poll_until(cq.get(), 1, std::span{wcs}.first(1), 2000);
	if (got != 1) {
		std::fprintf(stderr, "等 RDMA_READ 完成项超时（%d/1）\n", got);
		return EXIT_FAILURE;
	}
	print_wc(wcs[0]);
	// READ 成功时 byte_len 应该等于请求长度；READ 的 CQE 也用于传递“读到了多少”。
	const bool read_ok = wcs[0].status == IBV_WC_SUCCESS &&
			wcs[0].byte_len == kLen &&
			std::memcmp(local.data() + kReadbackOff, local.data(), kLen) == 0;
	std::printf("  读回内容: \"%.*s\"（校验 %s）\n",
		static_cast<int>(kLen), local.data() + kReadbackOff,
		read_ok ? "通过" : "失败");

	// =================================================================
	// 6. 权限演示：对端 MR 没注册 REMOTE_READ，READ 会被对端拒绝
	//
	// readonly 只带 IBV_ACCESS_LOCAL_WRITE。发起 READ 时它不具备远程读权限，
	// 对端返回“访问错误”，发起方 CQE 的 status 为 IBV_WC_REM_ACCESS_ERR。
	// 这类错误是“线上回来的错误”，不是 post 失败；同时 RC QP 会进入
	// IBV_QPS_ERR，之后这个 QP 不能再用了（必须 reset 重建），所以放最后。
	// =================================================================
	std::printf("\n--- 3) 故意 READ 没有 REMOTE_READ 权限的 MR ---\n");

	ibv_sge bad_sge{};
	bad_sge.addr = reinterpret_cast<std::uintptr_t>(local.data() + kReadbackOff);
	bad_sge.length = kLen;
	bad_sge.lkey = local_mr->lkey;

	ibv_send_wr perm_wr{};
	perm_wr.wr_id = 3;
	perm_wr.sg_list = &bad_sge;
	perm_wr.num_sge = 1;
	perm_wr.opcode = IBV_WR_RDMA_READ;
	perm_wr.send_flags = IBV_SEND_SIGNALED;
	perm_wr.wr.rdma.remote_addr = reinterpret_cast<std::uintptr_t>(readonly.data());
	perm_wr.wr.rdma.rkey = ro_mr->rkey;

	rc = ibv_post_send(qp.get(), &perm_wr, &bad);
	if (rc != 0) {
		std::fprintf(stderr, "post_send 失败: %s\n", std::strerror(rc));
		return EXIT_FAILURE;
	}

	got = poll_until(cq.get(), 1, std::span{wcs}.first(1), 2000);
	bool perm_ok = false;
	if (got == 1) {
		print_wc(wcs[0]);
		perm_ok = wcs[0].status == IBV_WC_REM_ACCESS_ERR;
		std::printf("  预期 IBV_WC_REM_ACCESS_ERR，实际 %s\n",
				perm_ok ? "一致" : "不一致");
	} else {
		std::fprintf(stderr, "等完成项超时（%d/1）\n", got);
	}

	// 用 ibv_query_qp 确认 QP 已经被打进错误态（这是 RC QP 的规范行为）。
	{
		ibv_qp_attr qa{};
		ibv_qp_init_attr qi{};
		if (ibv_query_qp(qp.get(), &qa, IBV_QP_STATE, &qi) == 0) {
			std::printf("  出错后 QP 状态: %d（%s，6=IBV_QPS_ERR，需要 reset 才能复用）\n",
				static_cast<int>(qa.qp_state),
				qa.qp_state == IBV_QPS_ERR ? "IBV_QPS_ERR" : "其它");
		}
	}

	std::printf("\n完成：WRITE %s，READ %s，权限拒绝 %s\n",
		write_ok ? "成功" : "失败", read_ok ? "成功" : "失败",
		perm_ok ? "符合预期" : "不符合预期");
	// RAII 析构顺序：qp → cq → ro_mr → peer_mr → local_mr → pd → ctx。
	return (write_ok && read_ok && perm_ok) ? EXIT_SUCCESS : EXIT_FAILURE;
}
