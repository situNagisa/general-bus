// 不在本机编译：Linux 内核态 xHCI 主机控制器驱动片段，需要内核头文件、DMA 一致性内存和真实 xHCI 控制器；内核代码只能用 C，所以本文件保持 .c 而不改成 .cpp。
//
// 这是什么
// --------
// xHCI（eXtensible Host Controller Interface，USB 3.x/2.x 主机控制器接口）
// 驱动把一次 USB 控制传输（control transfer）做成一个 TD（Transfer
// Descriptor），挂到端点传输环（Transfer Ring）上，然后敲一次门铃
// （Doorbell）交给硬件。
//
// 这是驱动实现 link 的手段，link 层看不到它：ibus 的链路抽象只看到
// “提交一个控制请求 / 收到一个完成事件”；TRB 环、Cycle 位、IOC 位、
// 门铃、事件环、DMA 地址全部封在主机控制器驱动里。
//
// 形态（一次调用交几段内存、对应几个线上单元、提交几次、完成几次）
// ----------------------------------------------------------------
//   - 线上单元：USB 控制传输本身固定是 SETUP / DATA(0..n) / STATUS 三个阶段。
//   - 内存段：Setup 包（8 字节）不单独占 DMA 内存，直接内嵌在 Setup Stage
//     TRB 的 Parameter 字段里（IDT=1）；Data 阶段是驱动提供的 DMA 缓冲；
//     Status 阶段没有数据缓冲（Parameter=0）。
//   - 提交次数：1 次。3 个 TRB（Setup/Data/Status；无 Data 阶段时只有 2 个）
//     用 CH（Chain）位串成同一个 TD，写完后只往 Doorbell 写一次。
//   - 完成次数：1 次。只有 Status Stage 的 IOC=1，所以整条 TD 完成时
//     事件环上只出现 1 个 Transfer Event TRB（xHCI 只在 TRB 的 IOC=1、
//     或 ISP 且发生短包、或出错时才产生 Transfer Event，见 xHCI 1.2
//     §4.9.1、§6.4.2.1）。如果 Setup/Data 也设 IOC，每个设了 IOC 的阶段
//     完成时都会各自产生一个 Transfer Event，驱动就要处理“半个 TD”。
//
// 什么时候用：所有 USB 控制传输——枚举、标准请求、类请求、HID report
// 描述符读取等等。批量/中断/等时传输用的是 Normal TRB，原理相同（每个 TD
// 的最后一个 TRB 设 IOC）。
//
// 需要的条件：xHCI 1.2 兼容主机控制器、Linux 内核 6.x 的 xhci-hcd
// 环境、DMA 一致性内存（传输环、数据缓冲）。本文件是
// drivers/usb/host/xhci-ring.c 中 xhci_queue_ctrl_tx() 的简化教学版：
// 字段按规范逐位展开，便于和规范表对照；真正合入内核时应改用 xhci.h 的
// TRB_* 宏与 queue_trb()，本文件里的 union xhci_trb / TRB_* 定义也要删掉，
// 否则会和 xhci.h 重名。
//
// 怎么编译/怎么运行：不能在本机编译。要验证只能在带 xHCI 控制器的机器上
// 修改 xhci-hcd 并重新加载模块，用 usbmon、dynamic debug（xhci_dbg）或
// 硬件跟踪看 TRB 与事件。预期现象：一次控制传输对应 2~3 条 TRB 入环、
// 1 次 doorbell 写、事件环上 1 条 Transfer Event（Completion Code=1
// Success；短包时为 13 Short Packet）。
//

#include <linux/bits.h>
#include <linux/device.h>
#include <linux/dma-mapping.h>
#include <linux/errno.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/usb/ch9.h>

/* ------------------------------------------------------------------ */
/* xHCI 1.2 §6.4.1：所有 TRB 都是 16 字节，四个 DWORD。               */
/* Parameter 是 64 位，Status 和 Control 各 32 位，全部小端。          */
/* 这里用 union 同时提供“按字段”和“按 DWORD”两种视角，便于和规范对照。 */
/* ------------------------------------------------------------------ */
union xhci_trb {
	struct {
		__le64 parameter;	/* DW0/DW1：缓冲地址、setup 数据或事件相关值 */
		__le32 status;		/* DW2：长度、完成码、Interrupter Target 等 */
		__le32 control;		/* DW3：Cycle、IOC、IDT、DIR、TRT、类型等 */
	} __packed;
	__le32 words[4];
};

/* DW3 的通用位（xHCI 1.2 §6.4.1 各 TRB 字段表） */
#define TRB_C		BIT(0)		/* Cycle：生产者/消费者环的翻转位 */
#define TRB_ENT		BIT(1)		/* Evaluate Next TRB（Status Stage 可设，本例不设） */
#define TRB_ISP		BIT(2)		/* Interrupt on Short Packet */
#define TRB_CH		BIT(4)		/* Chain：本 TRB 后面还有同一 TD 的 TRB */
#define TRB_IOC		BIT(5)		/* Interrupt on Completion：完成时产生 Transfer Event */
#define TRB_IDT		BIT(6)		/* Immediate Data：Setup Stage 必须为 1 */
#define TRB_TC		BIT(1)		/* Link TRB 的 Toggle Cycle */
#define TRB_DIR_IN	BIT(16)		/* DIR：1=IN（设备到主机） */
#define TRB_TYPE(t)	((__u32)(t) << 10)	/* DW3[15:10] TRB Type */
#define TRB_TRT(t)	((__u32)(t) << 16)	/* DW3[17:16] Setup Stage 的 Transfer Type */
#define TRB_LEN(n)	((__u32)(n) & 0x1ffff)	/* DW2[16:0] 传输长度 */

/* TRB Type（xHCI 1.2 §6.4） */
#define TRB_NORMAL	1
#define TRB_SETUP	2
#define TRB_DATA	3
#define TRB_STATUS	4
#define TRB_LINK	6
#define TRB_TRANSFER	32

/* Setup Stage 的 TRT（xHCI 1.2 §6.4.1.2.1）：注意 1 是保留值，不是 OUT！ */
#define TRT_NO_DATA	0
#define TRT_OUT		2
#define TRT_IN		3

/* 完成码（xHCI 1.2 §6.4.2 事件 TRB；Linux xhci.h 的 COMP_* 枚举） */
#define COMP_SUCCESS		1
#define COMP_SHORT_PACKET	13

// Doorbell 寄存器（xHCI 1.2 §5.3 运行时寄存器里的 Doorbell 数组）：每个 32 位
//   [7:0]   DB Target：端点门铃写端点 DCI；主机控制器门铃（DB[0]）写 0
//   [15:8]  保留
//   [31:16] DB Stream ID：使用 Stream 的端点才用
// 本片段不使用 Stream，只写 DB Target。
//

/* ------------------------------------------------------------------ */
/* 传输环：一段 DMA 一致性内存 + 生产者下标 + 生产者 Cycle State。      */
/* 环的最后一个槽位是 Link TRB，用来回绕；可用槽位是 0..num_trbs-2。    */
/* ------------------------------------------------------------------ */
struct demo_xhci_ring {
	union xhci_trb	*trbs;		/* 环的 CPU 虚拟地址（DMA 一致性内存） */
	dma_addr_t	dma;		/* 环的 DMA 地址，Link TRB 要指回它 */
	unsigned int	num_trbs;	/* 含 Link TRB 在内的槽位总数 */
	unsigned int	enqueue;	/* 生产者下一个要写的槽位 */
	__u32		cycle_state;	/* 生产者 Cycle 位，新建环从 1 开始 */
};

struct demo_xhci {
	struct device	*dev;
	void __iomem	*db;		/* Doorbell 数组基址 = MMIO + DBOFF */
};

/* 判断环里还剩多少可用槽位（Link TRB 那个槽位不算） */
static unsigned int demo_ring_space(const struct demo_xhci_ring *ring)
{
	return ring->num_trbs - 1 - ring->enqueue;
}

//
// 往环上写一个 TRB。
//
// Cycle 位是 xHCI 的生产者/消费者协议：硬件只处理 C 位等于自己当前期望值
// 的 TRB。软件每写一个 TRB 都带上 ring->cycle_state；写到 Link TRB 时，
// 先按当前 cycle 写 Link TRB，然后翻转 cycle_state，再回到环首继续写。
// 漏掉翻转或提前翻转都会让硬件“看不到”后面的 TRB（表现为传输永远不完成）。
//
static void demo_ring_push(struct demo_xhci_ring *ring, __u64 parameter,
			__u32 status, __u32 control)
{
	union xhci_trb *trb = &ring->trbs[ring->enqueue];

	trb->parameter = cpu_to_le64(parameter);
	trb->status = cpu_to_le32(status);
	/* control 里已经带了 CH/IOC/IDT/DIR/TRT/Type，这里补上 Cycle */
	trb->control = cpu_to_le32(control | ring->cycle_state);

	ring->enqueue++;
	if (ring->enqueue == ring->num_trbs - 1) {
		// 到达保留的 Link TRB 槽位：写一条指回环首的 Link TRB。
		// TC（Toggle Cycle）=1 表示硬件读完这条 Link 后翻转自己的
		// Cycle 期望值——这和软件翻转 ring->cycle_state 必须配对。
		//
		union xhci_trb *link = &ring->trbs[ring->enqueue];

		link->parameter = cpu_to_le64(ring->dma);
		link->status = 0;
		link->control = cpu_to_le32(TRB_TYPE(TRB_LINK) | TRB_TC |
			ring->cycle_state);
		ring->enqueue = 0;
		ring->cycle_state ^= 1;
	}
}

//
// 敲门铃：告诉硬件“这个 slot 的这个端点有新 TRB 了”。
// slot_id 是设备槽位号，dci 是端点 DCI（Device Context Index）。
// 对默认控制端点 EP0，DCI=1（不是 0）。写错 DCI 硬件不会报错，只是
// 去执行另一个端点的环，表现为超时。
//
static void demo_doorbell(struct demo_xhci *xhci, unsigned int slot_id,
			unsigned int dci)
{
	// 保证 TRB 内容和 Link TRB 先于门铃对硬件可见；缺了屏障，
	// 硬件可能读到半写的 TRB（尤其是写合并/弱序架构）。
	//
	wmb();
	writel(dci, xhci->db + slot_id * sizeof(__u32));
}

//
// 一个控制传输 TD 的描述：setup 包 + 可选的 Data 阶段缓冲。
// setup 是 8 字节小端 USB SETUP 包，字段见 USB 2.0 §9.3。
//
struct demo_ctrl_req {
	struct usb_ctrlrequest	setup;
	dma_addr_t		data_dma;	/* Data 阶段 DMA 地址，无数据阶段时忽略 */
	__u32			data_len;
	bool			data_in;	/* true: IN（设备→主机） */
};

//
// 把一次控制传输排进端点传输环，并敲一次门铃。
//
// 三个 TRB 的字段（xHCI 1.2 §6.4.1.2）：
//
//   Setup Stage TRB（Type=2）
//     Parameter = 8 字节 SETUP 包本身（IDT=1，所以不是指针）
//     DW2[16:0] = 8（固定）
//     DW3: IDT=1, TRT=00 无数据 / 10 OUT / 11 IN, CH=1, IOC=0
//
//   Data Stage TRB（Type=3，只有 wLength>0 才需要）
//     Parameter = 数据缓冲 DMA 地址
//     DW2[16:0] = 本 TRB 的字节数（最大 64 KiB）
//     DW3: DIR=1 表示 IN, CH=1（后面还有 Status）, IOC=0
//
//   Status Stage TRB（Type=4）
//     Parameter = 0
//     DW3: DIR 与 Data 阶段相反（Data IN → Status OUT；Data OUT 或无
//          数据 → Status IN）, CH=0（TD 结束）, IOC=1
//
// 只有 Status 设 IOC，所以整条 TD 完成时只产生 1 个 Transfer Event。
// 这也是 Linux xhci-hcd 的做法：xhci_queue_ctrl_tx() 在 Status Stage
// 的 field 上 OR 了 TRB_IOC，Setup/Data 阶段都不设。
//
static int demo_queue_control_td(struct demo_xhci *xhci,
				struct demo_xhci_ring *ring,
				unsigned int slot_id, unsigned int dci,
				const struct demo_ctrl_req *req)
{
	unsigned int need = req->data_len ? 3 : 2;
	__u32 trt;
	__u32 dw0, dw1;

	// setup 包里的 wLength 必须和实际 Data 阶段长度一致，否则硬件会按
	// 错误的 TRT/长度执行，设备侧看到畸形请求。
	if (req->data_len != le16_to_cpu(req->setup.wLength))
		return -EINVAL;

	// 一个 TD 不能跨过 Link TRB；空间不够就返回，让调用者等完成事件
	// 腾出空间后重试（真实驱动会等 ring 有空位或把 TD 拆开）。
	//
	if (demo_ring_space(ring) < need)
		return -ENOSPC;

	// Data 阶段超过 64 KiB 时要拆成多条 Data TRB，这里为保持例子聚焦
	// 只处理单条；USB 控制传输的 wLength 很少超过 64 KiB。
	//
	if (req->data_len > 64 * 1024)
		return -EINVAL;

	// TRT 告诉硬件“后面还有没有 Data Stage、方向是什么”。
	// 硬件用它检查 Status Stage 的方向是否合法。
	//
	if (req->data_len == 0)
		trt = TRT_NO_DATA;
	else
		trt = req->data_in ? TRT_IN : TRT_OUT;

	/* ---- Setup Stage TRB ---- */
	// SETUP 包的 8 个字节按小端拼进 64 位 Parameter：
	// bmRequestType(1) bRequest(1) wValue(2) wIndex(2) wLength(2)
	//
	dw0 = (__u32)req->setup.bRequestType |
		((__u32)req->setup.bRequest << 8) |
		((__u32)le16_to_cpu(req->setup.wValue) << 16);
	dw1 = (__u32)le16_to_cpu(req->setup.wIndex) |
		((__u32)le16_to_cpu(req->setup.wLength) << 16);

	demo_ring_push(ring, ((__u64)dw1 << 32) | dw0,
			TRB_LEN(8),	/* Setup 长度固定 8 */
			TRB_TYPE(TRB_SETUP) | TRB_IDT | TRB_TRT(trt) | TRB_CH);

	/* ---- Data Stage TRB（可选）---- */
	if (req->data_len) {
		demo_ring_push(ring, (__u64)req->data_dma,
				TRB_LEN(req->data_len),
				TRB_TYPE(TRB_DATA) |
				(req->data_in ? TRB_DIR_IN : 0) | TRB_CH);
	}

	/* ---- Status Stage TRB ---- */
	// 方向与 Data 阶段相反：
	//   Data IN  → Status OUT（DIR=0），主机发 0 长度 OUT 包
	//   Data OUT → Status IN （DIR=1），主机收 0 长度 IN 包
	//   无 Data  → Status IN （DIR=1）
	//
	demo_ring_push(ring, 0, 0,
			TRB_TYPE(TRB_STATUS) |
			((req->data_len && req->data_in) ? 0 : TRB_DIR_IN) |
			TRB_IOC);	/* 只有这里要完成事件 */

	/* 三个 TRB 一次提交：只敲一次门铃。 */
	demo_doorbell(xhci, slot_id, dci);
	return 0;
}

/* ------------------------------------------------------------------ */
/* 事件环：Transfer Event TRB（Type=32）的字段。                       */
/* ------------------------------------------------------------------ */
struct demo_transfer_event {
	__le64	trb_ptr;	// DW0/DW1：触发事件的 TRB 的 DMA 地址
	__le32	status;		// DW2[23:0]=Transfer Length，[31:24]=Completion Code
	// DW3[31:24]=Slot ID，[20:16]=Endpoint ID(DCI)，[2]=ED，[0]=Cycle，
	// [15:10]=32
	__le32	control;
} __packed;

struct demo_completion {
	__u64	trb_dma;	/* 对应哪个 TRB：Status Stage 的地址 */
	__u32	length;		/* 剩余未传输字节数（短包时非 0） */
	__u32	code;		/* 1=Success，13=Short Packet，其余为错误 */
};

//
// 解析事件环上的一条 Transfer Event。
//
// 正常完成时：Completion Code=1，TRB Pointer 指向 Status Stage TRB
// （因为它带 IOC），Transfer Length=0。
// 短包时：Completion Code=13，Transfer Length = 请求长度 - 实际长度，
// 驱动用“请求长度 - 剩余长度”算实际收到多少字节。
// 出错时（Stall/Babble/TRB Error 等）：Completion Code 是错误码，TRB
// Pointer 指向出错的那个 TRB；此时不管 IOC 设在哪，硬件都会报事件。
//
// 注意：事件环是消费者环，软件消费前要比较事件 TRB 的 Cycle 位和本地
// 消费者 Cycle State（CCS），消费后推进 dequeue 指针并更新 ERDP 寄存器
// （含 EHB 位）。这部分属于 xhci_irq()/xhci_handle_event() 的职责，
// 本片段只解码一条事件。
//
static void demo_decode_transfer_event(const struct demo_transfer_event *ev,
					struct demo_completion *out)
{
	__u32 status = le32_to_cpu(ev->status);
	__u32 control = le32_to_cpu(ev->control);

	out->trb_dma = le64_to_cpu(ev->trb_ptr);
	out->length = status & 0xffffff;	/* DW2[23:0] Transfer Length */
	out->code = (status >> 24) & 0xff;	/* DW2[31:24] Completion Code */

	/* 事件里也带 Slot ID / Endpoint ID，可用来确认事件属于哪个设备端点 */
	(void)((control >> 24) & 0xff);		/* Slot ID */
	(void)((control >> 16) & 0x1f);		/* Endpoint ID (DCI) */
}

//
// 调用点示意（不是完整函数）：
//
//   struct demo_ctrl_req req = {
//           .setup = {
//                   .bRequestType = USB_DIR_IN | USB_TYPE_STANDARD |
//                                   USB_RECIP_DEVICE,
//                   .bRequest = USB_REQ_GET_DESCRIPTOR,
//                   .wValue = cpu_to_le16(USB_DT_DEVICE << 8),
//                   .wIndex = 0,
//                   .wLength = cpu_to_le16(18),
//           },
//           .data_dma = desc_dma,       // dma_alloc_coherent 得到
//           .data_len = 18,
//           .data_in  = true,
//   };
//   demo_queue_control_td(xhci, &ep0_ring, slot_id, 1 /* EP0 的 DCI */,
//                         &req);
//
// 完成不在这里同步等待：硬件把 Transfer Event 写到事件环并产生中断，
// 驱动的中断处理程序解码后唤醒等待者（Linux 里是 urb->complete()）。
// 这正是“提交一次、完成一次”在 link 层看到的样子。
//

//
// EHCI 对照：EHCI 的 qTD（Queue Element Transfer Descriptor）把 IOC 放在
// qTD Token DWORD 的 bit 15（Linux ehci.h: #define QTD_IOC (1 << 15)）。
// EHCI 的控制传输本来就是多个 qTD（setup、data、status 各是独立 qTD），
// 每个 qTD 有各自的 IOC 和 Active 位，完成/中断的粒度天然是“每个 qTD”。
// xHCI 把三个阶段做成同一个 TD 里的三个 TRB，IOC 变成每个 TRB 的位，
// 驱动可以自己选择在哪个 TRB 上要事件；Linux 选择只在 Status Stage 设，
// 于是上层看到的仍然是“一次控制传输 = 一次完成”。
//
// 常见坑：
//   1. TRT 值写错：1 是保留值，OUT=2、IN=3（不是 1/2）。
//   2. Status 的 DIR 忘了取反，硬件报 TRB Error 或直接挂起。
//   3. Setup 的 Parameter 写成了 DMA 指针而不是 8 字节内嵌数据（IDT=1）。
//   4. 忘记翻转 Cycle 位或漏写 Link TRB，环回绕后硬件不再取 TRB。
//   5. 在 Data Stage 也设 IOC，结果一次传输收到两个事件，第二个事件
//      对应的 TD 状态没准备好，容易 double free / URB 重复完成。
//   6. 写门铃前没有 wmb()，弱序架构上硬件可能看到旧 TRB。
//
