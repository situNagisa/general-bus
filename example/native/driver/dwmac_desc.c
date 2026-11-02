/* 不在本机编译：Linux 内核驱动代码（Synopsys DWMAC4 / stmmac 描述符与寄存器），需要对目标内核编译并在带 GMAC 的板子上加载。
* （Linux 内核只能用 C 编译，所以本文件保持 .c，不改成 .cpp。）
*
* 这是驱动实现 link 的手段，link 层看不到它。
* link 层只看到“一帧以太网数据从 A 到 B”；描述符环、OWN 位、首段/末段、
* 尾指针、两块缓冲怎么拼成一帧，都是网卡驱动和 MAC/DMA 硬件之间的事。
*
* ============================================================================
* 这个接口是什么
* ============================================================================
* Synopsys DesignWare MAC（DWMAC，Linux 里叫 stmmac 驱动）是以太网控制器
* 的常见 IP，STM32MP1、很多 SoC 的 GMAC 都是它。GMAC4.x（DWMAC4）用一组
* 4 字（16 字节）的“普通描述符”组成发送/接收环：
*
*   发送（TDES）                        接收（RDES）
*   TDES0  buf1 地址 (32 位)            RDES0  buf1 地址 (32 位)
*   TDES1  buf2 地址 (32 位)            RDES1  buf2 地址 (32 位)
*   TDES2  [13:0] buf1 长度             RDES2  读格式保留
*          [29:16] buf2 长度            RDES3  [24] BUF1V  [25] BUF2V
*          [31] IC 完成中断                    [30] IOC   [31] OWN
*   TDES3  [31] OWN  [29] FD  [28] LD
*          [14:0] 整帧长度（LD 描述符）
*
* 一个描述符可以带两块缓冲：buf1 放协议头（以太网头 + IP/TCP 头），
* buf2 放载荷。这不是硬件强制，而是驱动常用的“头部分离”手法：头部通常
* 由协议栈现成拼好，载荷可能是上层的大块内存，两块分开挂就不用 memcpy
* 拼成一整块。DWMAC 的 TSO 路径也依赖两块缓冲（buf1 头、buf2 数据）。
*
* 什么时候用：写网卡驱动、或读 stmmac 源码时理解 DMA 环如何工作。
*
* ============================================================================
* 形态
* ============================================================================
*   - 一帧 = 一个或多个描述符。只有一个描述符时 FD=LD=1；分片时第一个
*     描述符 FD=1、最后一个 LD=1，硬件据此知道帧的边界。
*   - 一次提交 = 把描述符的 OWN 置 1，然后推进“尾指针”寄存器，告诉 DMA
*     “有效描述符到这里为止”。尾指针的值是“最后一个有效描述符 + 1”的
*     地址（下一个空位），硬件只处理 tail 之前的描述符。
*   - 完成 = 硬件把 OWN 清 0，并在 LD 描述符上写回状态（错误位、帧长）。
*     描述符置了 IC/IOC 的那一个才会产生中断；通常只给每帧最后一个描述符
*     置 IC，否则一帧多描述符就会中断风暴。
*   - 内存：描述符环保留在 dma_alloc_coherent() 里（硬件和 CPU 都要看，
*     且要长期稳定）；数据缓冲用 dma_map_single() 映射（本示例）或 coherent
*     分配（RX 环常用）。
*
* ============================================================================
* 需要的条件
* ============================================================================
*   - Synopsys DWMAC4 / GMAC4.x 硬件，DMA 使用 32 位地址（本示例）；
*   - 寄存器基址由 platform resource 给出，DMA 通道寄存器从基址 + 0x1100
*     开始（通道 0）；
*   - 真实驱动还要配 MAC 的速率/双工、PHY、DMA 中断等，这里只展示 DMA 环。
*
* 未核对：DWMAC4 在 64 位 DMA 地址模式下改用 8 字（32 字节）增强描述符，
* 地址占 TDES0..TDES3，长度/控制位整体后移；具体字段位置本机无法核对，
* 本示例不涉及，只写 32 位地址的 4 字普通描述符。
*
* ============================================================================
* 怎么编译、怎么运行、预期输出
* ============================================================================
*   编译：把文件放到内核树里（例如 drivers/net/ethernet/ibus/），
*         make -C /lib/modules/$(uname -r)/build M=$PWD modules
*   运行：需要一块 GMAC4 板子，并把 compatible 改成实际平台；insmod 后
*         dmesg 应看到：
*           ibus-dwmac-demo ...: 描述符 16 字节, TX 环 64 项 @0x...
*           ibus-dwmac-demo ...: RX 环已装填, 尾指针 0x...
*   本示例的 probe 只初始化环、打印地址，不真的发帧（避免往别人的网络上
*   发垃圾包）。ibus_dwmac4_xmit() 就是 ndo_start_xmit 里该做的事。
*
* 字段来源：drivers/net/ethernet/stmicro/stmmac/dwmac4_descs.h（上游 v6.18）
* 与 stmmac 的 dwmac4_descs.c，已按源码核对宏名和位段。
*/

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/dma-mapping.h>
#include <linux/slab.h>
#include <linux/bitops.h>
#include <linux/build_bug.h>
#include <linux/err.h>

/* ---------------------------------------------------------------------------
* DWMAC4 普通描述符（32 位地址）的控制/状态位
* 数值与 dwmac4_descs.h 一致
* ---------------------------------------------------------------------------
*/
/* TDES2（读格式）：两块缓冲的长度，以及“完成时中断” */
#define TDES2_BUFFER1_SIZE_MASK		GENMASK(13, 0)	/* buf1 长度，最大 16383 */
#define TDES2_BUFFER2_SIZE_MASK		GENMASK(29, 16)	/* buf2 长度，最大 16383 */
#define TDES2_BUFFER2_SIZE_MASK_SHIFT	16
#define TDES2_INTERRUPT_ON_COMPLETION	BIT(31)		/* IC：该描述符完成时中断 */

/* TDES3：读格式里的控制位，写回格式里的状态位 */
#define TDES3_OWN			BIT(31)		/* 1=硬件拥有，0=CPU 拥有 */
#define TDES3_FIRST_DESCRIPTOR		BIT(29)		/* FD：帧的第一段 */
#define TDES3_LAST_DESCRIPTOR		BIT(28)		/* LD：帧的最后一段 */
#define TDES3_PACKET_SIZE_MASK		GENMASK(14, 0)	/* 整帧字节数，写在 LD 上 */
#define TDES3_ERROR_SUMMARY		BIT(15)		/* 写回：该帧有错 */

/* RDES3（读格式，CPU 填给硬件） */
#define RDES3_OWN			BIT(31)
#define RDES3_BUFFER1_VALID_ADDR	BIT(24)		/* BUF1V：buf1 地址有效 */
#define RDES3_BUFFER2_VALID_ADDR	BIT(25)		/* BUF2V：buf2 地址有效 */
#define RDES3_INT_ON_COMPLETION_EN	BIT(30)		/* IOC：完成时中断 */

/* RDES3（写回格式，硬件填给 CPU） */
#define RDES3_PACKET_SIZE_MASK		GENMASK(14, 0)	/* 本帧长度 */
#define RDES3_ERROR_SUMMARY		BIT(15)
#define RDES3_FIRST_DESCRIPTOR		BIT(29)
#define RDES3_LAST_DESCRIPTOR		BIT(28)
#define RDES3_RDES0_VALID		BIT(25)		/* 写回时 RDES0 里的状态有效 */
#define RDES3_RDES1_VALID		BIT(26)
#define RDES3_RDES2_VALID		BIT(27)

/* ---------------------------------------------------------------------------
* DMA 通道寄存器（通道 0 基址 0x1100，每通道间隔 0x80）
* 这些名字与 dwmac4_dma.h 一致。
* ---------------------------------------------------------------------------
*/
#define DMA_CHAN_BASE_ADDR		0x00001100
#define DMA_CHAN_BASE_OFFSET		0x80
#define DMA_CHANX_BASE_ADDR(x)		(DMA_CHAN_BASE_ADDR + ((x) * DMA_CHAN_BASE_OFFSET))
#define DMA_CHAN_TX_CONTROL(x)		(DMA_CHANX_BASE_ADDR(x) + 0x04)
#define DMA_CHAN_RX_CONTROL(x)		(DMA_CHANX_BASE_ADDR(x) + 0x08)
#define DMA_CHAN_TX_BASE_ADDR_HI(x)	(DMA_CHANX_BASE_ADDR(x) + 0x10)
#define DMA_CHAN_TX_BASE_ADDR(x)	(DMA_CHANX_BASE_ADDR(x) + 0x14)
#define DMA_CHAN_RX_BASE_ADDR_HI(x)	(DMA_CHANX_BASE_ADDR(x) + 0x18)
#define DMA_CHAN_RX_BASE_ADDR(x)	(DMA_CHANX_BASE_ADDR(x) + 0x1c)
#define DMA_CHAN_TX_END_ADDR(x)		(DMA_CHANX_BASE_ADDR(x) + 0x20)
#define DMA_CHAN_RX_END_ADDR(x)		(DMA_CHANX_BASE_ADDR(x) + 0x28)
#define DMA_CHAN_TX_RING_LEN(x)		(DMA_CHANX_BASE_ADDR(x) + 0x2c)
#define DMA_CHAN_RX_RING_LEN(x)		(DMA_CHANX_BASE_ADDR(x) + 0x30)
/* 发送/接收控制寄存器的位 */
#define DMA_CONTROL_ST			BIT(0)		/* TX start */
#define DMA_CONTROL_SR			BIT(0)		/* RX start */
#define DMA_RBSZ_MASK			GENMASK(14, 1)	/* RX 每块缓冲字节数 */
#define DMA_RBSZ_SHIFT			1

/* 说明：TX_END_ADDR 就是数据手册里的 DMA_CHx_TxDesc_Tail_Pointer（通道 0
* 地址 0x1120）。stmmac 头文件沿用 Synopsys 的旧叫法“end address”；它是
* “尾指针”，不是环的结束地址。RX 的尾指针在 0x1128。
*/

#define IBUS_TX_RING_LEN	64	/* 必须是 2 的幂，方便取模 */
#define IBUS_RX_RING_LEN	64
#define IBUS_RX_BUF_SIZE	2048

/* 一个 DWMAC4 普通描述符就是 4 个 32 位字，16 字节 */
struct ibus_dwmac4_desc {
	__le32 des0;
	__le32 des1;
	__le32 des2;
	__le32 des3;
};
static_assert(sizeof(struct ibus_dwmac4_desc) == 16, "DWMAC4 normal descriptor must be 16 bytes");

/* 发送时用来在完成回收阶段 unmap 的簿记 */
struct ibus_tx_meta {
	void *hdr;
	dma_addr_t hdr_dma;
	size_t hdr_len;
	void *payload;
	dma_addr_t payload_dma;
	size_t payload_len;
	int mapped;
};

/* 接收缓冲：本示例用 coherent 内存，简单起见不搞页面回收 */
struct ibus_rx_meta {
	void *buf;
	dma_addr_t dma;
};

struct ibus_dwmac4 {
	struct device *dev;
	void __iomem *ioaddr;		/* MAC 寄存器基址（DMA 寄存器在 +0x1100） */
	struct ibus_dwmac4_desc *tx_ring;
	dma_addr_t tx_ring_dma;
	struct ibus_tx_meta *tx_meta;
	unsigned int tx_count;
	unsigned int tx_head;		/* 下一个要写的描述符 */
	unsigned int tx_tail;		/* 最老的、还没回收的描述符 */

	struct ibus_dwmac4_desc *rx_ring;
	dma_addr_t rx_ring_dma;
	struct ibus_rx_meta *rx_meta;
	unsigned int rx_count;
	unsigned int rx_clean;		/* 下一个要检查的描述符 */
};

/* 把环形下标加一。环长是 2 的幂，用掩码比 % 便宜。 */
static inline unsigned int ibus_ring_next(unsigned int i, unsigned int count)
{
	return (i + 1) & (count - 1);
}

/* ---------------------------------------------------------------------------
* 初始化发送环
* ---------------------------------------------------------------------------
*/
static int ibus_dwmac4_tx_init(struct ibus_dwmac4 *priv)
{
	size_t bytes = IBUS_TX_RING_LEN * sizeof(struct ibus_dwmac4_desc);

	/* 描述符环必须让硬件长期看到，且 CPU 会频繁读写，用 coherent 分配。
	* 它同时返回 CPU 虚拟地址和 DMA 地址（这里是物理地址）。
	*/
	priv->tx_ring = dma_alloc_coherent(priv->dev, bytes,
						&priv->tx_ring_dma, GFP_KERNEL);
	if (!priv->tx_ring)
		return -ENOMEM;
	/* 全 0 = 所有描述符 OWN=0，归 CPU 所有 */
	memset(priv->tx_ring, 0, bytes);

	priv->tx_meta = devm_kcalloc(priv->dev, IBUS_TX_RING_LEN,
					sizeof(*priv->tx_meta), GFP_KERNEL);
	if (!priv->tx_meta)
		return -ENOMEM;

	priv->tx_count = IBUS_TX_RING_LEN;
	priv->tx_head = 0;
	priv->tx_tail = 0;

	/* 告诉 DMA：环从哪开始（低 32 位 + 高 32 位）、有多少字节。
	* RING_LEN 的单位是字节，不是描述符个数，且必须是描述符大小的整数倍。
	*/
	writel(lower_32_bits(priv->tx_ring_dma),
		priv->ioaddr + DMA_CHAN_TX_BASE_ADDR(0));
	writel(upper_32_bits(priv->tx_ring_dma),
		priv->ioaddr + DMA_CHAN_TX_BASE_ADDR_HI(0));
	writel(bytes, priv->ioaddr + DMA_CHAN_TX_RING_LEN(0));

	/* ST=1：启动发送 DMA。此时环里还没有有效描述符，硬件会停在环首
	* 等尾指针推进。
	*/
	writel(DMA_CONTROL_ST, priv->ioaddr + DMA_CHAN_TX_CONTROL(0));

	dev_info(priv->dev, "描述符 %zu 字节, TX 环 %u 项 @%pad\n",
			sizeof(struct ibus_dwmac4_desc), priv->tx_count,
			&priv->tx_ring_dma);
	return 0;
}

/* ---------------------------------------------------------------------------
* 发送一帧：buf1 = 协议头，buf2 = 载荷
* ---------------------------------------------------------------------------
* 返回 0 表示已交给硬件；-EBUSY 表示环满；-EIO 表示映射失败。
* 真实驱动在 ndo_start_xmit 里调用它，返回值决定是否 stop 队列。
*/
static int ibus_dwmac4_xmit(struct ibus_dwmac4 *priv,
				const void *hdr, size_t hdr_len,
				const void *payload, size_t payload_len)
{
	struct ibus_dwmac4_desc *p;
	struct ibus_tx_meta *meta;
	dma_addr_t hdr_dma, payload_dma;
	unsigned int next;
	size_t total = hdr_len + payload_len;
	u32 tdes3;

	if (!hdr_len || !payload_len ||
		hdr_len > TDES2_BUFFER1_SIZE_MASK ||
		payload_len > TDES2_BUFFER2_SIZE_MASK)
		return -EINVAL;

	/* 环满判断：留一个空位区分“满”和“空”。tx_head 追上 tx_tail
	* 说明还有一圈没回收，不能再写。
	*/
	next = ibus_ring_next(priv->tx_head, priv->tx_count);
	if (next == priv->tx_tail)
		return -EBUSY;

	p = &priv->tx_ring[priv->tx_head];
	meta = &priv->tx_meta[priv->tx_head];

	/* 数据缓冲由协议栈提供，映射成设备地址。DMA_TO_DEVICE 表示
	* 数据流向设备，映射时会做 cache clean（如有 D-Cache）。
	*/
	hdr_dma = dma_map_single(priv->dev, (void *)hdr, hdr_len, DMA_TO_DEVICE);
	if (dma_mapping_error(priv->dev, hdr_dma))
		return -EIO;
	payload_dma = dma_map_single(priv->dev, (void *)payload, payload_len,
					DMA_TO_DEVICE);
	if (dma_mapping_error(priv->dev, payload_dma)) {
		dma_unmap_single(priv->dev, hdr_dma, hdr_len, DMA_TO_DEVICE);
		return -EIO;
	}

	meta->hdr = (void *)hdr;
	meta->hdr_dma = hdr_dma;
	meta->hdr_len = hdr_len;
	meta->payload = (void *)payload;
	meta->payload_dma = payload_dma;
	meta->payload_len = payload_len;
	meta->mapped = 1;

	/* 两块缓冲的地址各占一个 32 位字。地址必须按总线宽度对齐；
	* 网卡头一般 2 字节对齐即可，但 DMA 地址最好 4 字节对齐。
	*/
	p->des0 = cpu_to_le32(lower_32_bits(hdr_dma));
	p->des1 = cpu_to_le32(lower_32_bits(payload_dma));

	/* TDES2：两个长度 + IC。IC 只在“这一帧最后一个描述符”上置位；
	* 本示例一帧只有一个描述符，所以这里置。
	*/
	p->des2 = cpu_to_le32((hdr_len & TDES2_BUFFER1_SIZE_MASK) |
				((payload_len << TDES2_BUFFER2_SIZE_MASK_SHIFT) &
				TDES2_BUFFER2_SIZE_MASK) |
				TDES2_INTERRUPT_ON_COMPLETION);

	/* TDES3：FD 和 LD 同时置位 = 这一帧只有这一段；总长度写在 LD
	* 描述符上，硬件用它判断帧尾（不够会自动补 CRC，具体看 MAC 配置）。
	*/
	tdes3 = TDES3_FIRST_DESCRIPTOR | TDES3_LAST_DESCRIPTOR |
		(total & TDES3_PACKET_SIZE_MASK);
	p->des3 = cpu_to_le32(tdes3);

	/* 关键顺序：地址、长度、状态都写完之后再置 OWN。
	* dma_wmb() 保证 CPU 的写顺序对 DMA 可见；否则硬件可能先看到
	* OWN=1，再去读还没写好的地址，搬错内存。
	*/
	dma_wmb();
	p->des3 |= cpu_to_le32(TDES3_OWN);

	priv->tx_head = next;

	/* 推进尾指针：写“最后一个有效描述符 + 1”的地址，也就是下一个空位。
	* 数据手册明确要求 tail 指向 last available descriptor + 1，硬件只
	* 处理 tail 之前的描述符（这点 stmmac 在 2018 年专门修过：
	* commit 0431100b3d82 "net: stmmac: Fixup the tail addr setting"）。
	* 写成 head 的地址后，硬件从旧 tail 走到新 tail 之前，正好包含刚写
	* 的那一个描述符。写 tail-1 会让最后一个描述符永远不被处理。
	*/
	writel(lower_32_bits(priv->tx_ring_dma +
				priv->tx_head * sizeof(*p)),
		priv->ioaddr + DMA_CHAN_TX_END_ADDR(0));

	return 0;
}

/* ---------------------------------------------------------------------------
* 发送完成回收
* ---------------------------------------------------------------------------
* 中断里（或 NAPI poll 里）调用。OWN=0 表示硬件已经把这个描述符还给我们。
*/
static void ibus_dwmac4_tx_clean(struct ibus_dwmac4 *priv)
{
	while (priv->tx_tail != priv->tx_head) {
		struct ibus_dwmac4_desc *p = &priv->tx_ring[priv->tx_tail];
		struct ibus_tx_meta *meta = &priv->tx_meta[priv->tx_tail];
		u32 des3 = le32_to_cpu(READ_ONCE(p->des3));

		/* OWN 还是 1：硬件还没搬完，后面的描述符更不可能完成 */
		if (des3 & TDES3_OWN)
			break;

		/* LD 描述符上才有完整的帧状态；中间的描述符只置自己的位。
		* ERROR_SUMMARY 是各种错误（underflow、carrier、collision…）
		* 的汇总位，具体原因在 TDES3 其它位里。
		*/
		if ((des3 & TDES3_LAST_DESCRIPTOR) &&
			(des3 & TDES3_ERROR_SUMMARY))
			dev_warn_ratelimited(priv->dev, "TX 帧出错, TDES3=0x%08x\n",
						des3);

		dma_unmap_single(priv->dev, meta->hdr_dma, meta->hdr_len,
					DMA_TO_DEVICE);
		dma_unmap_single(priv->dev, meta->payload_dma, meta->payload_len,
					DMA_TO_DEVICE);

		/* 清掉写回状态，OWN 保持 0，描述符可以再次使用 */
		p->des3 = 0;
		meta->mapped = 0;

		priv->tx_tail = ibus_ring_next(priv->tx_tail, priv->tx_count);
	}
}

/* ---------------------------------------------------------------------------
* 初始化接收环
* ---------------------------------------------------------------------------
* 每个描述符给一块 2KB 缓冲（BUF1V=1，不用 buf2）。硬件收到帧后：
*   - OWN 清 0；
*   - LD 置 1（本示例一帧一个描述符）；
*   - RDES3[14:0] 写本帧长度；
*   - 有错时 RDES3[15] 置 1。
*/
static int ibus_dwmac4_rx_init(struct ibus_dwmac4 *priv)
{
	size_t bytes = IBUS_RX_RING_LEN * sizeof(struct ibus_dwmac4_desc);
	unsigned int i;

	priv->rx_ring = dma_alloc_coherent(priv->dev, bytes,
						&priv->rx_ring_dma, GFP_KERNEL);
	if (!priv->rx_ring)
		return -ENOMEM;
	memset(priv->rx_ring, 0, bytes);

	priv->rx_meta = devm_kcalloc(priv->dev, IBUS_RX_RING_LEN,
					sizeof(*priv->rx_meta), GFP_KERNEL);
	if (!priv->rx_meta)
		return -ENOMEM;

	priv->rx_count = IBUS_RX_RING_LEN;
	priv->rx_clean = 0;

	for (i = 0; i < priv->rx_count; i++) {
		struct ibus_dwmac4_desc *p = &priv->rx_ring[i];
		void *buf;
		dma_addr_t dma;

		buf = dma_alloc_coherent(priv->dev, IBUS_RX_BUF_SIZE, &dma,
						GFP_KERNEL);
		if (!buf)
			return -ENOMEM;
		priv->rx_meta[i].buf = buf;
		priv->rx_meta[i].dma = dma;

		p->des0 = cpu_to_le32(lower_32_bits(dma));
		p->des1 = 0;	/* 不用第二块缓冲 */
		p->des2 = 0;
		/* OWN=1：交给硬件；BUF1V 告诉硬件 buf1 地址有效；
		* IOC：这个描述符完成时中断（一帧一个描述符，正好一帧一中断）。
		*/
		p->des3 = cpu_to_le32(RDES3_OWN | RDES3_BUFFER1_VALID_ADDR |
					RDES3_INT_ON_COMPLETION_EN);
	}

	writel(lower_32_bits(priv->rx_ring_dma),
		priv->ioaddr + DMA_CHAN_RX_BASE_ADDR(0));
	writel(upper_32_bits(priv->rx_ring_dma),
		priv->ioaddr + DMA_CHAN_RX_BASE_ADDR_HI(0));
	writel(bytes, priv->ioaddr + DMA_CHAN_RX_RING_LEN(0));

	/* RX 尾指针同样指向“最后一个有效描述符 + 1”。整圈都装填完毕后，
	* 最后一个有效描述符是 count-1，tail 就回绕到环首（基地址）。
	*/
	writel(lower_32_bits(priv->rx_ring_dma),
		priv->ioaddr + DMA_CHAN_RX_END_ADDR(0));

	/* RX 控制：RBSZ 是每块缓冲的字节数（写在 [14:1]），SR=1 启动 */
	writel(((IBUS_RX_BUF_SIZE << DMA_RBSZ_SHIFT) & DMA_RBSZ_MASK) |
		DMA_CONTROL_SR,
		priv->ioaddr + DMA_CHAN_RX_CONTROL(0));

	dev_info(priv->dev, "RX 环已装填, 尾指针 0x%llx\n",
			(unsigned long long)priv->rx_ring_dma);
	return 0;
}

/* ---------------------------------------------------------------------------
* 接收回收
* ---------------------------------------------------------------------------
* 真正的驱动会把数据交给协议栈（napi_alloc_skb + skb_put_data 或 page frag），
* 这里只演示描述符状态的读法，不建 skb。
*/
static int ibus_dwmac4_rx_poll(struct ibus_dwmac4 *priv, int budget)
{
	int done = 0;

	while (done < budget) {
		struct ibus_dwmac4_desc *p = &priv->rx_ring[priv->rx_clean];
		u32 des3 = le32_to_cpu(READ_ONCE(p->des3));
		u32 len;

		/* OWN=1 说明硬件还没写这个描述符，后面的描述符更没写 */
		if (des3 & RDES3_OWN)
			break;

		/* 读数据之前要确保描述符和缓冲的写已经对 CPU 可见。
		* coherent 内存本身不经过 cache，但描述符状态的读仍应先于
		* 数据读；dma_rmb() 给出顺序。
		*/
		dma_rmb();

		if (!(des3 & RDES3_LAST_DESCRIPTOR))
			dev_warn(priv->dev, "RX 多描述符帧，本示例未处理\n");

		len = des3 & RDES3_PACKET_SIZE_MASK;

		if (des3 & RDES3_ERROR_SUMMARY)
			dev_warn_ratelimited(priv->dev, "RX 帧出错, RDES3=0x%08x\n",
						des3);
		else
			dev_info_ratelimited(priv->dev, "RX %u 字节\n", len);

		/* 重新装填：把 OWN 重新置 1，地址不变（本示例不复制数据，
		* 真实驱动会在这里把 buf 交给协议栈再换新缓冲）。
		* 写回状态位（LD/长度/错误）会被这次写入覆盖清掉。
		*/
		p->des3 = cpu_to_le32(RDES3_OWN | RDES3_BUFFER1_VALID_ADDR |
					RDES3_INT_ON_COMPLETION_EN);

		priv->rx_clean = ibus_ring_next(priv->rx_clean, priv->rx_count);
		done++;
	}

	/* 尾指针推进到“最后一个重新装填的描述符 + 1”，也就是 rx_clean
	* 指向的下标；硬件处理到它之前为止。
	*/
	if (done) {
		unsigned int next = priv->rx_clean % priv->rx_count;

		wmb();
		writel(lower_32_bits(priv->rx_ring_dma +
					next * sizeof(struct ibus_dwmac4_desc)),
			priv->ioaddr + DMA_CHAN_RX_END_ADDR(0));
	}

	return done;
}

static int ibus_dwmac4_probe(struct platform_device *pdev)
{
	struct ibus_dwmac4 *priv;
	int ret;

	priv = devm_kzalloc(&pdev->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;
	priv->dev = &pdev->dev;

	priv->ioaddr = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(priv->ioaddr))
		return PTR_ERR(priv->ioaddr);

	/* 本示例按 32 位 DMA 地址写描述符；如果平台只能给 64 位地址，
	* 描述符格式要换成 8 字增强描述符（未在示例范围内）。
	*/
	ret = dma_set_mask_and_coherent(priv->dev, DMA_BIT_MASK(32));
	if (ret)
		return ret;

	ret = ibus_dwmac4_tx_init(priv);
	if (ret)
		return ret;

	ret = ibus_dwmac4_rx_init(priv);
	if (ret)
		return ret;

	platform_set_drvdata(pdev, priv);

	/* 真实驱动在这里注册 netdev，之后 ndo_start_xmit 调
	* ibus_dwmac4_xmit()、中断处理里调 tx_clean / rx_poll。
	* 示例不发帧，避免在别人的网络上产生垃圾包。
	*/
	return 0;
}

static struct platform_driver ibus_dwmac4_driver = {
	.probe = ibus_dwmac4_probe,
	.driver = {
		.name = "ibus-dwmac4-demo",
	},
};
module_platform_driver(ibus_dwmac4_driver);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("DWMAC4 normal descriptor ring demo (TX/RX, two buffers per descriptor)");
