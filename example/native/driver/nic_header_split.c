/* 不在本机编译：Linux 内核驱动内部 API（Intel ixgbe 网卡描述符与 SRRCTL 寄存器），需要目标内核的 ixgbe 源码树和 82599 级硬件。
* （Linux 内核只能用 C 编译，所以本文件保持 .c，不改成 .cpp。）
*
* 这是驱动实现 link 的手段，link 层看不到它。
* link 层只看到“收到一帧、交给上层”；网卡按协议解析后把头部和载荷
* 分别 DMA 到两块缓冲、描述符里两个地址怎么写、DD/SPH 位怎么读，
* 全是 ixgbe 驱动内部的事。
*
* ============================================================================
* 这个接口是什么
* ============================================================================
* header split（头部分离，Intel 文档也叫 packet split 的一部分）是网卡接收
* 路径的一种硬件加速：网卡自己解析以太网头、IP 头、TCP/UDP 头，把
*   - L2/L3/L4 协议头（通常几十到一百多字节）DMA 到一块小缓冲 hdr_addr；
*   - 剩下的载荷 DMA 到一块大缓冲 pkt_addr。
* 一个接收描述符因此带两个缓冲地址。驱动拿到描述符后，头部已经在固定的小
* 缓冲里，可以直接读/查表/建 skb 的线性区；载荷留在页里，整页挂成 skb frag，
* 不用 memcpy。
*
* 什么时候用：需要逐包解析头部、又希望载荷零拷贝的网络栈；或者希望头部
* 集中在小缓冲里，提高 cache 命中率。普通网卡驱动可以不启用，全部数据都
* 放 pkt_addr 一块缓冲里。
*
* ============================================================================
* 形态
* ============================================================================
*   - 一个接收描述符 = 两块接收缓冲（hdr_addr 小、pkt_addr 大）。
*   - 一个以太网帧通常 = 一个描述符（一帧跨多个描述符时用 EOP 判断帧尾）。
*   - 分界点由网卡硬件按协议解析决定，驱动不能逐包指定“切在第几字节”。
*     驱动只能通过 SRRCTL.DESCTYPE 选择策略（按协议头切 / 总是切 / 不切）
*     和 SRRCTL.BSIZEHDR 给出头缓冲大小。
*   - 提交：驱动把两个缓冲地址写进描述符（地址的 bit0 是 DD 位，必须为 0），
*     然后推进 RDT 尾指针，把描述符交还硬件。
*   - 完成：硬件把数据 DMA 完，置 DD=1（写回 read.pkt_addr 的 bit0 位置），
*     若发生头部分离则置 SPH=1，并在 hdr_info 里写头部长度，在
*     upper.length 里写整帧长度（含头部）。IOC/中断由队列配置决定。
*   - 内存：描述符环 coherent 分配；头缓冲可以是 coherent 小缓冲；载荷缓冲
*     用页 + dma_map_page，完成后整页交给 skb。
*
* ============================================================================
* ixgbe advanced RX descriptor（union ixgbe_adv_rx_desc）关键字段
* ============================================================================
*   read.pkt_addr   u64  载荷缓冲 DMA 地址；bit0 同时是 DD 位
*   read.hdr_addr   u64  头部缓冲 DMA 地址（启用头部分离时用）
*
*   写回格式：
*   wb.upper.status_error  u32
*        bit0  DD   描述符完成（IXGBE_RXD_STAT_DD = 0x01）
*        bit1  EOP  帧结束（IXGBE_RXD_STAT_EOP = 0x02）
*        bit23 ERR_HBO 头部缓冲溢出（IXGBE_RXDADV_ERR_HBO = 0x00800000）
*        bit31:24 各种 CRC/长度/IP/TCP 校验错误（IXGBE_RXDADV_ERR_*）
*   wb.upper.length        u16  整帧长度（含头部）
*   wb.lower.hs_rss.hdr_info u16
*        bit15 SPH（IXGBE_RXDADV_SPH = 0x8000）：本包做了头部分离
*        bit14:5 头部长度（IXGBE_RXDADV_HDRBUFLEN_MASK = 0x7FE0，
*                IXGBE_RXDADV_HDRBUFLEN_SHIFT = 5）
*        bit4:0 头部类型/标志（具体编码未核对）
*   wb.lower.hs_rss.pkt_info u16  RSS 类型/包类型
*
* 因此：头部长度 = (hdr_info & 0x7FE0) >> 5；载荷长度 = length - 头部长度。
* 没有 SPH 时整帧都在 pkt 缓冲里，头部长度按 0 处理。
*
* 以上宏名和数值按上游 v6.18 的 ixgbevf/defines.h（与 ixgbe_type.h 同一
* 套定义）核对；位段位置是按宏值与字段布局推出来的，未用硬件实测。
*
* ============================================================================
* 需要的条件
* ============================================================================
*   - Intel 82599/ixgbe 系列网卡，队列启用了 header split：
*       SRRCTL.DESCTYPE = HDR_SPLIT，SRRCTL.BSIZEHDR 给出头缓冲大小；
*   - 描述符是 advanced descriptor（ixgbe 的默认接收描述符）；
*   - 本文件是 ixgbe 驱动内的片段，应放在 drivers/net/ethernet/intel/ixgbe/
*     下，包含 "ixgbe.h"（它带入 ixgbe_type.h 的 union 和 IXGBE_* 宏）。
*
* 怎么编译、怎么运行：作为 ixgbe 驱动的一部分编译进内核/模块，在 82599
* 网卡上收包；dmesg 里每个包会打印“头 x 字节 + 载荷 y 字节”。
* 预期输出示例：
*   ibus-ixgbe-demo: 收到 1 帧: 头 54 字节 + 载荷 46 字节 (SPH=1)
*
* 未核对：RDT 的精确语义（“指向最后一个有效描述符”还是“指向它的下一个”）
* 本机没有 82599 手册，按 ixgbe 源码惯例写成 next_to_use；SRRCTL.BSIZEHDR
* 的字段位置/掩码（0x3F00，bits13:8，64 字节单位）与 BSIZEPKT（单位 KB，
* shift 10）已按 ixgbevf/defines.h 核对。
*/

#include <linux/kernel.h>
#include <linux/dma-mapping.h>
#include <linux/skbuff.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/if_ether.h>
#include <linux/mm.h>
#include <linux/if_vlan.h>
#include "ixgbe.h"

#define IBUS_RX_DESC_COUNT	64
#define IBUS_HDR_SIZE		256	/* 头缓冲大小：够放 Ethernet+IP+TCP 选项 */
#define IBUS_PKT_SIZE		2048	/* 载荷缓冲大小（一个页内） */

struct ibus_ixgbe_rx_ring {
	struct device *dev;
	struct net_device *netdev;
	struct napi_struct *napi;
	void __iomem *hw_addr;		/* ixgbe 寄存器基址 */
	union ixgbe_adv_rx_desc *desc;	/* 描述符环（coherent） */
	dma_addr_t desc_dma;
	void *hdr_virt[IBUS_RX_DESC_COUNT];	/* 头缓冲（coherent，CPU 直接读） */
	dma_addr_t hdr_dma[IBUS_RX_DESC_COUNT];
	struct page *pkt_page[IBUS_RX_DESC_COUNT];	/* 载荷页 */
	dma_addr_t pkt_dma[IBUS_RX_DESC_COUNT];
	u16 count;
	u16 next_to_use;		/* 下一个要装填的描述符 */
	u16 next_to_clean;		/* 下一个要检查完成的描述符 */
	u64 hdr_pkts;			/* 统计：发生了头部分离的包数 */
};

/* ---------------------------------------------------------------------------
* 1. 配置队列：让硬件做头部分离
* ---------------------------------------------------------------------------
* SRRCTL（Split and Replication Receive Control）每个队列一个。
*   DESCTYPE 位段决定描述符类型：
*       ADV_ONEBUF        所有数据放 pkt_addr
*       HDR_SPLIT         按协议解析结果分离头部（本示例）
*       HDR_SPLIT_ALWAYS  总是按固定头长分离
*   BSIZEHDR 是头缓冲大小。
*/
static void ibus_ixgbe_config_split(struct ibus_ixgbe_rx_ring *ring)
{
	u32 srrctl;

	/* 按协议头切分：硬件解析到 L4 头结束（含选项）就切；解析不出来
	* 的包（非 IP、分片等）可能不分离，描述符里 SPH=0。
	*/
	srrctl = IXGBE_SRRCTL_DESCTYPE_HDR_SPLIT;

	/* 头缓冲大小：BSIZEHDR 字段在 SRRCTL[13:8]，单位 64 字节。
	* 256 字节 => 4 个单位 => 4 << 8。ixgbe 头文件里这个字段的掩码叫
	* IXGBE_SRRCTL_BSIZEHDR_MASK（0x00003F00，已核对）。
	*/
	srrctl |= ((IBUS_HDR_SIZE / 64) << 8) & IXGBE_SRRCTL_BSIZEHDR_MASK;

	/* 包缓冲大小：BSIZEPKT 字段，单位 KB（1KB 计 1）。
	* 2048 字节 => 2048 >> 10 = 2。宏值已核对。
	*/
	srrctl |= (IBUS_PKT_SIZE >> IXGBE_SRRCTL_BSIZEPKT_SHIFT) &
			IXGBE_SRRCTL_BSIZEPKT_MASK;

	writel(srrctl, ring->hw_addr + IXGBE_SRRCTL(0));
}

/* ---------------------------------------------------------------------------
* 2. 装填接收环：每个描述符两个地址
* ---------------------------------------------------------------------------
*/
static int ibus_ixgbe_alloc_rx_ring(struct ibus_ixgbe_rx_ring *ring)
{
	size_t bytes = IBUS_RX_DESC_COUNT * sizeof(union ixgbe_adv_rx_desc);
	unsigned int i;

	/* 描述符环：硬件写回状态、CPU 读，长期存在，用 coherent */
	ring->desc = dma_alloc_coherent(ring->dev, bytes, &ring->desc_dma,
					GFP_KERNEL);
	if (!ring->desc)
		return -ENOMEM;
	memset(ring->desc, 0, bytes);

	for (i = 0; i < IBUS_RX_DESC_COUNT; i++) {
		union ixgbe_adv_rx_desc *rx_desc = &ring->desc[i];
		dma_addr_t dma;

		/* 头缓冲：小、固定、CPU 要立刻读，用 coherent 最省事 */
		ring->hdr_virt[i] = dma_alloc_coherent(ring->dev, IBUS_HDR_SIZE,
							&dma, GFP_KERNEL);
		if (!ring->hdr_virt[i])
			return -ENOMEM;
		ring->hdr_dma[i] = dma;

		/* 载荷缓冲：一个页，DMA_FROM_DEVICE 映射，完成后整页给 skb */
		ring->pkt_page[i] = alloc_page(GFP_KERNEL);
		if (!ring->pkt_page[i])
			return -ENOMEM;
		dma = dma_map_page(ring->dev, ring->pkt_page[i], 0,
					IBUS_PKT_SIZE, DMA_FROM_DEVICE);
		if (dma_mapping_error(ring->dev, dma))
			return -ENOMEM;
		ring->pkt_dma[i] = dma;

		/* read 格式的两个地址。
		* 关键：地址的 bit0 是 DD 位，必须为 0（映射出来的地址
		* 至少 4 字节对齐，所以自然为 0）；写地址本身就是“清 DD、
		* 把描述符交回硬件”。
		*/
		rx_desc->read.pkt_addr = cpu_to_le64(ring->pkt_dma[i]);
		rx_desc->read.hdr_addr = cpu_to_le64(ring->hdr_dma[i]);
	}

	ring->count = IBUS_RX_DESC_COUNT;
	ring->next_to_clean = 0;
	ring->next_to_use = 0;

	/* 把整圈都交出去：RDT 填“最后一个有效描述符的下一个”下标。
	* ixgbe 的惯例是 RDT = next_to_use；装填完一圈后正好是 0（回绕）。
	* 具体编码以 82599 手册为准——本机未核对。
	*/
	wmb();
	writel(ring->next_to_use, ring->hw_addr + IXGBE_RDT(0));

	return 0;
}

/* 为一个描述符准备新的载荷页：先分配并映射，成功后才替换旧的。
* 这样分配失败时描述符原封不动，不会出现“已经消费、却没缓冲可装填”的
* 空洞（NAPI 里不能睡眠，只能用 GFP_ATOMIC）。
*/
static struct page *ibus_ixgbe_new_rx_page(struct ibus_ixgbe_rx_ring *ring,
						dma_addr_t *dma)
{
	struct page *page = alloc_page(GFP_ATOMIC);

	if (!page)
		return NULL;
	*dma = dma_map_page(ring->dev, page, 0, IBUS_PKT_SIZE,
				DMA_FROM_DEVICE);
	if (dma_mapping_error(ring->dev, *dma)) {
		__free_page(page);
		return NULL;
	}
	return page;
}

/* ---------------------------------------------------------------------------
* 3. 收包：读描述符，把“头 + 载荷”拼成一个 skb
* ---------------------------------------------------------------------------
* 返回处理的包数（NAPI 的 budget 语义）。
*/
static int ibus_ixgbe_clean_rx_irq(struct ibus_ixgbe_rx_ring *ring, int budget)
{
	struct device *dev = ring->dev;
	unsigned int done = 0;
	u16 ntc = ring->next_to_clean;
	bool refilled = false;

	while (done < budget) {
		union ixgbe_adv_rx_desc *rx_desc = &ring->desc[ntc];
		u32 staterr = le32_to_cpu(rx_desc->wb.upper.status_error);
		u16 hdr_info, pkt_len;
		u32 hdr_len, head_len, payload_len;
		struct page *page, *new_page;
		dma_addr_t new_dma;
		struct sk_buff *skb;
		bool dropped = false;

		/* DD=0：硬件还没写完这个描述符，后面的更没写 */
		if (!(staterr & IXGBE_RXD_STAT_DD))
			break;

		/* 读描述符之间的顺序：先确认 DD，再读长度/头部信息。
		* coherent 内存不需要 cache 维护，但需要编译器/CPU 不重排。
		*/
		dma_rmb();

		/* 先准备好替换用的页；失败就不碰这个描述符，下一轮再试 */
		new_page = ibus_ixgbe_new_rx_page(ring, &new_dma);
		if (!new_page) {
			dev_warn_ratelimited(dev, "分配接收页失败，暂停收包\n");
			break;
		}

		/* 整帧长度（含头部）。硬件保证在 DD=1 后有效。 */
		pkt_len = le16_to_cpu(rx_desc->wb.upper.length);

		/* hdr_info：SPH + 头部长度 + 头类型 */
		hdr_info = le16_to_cpu(rx_desc->wb.lower.hs_rss.hdr_info);

		if (hdr_info & IXGBE_RXDADV_SPH) {
			/* 硬件做了分离：头在 hdr 缓冲，载荷在 pkt 缓冲
			* 的起始处（偏移 0 开始）。
			*/
			hdr_len = (hdr_info & IXGBE_RXDADV_HDRBUFLEN_MASK) >>
					IXGBE_RXDADV_HDRBUFLEN_SHIFT;
			if (hdr_len > pkt_len) {
				/* 不可能发生；真发生说明描述符被写坏 */
				WARN_ON_ONCE(1);
				hdr_len = pkt_len;
			}
			head_len = hdr_len;
			payload_len = pkt_len - hdr_len;
			ring->hdr_pkts++;
		} else {
			/* 没有分离：整帧都在 pkt 缓冲。为了让协议栈能解析
			* 以太网头，把前 IBUS_HDR_SIZE 字节拷进 skb 线性区，
			* 剩下的仍然零拷贝挂在 frag 里。
			*/
			hdr_len = 0;
			head_len = min_t(u32, pkt_len, IBUS_HDR_SIZE);
			payload_len = pkt_len - head_len;
		}

		/* 头部缓冲溢出 / 帧错误 / 长度不合法：整包丢掉，
		* 但描述符仍然要重新装填。
		*/
		if (pkt_len < ETH_HLEN ||
			(staterr & (IXGBE_RXDADV_ERR_HBO | IXGBE_RXDADV_ERR_FRAME_ERR_MASK))) {
			dev_warn_ratelimited(dev, "RX 错误: staterr=0x%08x len=%u\n",
						staterr, pkt_len);
			dropped = true;
		}

		/* 卸载 DMA 映射，之后 CPU（和协议栈）才能读这一页 */
		page = ring->pkt_page[ntc];
		dma_unmap_page(dev, ring->pkt_dma[ntc], IBUS_PKT_SIZE,
				DMA_FROM_DEVICE);

		skb = NULL;
		if (!dropped) {
			skb = napi_alloc_skb(ring->napi, head_len);
			if (!skb) {
				dev_warn_ratelimited(dev, "napi_alloc_skb 失败\n");
				dropped = true;
			}
		}

		if (!dropped) {
			/* 线性区：头部。分离时来自 hdr 缓冲；没分离时
			* 是从 pkt 页头部拷过来的。page_address 只对低端内存
			* 有效；高端内存的机器要改成 kmap_local_page()。
			*/
			if (head_len) {
				const void *src = (hdr_info & IXGBE_RXDADV_SPH) ?
							ring->hdr_virt[ntc] :
							page_address(page);
				skb_put_data(skb, src, head_len);
			}

			/* 载荷：整页挂成 frag，不再拷贝。
			* 分离时数据从页偏移 0 开始；没分离时偏移 head_len。
			*/
			if (payload_len)
				skb_add_rx_frag(skb, 0, page,
						(hdr_info & IXGBE_RXDADV_SPH) ?
						0 : head_len,
						payload_len, IBUS_PKT_SIZE);
			else
				__free_page(page);	/* 空载荷，页还回去 */

			skb->protocol = eth_type_trans(skb, ring->netdev);
			napi_gro_receive(ring->napi, skb);
		} else {
			__free_page(page);
		}

		/* 换上新页：写回两个地址 = 清 DD 并把描述符交还硬件。
		* RDT 统一在循环末尾推进，避免硬件在描述符还没写好时就看到它。
		*/
		ring->pkt_page[ntc] = new_page;
		ring->pkt_dma[ntc] = new_dma;
		rx_desc->read.pkt_addr = cpu_to_le64(new_dma);
		rx_desc->read.hdr_addr = cpu_to_le64(ring->hdr_dma[ntc]);
		refilled = true;

		ntc++;
		if (ntc == ring->count)
			ntc = 0;
		done++;
	}

	ring->next_to_clean = ntc;

	/* 把新装填的描述符交还硬件。RDT 指向“下一个待装填”的下标，
	* 也就是最后一个有效描述符的下一个。
	*/
	if (refilled) {
		ring->next_to_use = ntc;
		wmb();
		writel(ring->next_to_use, ring->hw_addr + IXGBE_RDT(0));
	}

	return done;
}

/* ---------------------------------------------------------------------------
* 4. 在 ixgbe 的队列初始化里调用
* ---------------------------------------------------------------------------
* 真实驱动里这些函数挂在 ixgbe_configure_rx_ring()/ixgbe_clean_rx_irq() 中，
* 由 netdev 的 NAPI 轮询调用。ring->hw_addr 来自 adapter->hw.hw_addr。
*/
static int ibus_ixgbe_rx_setup(struct ibus_ixgbe_rx_ring *ring)
{
	int ret;

	ibus_ixgbe_config_split(ring);

	ret = ibus_ixgbe_alloc_rx_ring(ring);
	if (ret)
		return ret;

	dev_info(ring->dev, "header split: hdr=%u B, pkt=%u B, %u 个描述符\n",
			IBUS_HDR_SIZE, IBUS_PKT_SIZE, IBUS_RX_DESC_COUNT);
	return 0;
}

/*
* 常见坑：
*   1. DD 位就在 read.pkt_addr 的 bit0。重新装填时不能只写“地址 | 0”，
*      必须写一个 bit0=0 的合法 DMA 地址，否则硬件认为描述符还没被释放。
*   2. hdr_info 只有在 DD=1 后才有效；SPH=0 时不要读 hdr 缓冲。
*   3. 头部长度是“硬件写回的头字节数”，不是驱动配置的 BSIZEHDR。
*      实际切点由协议解析决定，BSIZEHDR 只是容量上限。
*   4. ERR_HBO 说明头比头缓冲大（比如超长 TCP 选项），包已经不可靠，
*      必须丢；把 BSIZEHDR 调大或不用头部分离。
*   5. 载荷页在 dma_unmap_page 之后才能交给 skb；顺序反了会读到
*      还没同步的 cache 行（非 coherent 架构上尤其明显）。
*   6. skb_add_rx_frag 会把页的所有权转给 skb，不能再 __free_page。
*/
