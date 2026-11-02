/* 不在本机编译：Linux 内核驱动内部 API（dmaengine），只能对目标内核编译，并且需要一块带 DMA 控制器的板子。
* （Linux 内核只能用 C 编译，所以本文件保持 .c，不改成 .cpp。）
*
* 这是驱动实现 link 的手段，link 层看不到它。
* link 层只知道“把这 N 段内存按顺序送到链路上”“搬完告诉我”；至于
* dma_request_chan / 描述符 / cookie / 尾指针 / 中断，全在驱动里，link 层一概不见。
*
* ============================================================================
* 这个接口是什么
* ============================================================================
* dmaengine 是 Linux 内核里“从设备 DMA（slave DMA）”的通用客户端 API。
* 一块 SoC 上可能有 DW DMA、PL330、EDMA、STM32 DMA、Intel IOAT 等十几种
* DMA 控制器，寄存器布局和描述符格式各不相同。外设驱动（UART、SPI、I2S、
* 网卡……）不应该为每种控制器写一份搬运代码，所以内核抽象出 dmaengine：
*
*   - 客户端只描述“内存里的哪几段、搬到哪个外设 FIFO、总线宽度多少”；
*   - DMA 控制器驱动负责把这些描述翻译成硬件描述符链，配好寄存器；
*   - 客户端拿到一个 cookie，用它查询/等待完成。
*
* 什么时候用：外设自己有 FIFO，但没有（或不想用）总线 master 能力，需要
* 有人替它在内存和外设寄存器之间搬数据，并且希望 CPU 不参与每个字节。
*
* ============================================================================
* 形态（一次调用交几段内存、对应几个线上单元、提交几次、完成几次）
* ============================================================================
*   - 一次 dmaengine_prep_slave_sg() 交 N 段内存（N 个 scatterlist 条目），
*     对应硬件上一条描述符链。链按顺序执行，最终表现为“N 段数据依次送到
*     外设 FIFO”。线上是一个连续字节流，没有额外的包边界——边界完全由你
*     提交的段顺序决定。
*   - 提交（submit）一次：dmaengine_submit() 返回一个 dma_cookie_t，
*     整条链只有一个 cookie。
*   - 启动一次：dma_async_issue_pending()。之后 DMA 控制器自己跑。
*   - 完成一次：整条链跑完，若描述符带了 DMA_PREP_INTERRUPT，回调函数被
*     调用一次；也可以不用中断，在别处用 dmaengine_tx_status() 轮询。
*   - 循环模式（dmaengine_prep_dma_cyclic()）：一次提交，缓冲区被切成
*     buf_len / period_len 个 period，每跑完一个 period 回调一次，直到
*     你调用 dmaengine_terminate_sync()。它没有“整条链完成”这一次。
*   - 内存由调用方提供：源缓冲是你的 kmalloc/页，必须先用 dma_map_sg()
*     映射成设备能看见的 DMA 地址；描述符环由 DMA 驱动自己分配。
*
* 描述符链到底有多长？dmaengine 没有“链长”这个参数：
*   - 调用方用 nents 决定“这次交几段”；
*   - 调用方还可以多次 prep + submit，把多条链排进同一个通道，按 FIFO 执行；
*   - 每一段最终变成几个硬件描述符，由 DMA 驱动和控制器决定：有的控制器
*     一个 sg 条目一个描述符，有的会合并物理相邻的条目，有的因为突发长度、
*     地址对齐限制把一段拆成好几个。
* 所以调用方能控制的是“交几段、分几次提交”这个上限，硬件描述符的精确
* 个数由驱动决定。
*
* ============================================================================
* 需要的条件
* ============================================================================
*   - 目标内核里该 DMA 控制器的驱动已经加载；
*   - 设备在 DT/ACPI 里有 dmas 属性，例如：
*         dma-demo {
*                 compatible = "ibus,dma-demo";
*                 dmas = <&dma1 5>, <&dma1 6>;
*                 dma-names = "tx", "rx";
*         };
*     dma_request_chan(dev, "tx") 就是按 "tx" 这个名字去查。
*   - 没有 DT 的话，dma_request_chan() 会返回 -ENODEV；纯内存拷贝可以用
*     dma_request_chan_by_mask()（不在本示例范围）。
*
* ============================================================================
* 怎么编译、怎么运行、预期输出
* ============================================================================
*   编译（外部模块，对运行中的内核）：
*         make -C /lib/modules/$(uname -r)/build M=$PWD modules
*   或者放进内核树 drivers/misc/ 并加 obj-m。
*   运行：把模块和匹配的 DT 节点一起部署到板子上，insmod dmaengine_sg.ko；
*         dmesg 里应看到：
*           ibus-dma-demo ...: dma_request_chan("tx") ok
*           ibus-dma-demo ...: sg: mapped 3 段, cookie=1
*           ibus-dma-demo ...: sg: 完成, status=DMA_COMPLETE, residue=0
*           ibus-dma-demo ...: cyclic: 4 个 period 回调, 已终止
*   没有板子/没有 DT 时 probe 不会跑，模块加载本身不报错，这是正常的。
*
* 版本说明：以下 API/字段按上游 v6.18 的 include/linux/dmaengine.h 核对
* （本机没有内核源码树，是查上游源码核对的，不是编译验证）。
*/

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/dmaengine.h>
#include <linux/dma-mapping.h>
#include <linux/scatterlist.h>
#include <linux/completion.h>
#include <linux/atomic.h>
#include <linux/err.h>
#include <linux/slab.h>

/* 真实驱动里这个地址来自外设自己的寄存器块，比如
*   res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
*   fifo = res->start + UART_TX_FIFO;
* 这里为了示例写死一个假地址：模块只做编译/结构演示，不会在真板子上
* 真的往 0x40000000 写东西（没有匹配的从设备时 DMA 驱动会拒绝或超时）。
*/
#define IBUS_DEMO_FIFO_ADDR	0x40000000u

#define IBUS_SG_ENTRIES		3
#define IBUS_CYCLIC_PERIODS	4
#define IBUS_CYCLIC_PERIOD	32

struct ibus_dma_demo {
	struct device *dev;
	struct dma_chan *chan;
	struct completion sg_done;	/* SG 链完成的信号 */
	struct completion cyclic_done;	/* 循环模式里数够 period 的信号 */
	dma_cookie_t cookie;		/* 最近一次提交拿到的 cookie */
	atomic_t periods;		/* 循环模式已经收到的 period 回调数 */
	int sg_cb_count;		/* SG 回调次数，验证“整条链只回调一次” */
};

/* 描述符完成回调。注意上下文：这个函数通常在 DMA 驱动的 tasklet/软中断里
* 被调用，不能睡眠（不能 kmalloc(GFP_KERNEL)、不能 mutex_lock、不能等 IO）。
* complete() 是安全的：它只唤醒等待者。
*/
static void ibus_dma_sg_done(void *arg)
{
	struct ibus_dma_demo *d = arg;

	d->sg_cb_count++;
	complete(&d->sg_done);
}

/* 循环模式的回调：每个 period 结束调用一次，arg 还是同一个指针。
* 同一个 cookie、同一条描述符链，回调却来多次——这就是 cyclic 的形态。
*/
static void ibus_dma_period_done(void *arg)
{
	struct ibus_dma_demo *d = arg;
	int n = atomic_inc_return(&d->periods);

	if (n >= IBUS_CYCLIC_PERIODS)
		complete(&d->cyclic_done);
}

/* 配置通道：方向、外设地址、总线宽度、最大突发。
* 这些参数交给 DMA 驱动，由它决定怎么切描述符、怎么配控制器寄存器。
* 返回 0 表示成功，负数是 -EINVAL/-ENOSYS 之类。
*/
static int ibus_dma_config(struct ibus_dma_demo *d)
{
	struct dma_slave_config cfg = {};

	cfg.direction = DMA_MEM_TO_DEV;			/* 内存 -> 外设 FIFO */
	cfg.dst_addr = IBUS_DEMO_FIFO_ADDR;		/* 外设端地址（FIFO 寄存器） */
	cfg.dst_addr_width = DMA_SLAVE_BUSWIDTH_4_BYTES;	/* 每次总线写 4 字节 */
	cfg.dst_maxburst = 16;				/* 每次突发最多 16 个总线宽度 */
	/* src_addr/src_addr_width 在 MEM_TO_DEV 下用不到，保持 0 即可。
	* 如果是 DEV_TO_MEM，就填 src_addr/src_maxburst。
	*/
	return dmaengine_slave_config(d->chan, &cfg);
}

/* ---------------------------------------------------------------------------
* 普通 SG 传输：N 段内存 -> 外设 FIFO
* ---------------------------------------------------------------------------
*/
static int ibus_dma_run_sg(struct ibus_dma_demo *d)
{
	struct scatterlist *sgl;
	void *bufs[IBUS_SG_ENTRIES];
	size_t lens[IBUS_SG_ENTRIES] = { 64, 1024, 16 };	/* 头、载荷、尾 */
	struct dma_async_tx_descriptor *desc;
	struct dma_tx_state state = {};
	unsigned long left;
	int mapped;
	int ret = 0;
	int i;

	sgl = kcalloc(IBUS_SG_ENTRIES, sizeof(*sgl), GFP_KERNEL);
	if (!sgl)
		return -ENOMEM;

	for (i = 0; i < IBUS_SG_ENTRIES; i++) {
		bufs[i] = kmalloc(lens[i], GFP_KERNEL);
		if (!bufs[i]) {
			ret = -ENOMEM;
			goto out_free;
		}
		memset(bufs[i], 0x40 + i, lens[i]);	/* 随便填点内容 */
	}

	/* sg_init_table 初始化 N 个条目并把最后一个标记为结束；
	* sg_set_buf 填 page/offset/length。它不建立 DMA 映射，只是描述内存。
	*/
	sg_init_table(sgl, IBUS_SG_ENTRIES);
	for (i = 0; i < IBUS_SG_ENTRIES; i++)
		sg_set_buf(&sgl[i], bufs[i], lens[i]);

	/* dma_map_sg 把 CPU 虚拟地址翻译成设备 DMA 地址，并处理 cache 一致性。
	* 返回值可能小于 IBUS_SG_ENTRIES：IOMMU 或 DMA 驱动可能把物理相邻的
	* 条目合并。后面的 prep 必须用返回值，不能用原始 nents。
	*/
	mapped = dma_map_sg(d->dev, sgl, IBUS_SG_ENTRIES, DMA_TO_DEVICE);
	if (!mapped) {
		dev_err(d->dev, "dma_map_sg 失败\n");
		ret = -EIO;
		goto out_free;
	}

	/* prep 只是“准备好一条链”，还没有交给硬件。
	* flags 里的 DMA_PREP_INTERRUPT（值为 1<<0）表示：这条链完成时要
	* 触发回调。不加它，callback 字段不会被调用（有的驱动仍可轮询）。
	*
	* 注意：这里传的是 mapped（映射后实际可用的段数），不是 3。
	*/
	desc = dmaengine_prep_slave_sg(d->chan, sgl, mapped,
					DMA_MEM_TO_DEV, DMA_PREP_INTERRUPT);
	if (!desc) {
		dev_err(d->dev, "dmaengine_prep_slave_sg 返回 NULL（资源不足）\n");
		ret = -ENOMEM;
		goto out_unmap;
	}

	/* 回调必须在 submit 之前挂好：submit 之后驱动随时可能启动甚至完成。 */
	desc->callback = ibus_dma_sg_done;
	desc->callback_param = d;

	/* submit 把描述符交给 DMA 驱动，返回 cookie（一个递增的序号）。
	* 它还没有启动硬件；真正的“开始”是下面的 issue_pending。
	* dma_submit_error() 判断 cookie 是否 < 0。
	*/
	d->cookie = dmaengine_submit(desc);
	if (dma_submit_error(d->cookie)) {
		dev_err(d->dev, "dmaengine_submit 失败: %d\n", d->cookie);
		ret = d->cookie;
		goto out_unmap;
	}

	/* 把该通道上排队的描述符一次性踢给硬件。多次 submit 只需在这里
	* 踢一次；也可以每次 submit 后都踢。
	* 启动前把上一次可能残留的完成状态清掉，避免重复运行同一函数时
	* wait_for_completion_timeout() 立刻返回。
	*/
	d->sg_cb_count = 0;
	reinit_completion(&d->sg_done);
	dma_async_issue_pending(d->chan);

	/* 回调在中断上下文跑，这里睡等。超时不代表传输失败，可能只是
	* 设备没接、FIFO 不满；超时后要 terminate 把链停掉。
	*/
	left = wait_for_completion_timeout(&d->sg_done, msecs_to_jiffies(1000));
	if (!left) {
		dev_err(d->dev, "sg: 1 秒内没有完成，终止\n");
		dmaengine_terminate_sync(d->chan);
		ret = -ETIMEDOUT;
		goto out_unmap;
	}

	/* tx_status 用 cookie 查这条链的状态：
	*   DMA_COMPLETE    已完成
	*   DMA_IN_PROGRESS 还在跑
	*   DMA_PAUSED      暂停
	*   DMA_ERROR       出错
	* state.residue 是“还剩多少字节没搬”，完成时应为 0；
	* 有些驱动只填部分字段，别把它当唯一判据。
	*/
	if (dmaengine_tx_status(d->chan, d->cookie, &state) != DMA_COMPLETE)
		dev_warn(d->dev, "sg: 回调已到但状态不是 DMA_COMPLETE\n");

	dev_info(d->dev, "sg: mapped %d 段, cookie=%d, 回调 %d 次, residue=%u\n",
			mapped, d->cookie, d->sg_cb_count, state.residue);

out_unmap:
	/* 传回原始的 nents 还是 mapped？两种写法在内核里都能见到；
	* DMA-API 文档要求 unmap 的条目数与 map 时一致。这里用 mapped，
	* 与 dma_map_sg 的返回值对应。具体以 Documentation/core-api/
	* dma-api.rst 为准——本机没有内核文档，未核对这一处细节。
	*/
	dma_unmap_sg(d->dev, sgl, mapped, DMA_TO_DEVICE);
out_free:
	for (i = 0; i < IBUS_SG_ENTRIES; i++)
		kfree(bufs[i]);
	kfree(sgl);
	return ret;
}

/* ---------------------------------------------------------------------------
* 循环模式：同一个缓冲区被 DMA 反复搬，每个 period 完成回调一次
* ---------------------------------------------------------------------------
* 常见于音频（I2S 环形缓冲）、持续采样的 ADC。要点：
*   - buf_len 必须能被 period_len 整除，否则 prep 返回 NULL；
*   - 一次 prep + 一次 submit + 一次 issue_pending；
*   - 回调每个 period 一次，不是整圈一次；
*   - 缓冲区在 terminate 之前不能 unmap、不能释放；
*   - 不能在回调里调用 dmaengine_terminate_sync()（它会睡眠）。
*/
static int ibus_dma_run_cyclic(struct ibus_dma_demo *d)
{
	size_t buf_len = IBUS_CYCLIC_PERIODS * IBUS_CYCLIC_PERIOD;
	struct dma_async_tx_descriptor *desc;
	dma_addr_t buf_dma;
	void *buf;
	unsigned long left;
	int ret = 0;

	buf = kmalloc(buf_len, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;
	memset(buf, 0x5a, buf_len);

	buf_dma = dma_map_single(d->dev, buf, buf_len, DMA_TO_DEVICE);
	if (dma_mapping_error(d->dev, buf_dma)) {
		ret = -EIO;
		goto out_free;
	}

	/* period_len 是“每多少次回调切一刀”的粒度，硬件描述符链会按
	* buf_len / period_len 组织；具体怎么摊到硬件描述符上由驱动决定。
	*/
	desc = dmaengine_prep_dma_cyclic(d->chan, buf_dma, buf_len,
						IBUS_CYCLIC_PERIOD, DMA_MEM_TO_DEV,
						DMA_PREP_INTERRUPT);
	if (!desc) {
		ret = -ENOMEM;
		goto out_unmap;
	}

	desc->callback = ibus_dma_period_done;
	desc->callback_param = d;

	d->cookie = dmaengine_submit(desc);
	if (dma_submit_error(d->cookie)) {
		ret = d->cookie;
		goto out_unmap;
	}

	atomic_set(&d->periods, 0);
	reinit_completion(&d->cyclic_done);
	dma_async_issue_pending(d->chan);

	/* 等收到 4 个 period 的回调。循环传输本身不会“整体完成”，
	* 所以这里等的是“够数了”，不是 DMA 停了。
	*/
	left = wait_for_completion_timeout(&d->cyclic_done, msecs_to_jiffies(2000));
	if (!left) {
		dev_err(d->dev, "cyclic: 2 秒内没收到 %d 个 period\n",
			IBUS_CYCLIC_PERIODS);
		ret = -ETIMEDOUT;
	}

	/* 停掉循环并等它真正停下，之后才能 unmap。 */
	dmaengine_terminate_sync(d->chan);
	dev_info(d->dev, "cyclic: 已收到 %d 个 period 回调, 已终止\n",
			atomic_read(&d->periods));

out_unmap:
	dma_unmap_single(d->dev, buf_dma, buf_len, DMA_TO_DEVICE);
out_free:
	kfree(buf);
	return ret;
}

static int ibus_dma_demo_probe(struct platform_device *pdev)
{
	struct ibus_dma_demo *d;
	int ret;

	d = devm_kzalloc(&pdev->dev, sizeof(*d), GFP_KERNEL);
	if (!d)
		return -ENOMEM;
	d->dev = &pdev->dev;
	init_completion(&d->sg_done);
	init_completion(&d->cyclic_done);
	atomic_set(&d->periods, 0);

	/* 按名字申请通道。返回 ERR_PTR，不是 NULL：
	*   -EPROBE_DEFER 表示 DMA 控制器还没注册好，应该返回它让内核稍后重试；
	*   -ENODEV 表示 DT 里没有这个 dma-names，或者根本没有 dmas 属性。
	*/
	d->chan = dma_request_chan(d->dev, "tx");
	if (IS_ERR(d->chan)) {
		ret = PTR_ERR(d->chan);
		if (ret == -EPROBE_DEFER)
			return ret;
		dev_err(d->dev, "dma_request_chan(\"tx\") 失败: %d\n", ret);
		return ret;
	}
	dev_info(d->dev, "dma_request_chan(\"tx\") ok\n");

	ret = ibus_dma_config(d);
	if (ret) {
		dev_err(d->dev, "dmaengine_slave_config 失败: %d\n", ret);
		goto out_release;
	}

	/* 演示一：一条链，3 段内存。
	* 失败不影响卸载：仍然要把通道还回去。
	*/
	(void)ibus_dma_run_sg(d);

	/* 演示二：循环模式，4 个 period。 */
	(void)ibus_dma_run_cyclic(d);

out_release:
	dma_release_channel(d->chan);
	return 0;	/* 演示模块：失败也返回 0，方便反复 insmod 观察 */
}

static const struct of_device_id ibus_dma_demo_of_match[] = {
	{ .compatible = "ibus,dma-demo" },
	{ }
};
MODULE_DEVICE_TABLE(of, ibus_dma_demo_of_match);

static struct platform_driver ibus_dma_demo_driver = {
	.probe = ibus_dma_demo_probe,
	.driver = {
		.name = "ibus-dma-demo",
		.of_match_table = ibus_dma_demo_of_match,
	},
};
module_platform_driver(ibus_dma_demo_driver);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("dmaengine slave SG / cyclic usage demo");
