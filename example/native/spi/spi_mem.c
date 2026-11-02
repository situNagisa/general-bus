/*
* 不在本机编译：这是 Linux 内核模块代码，用的是内核内部 API
* <linux/spi/spi-mem.h>，必须在内核源码树里用 kbuild 编译成 .ko；本机
* （WSL2，6.18）没有内核源码树、没有任何 SPI 控制器，用户态 gcc 无法编译，
* 也无法加载运行。另外 Linux 内核不提供 C++ 运行时（没有 new/异常/STL，
* 内核里的 C++ 代码不会被接受），驱动只能写 C —— 所以本文件是本目录唯一
* 保留 .c 后缀的示例，其它示例都是 .cpp。下面所有 API/字段名都对着
* 上游 include/linux/spi/spi-mem.h 和 drivers/spi/spi-mem.c 核过；
* 本机装不了内核头文件，所以具体内核版本以注释为准。
*
* 一、这是什么
* spi-mem 是 Linux 内核里“SPI 存储器”的通用传输层。SPI NOR/NAND flash、
* SPI SRAM、SPI EEPROM 的线上时序都是同一种形状：
*
*	CS# __/‾‾‾ 8 位操作码 ... 地址 ... 交替字节 ... dummy ... 数据 ... ‾\__
*
* 一条命令由固定顺序的若干“阶段”组成，而且每个阶段用的线数可以不同
* （1/2/4/8 线，即 single/dual/quad/octal）。老的 spi_message 抽象只能描述
* “一段 Tx + 一段 Rx”，没法表达“地址阶段 1 线、数据阶段 4 线”这种形状，
* 于是内核加了一层 spi-mem：把整条命令描述成一个 struct spi_mem_op，交给
* 控制器驱动，由它把每个阶段变成硬件时序。spi-nor、spi-nand、mtd 都建在
* 这一层上。
*
* 二、解决什么问题 / 什么时候用
* - 你写的是 flash 类存储器的驱动（或者要在驱动里直接读 flash 的 ID、
*   状态寄存器、OTP），而不是普通 SPI 外设（传感器、ADC 那种），就用它。
* - 普通外设用 spi_write()/spi_read()/spi_sync()；存储器用 spi_mem_exec_op()。
* - 写一个“最小 SPI NOR 读驱动”：probe 时读 JEDEC ID 确认片子在线，然后
*   从地址 0 读一页数据。
*
* 三、形态（一次调用交几段内存、几个线上单元、提交几次、完成几次）
* - 一次 spi_mem_exec_op() 只描述并执行“一条命令”，最多四个阶段：
*	cmd（操作码）→ addr（地址）→ dummy（dummy 时钟）→ data（数据）。
*	阶段顺序由 flash 规定，调用者只能选择“有/没有”，不能重排。
*	（spi-mem 的通用模型里没有“交替字节/模式位”阶段；需要时通常把操作码
*	扩成 2 字节，或者由控制器驱动自己处理。）
* - 一次调用对应“一次片选有效”，里面可以有多段 SPI 传输：控制器有原生
*	mem_ops->exec_op 时由硬件一口气做完；没有时核心会自己拼一个
*	spi_message（cmd/addr/dummy 用一块 kmalloc 的临时缓冲发出去，data 那
*	一段直接用你给的缓冲区），仍然是一次 spi_sync、一次片选。
* - 提交 1 次、完成 1 次：同步阻塞调用，返回时整条命令（含数据阶段）已经
*	做完。成功返回 0，失败返回负 errno（-EOPNOTSUPP 形状不支持、
*	-EINVAL 参数非法、-EIO 实际传输长度和预期不符、-ETIMEDOUT 等）。
* - 内存由调用者提供：数据缓冲区的指针直接写进 op.data.buf.in/out，
*	核心不拷贝。要求这块内存是 DMA 可用的（不能是栈上数组，核心有
*	object_is_on_stack() 检查并 WARN）。
* - 一次调用只能传“一段”数据，读或写二选一，由 data.dir 决定。
*
* 四、需要什么
* - 内核开了 CONFIG_SPI_MEM（依赖 CONFIG_SPI），且有支持 spi-mem 的控制器：
*   控制器驱动要提供 spi_controller_mem_ops（至少 exec_op）。
* - 设备树里 flash 节点的 compatible 要和本驱动匹配上。
* - spi_mem_* 符号是 EXPORT_SYMBOL_GPL，模块许可证必须是 GPL 兼容的。
*
* 五、怎么编译、怎么运行
*	# 在放本文件的目录里建一个 Makefile：
*	#   obj-m += nor_demo.o
*	#   nor_demo-objs := spi_mem.o
*	make -C /lib/modules/$(uname -r)/build M=$PWD modules
*	sudo insmod nor_demo.ko
*	dmesg | tail -n 5
*	sudo rmmod nor_demo
*   （不能直接写 obj-m += spi_mem.o：模块名会和内核里的 spi-mem 核心重名。
*    用 xxx-objs 把本文件编进一个改名后的模块。）
*
* 六、预期输出（以 W25Q128 为例）
*	demo-nor spi0.0: JEDEC ID ef 40 18
*	demo-nor spi0.0: read 256 bytes from 0x0, first byte 0xff
*   读失败时对应的 dev_err 会打印 errno。
*/

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/errno.h>
#include <linux/of.h>
#include <linux/spi/spi.h>
#include <linux/spi/spi-mem.h>

/* W25Q128FV 的读 ID 与快速读命令。真驱动应该在 probe 里读 ID 后查表，
* 这里从简，只演示 spi-mem 的用法。
* 0x9F = Read JEDEC ID：只有指令 + 数据两个阶段。
* 0x6B = Fast Read Quad Output：1 线指令 + 1 线地址 + 8 个 dummy 时钟
*	+ 4 线数据，正好演示“每个阶段线数不同”。
*/
#define DEMO_RDID_OPCODE	0x9f
#define DEMO_READ_OPCODE	0x6b

/* W25Q128 用 3 字节（24 位）地址。32 MiB 以上的片子要用 4 字节地址
* 命令（例如 0x6C Fast Read Quad Output with 4-Byte Address）。 */
#define DEMO_ADDR_BYTES		3

/* 假设控制器一次最多搬 128 字节。真实上限由控制器的 adjust_op_size
* 告诉我们，这个宏只是为了让示例里的分块循环一定跑起来。 */
#define DEMO_MAX_CHUNK		128

#define DEMO_PAGE_SIZE		256

struct demo_nor {
	struct spi_mem *mem;
	u8 id[3];
	u8 *buf;	/* 页缓冲，必须 DMA 可用，所以 kmalloc 而不是栈数组 */
};

/*
* 读 JEDEC ID：最简形状，只有 cmd + data 两个阶段。
*
* 顺便解释 struct spi_mem_op 的各个成员：
*   cmd.nbytes    操作码字节数（非 DTR 时核心只接受 1）
*   cmd.buswidth  操作码在几条线上发：1/2/4/8
*   cmd.opcode    操作码本身
*   addr.nbytes   地址字节数，0 表示没有地址阶段
*   addr.buswidth 地址阶段的线数
*   addr.val      地址值（u64，按 addr.nbytes 截取，MSB 先出）
*   dummy.nbytes  dummy 阶段“字节数”，0 表示没有 dummy
*   dummy.buswidth dummy 阶段的线数
*   data.buswidth 数据阶段的线数
*   data.dir      SPI_MEM_DATA_IN（设备→主机，读）或 SPI_MEM_DATA_OUT
*   data.nbytes   数据字节数
*   data.buf.in   读缓冲 / data.buf.out 写缓冲（同一个 union）
*
* dummy 的坑：它按“字节”给，硬件上却是“时钟周期”。核心算周期数的公式是
*	cycles = dummy.nbytes * 8 / dummy.buswidth          （DTR 再除以 2）
* 反过来要表达 8 个 dummy 周期：
*	1 线：nbytes = 8 * 1 / 8 = 1
*	4 线：nbytes = 8 * 4 / 8 = 4
* 直接把“周期数”填进 nbytes 是最常见的错误：1 线时会把 8 个周期变成
* 64 个，4 线时也会变成 16 个。
* spi-nor 核心里也是先固定 dummy.buswidth = data.buswidth，再按上面的
* 公式换算出 nbytes。
*/
static int demo_nor_read_id(struct demo_nor *nor)
{
	/* 四个参数顺序固定：cmd、addr、dummy、data，和结构体成员顺序一致。
	* 没有的阶段用 SPI_MEM_OP_NO_XXX 占位，它展开成空的指定初始化器，
	* 对应结构体成员全为 0。
	*/
	struct spi_mem_op op = SPI_MEM_OP(
		SPI_MEM_OP_CMD(DEMO_RDID_OPCODE, 1),
		SPI_MEM_OP_NO_ADDR,
		SPI_MEM_OP_NO_DUMMY,
		SPI_MEM_OP_DATA_IN(ARRAY_SIZE(nor->id), nor->id, 1));
	int ret;

	/* spi_mem_supports_op() 只回答“这个形状能不能做”，返回 bool：
	* 它先做通用检查（cmd.nbytes 必须是 1、各阶段 buswidth 必须是
	* 1/2/4/8、数据缓冲区不能在栈上），再问控制器驱动的 supports_op。
	* 注意：控制器没实现 supports_op 时用的是核心的默认检查，并不可靠，
	* 真正的“不支持”也可能直到 exec_op 才以 -EOPNOTSUPP 返回。
	* （这个函数历史上签名变化过：有的分支/版本返回 int 并用负数表示
	*  错误，有的只返回 bool。这里只按 bool 用，具体版本未核对。）
	*/
	if (!spi_mem_supports_op(nor->mem, &op))
		return -EOPNOTSUPP;

	/* 让控制器按自己的限制缩小 op（FIFO 深度、DMA 对齐、最大传输长度
	* 等）。它可能把 op.data.nbytes 改小，所以之后必须用改完的值，
	* 不能再假设还是原来要的长度。返回 0 表示 op 可用（可能已缩小），
	* 负数表示控制器没法把这个 op 调整成它能做的形状。
	*/
	ret = spi_mem_adjust_op_size(nor->mem, &op);
	if (ret)
		return ret;

	if (op.data.nbytes != ARRAY_SIZE(nor->id))
		return -EOPNOTSUPP;

	/* 同步执行：内部会拿这条 SPI 总线的锁（spi_mem_access_start），
	* 所以可能睡眠，只能在进程上下文调用，不能在中断/原子上下文用。
	*/
	return spi_mem_exec_op(nor->mem, &op);
}

/*
* 从 flash 的 addr 处读 len 字节。
*
* 这里演示“一次 exec_op 只传一段数据、长度受控制器限制”时怎么循环：
* 每轮重新构造 op（因为 adjust_op_size 会改数据长度），用改完的
* op.data.nbytes 推进偏移，直到读完。
*/
static int demo_nor_read(struct demo_nor *nor, u64 addr, void *dst, size_t len)
{
	size_t done = 0;

	while (done < len) {
		/* data.nbytes 是 unsigned int，这里先自己限一个块大小，
		* 免得把超长请求直接丢给控制器。 */
		const u8 chunk = (u8)min_t(size_t, len - done, DEMO_MAX_CHUNK);
		struct spi_mem_op op = SPI_MEM_OP(
			SPI_MEM_OP_CMD(DEMO_READ_OPCODE, 1),
			SPI_MEM_OP_ADDR(DEMO_ADDR_BYTES, addr + done, 1),
			SPI_MEM_OP_DUMMY(4, 4),
			SPI_MEM_OP_DATA_IN(chunk, (u8 *)dst + done, 4));
		int ret;

		/* 注意上面 dummy 写的是 4 字节 × 4 线：按公式
		* 4 * 8 / 4 = 8 个时钟周期，正好是 0x6B 要的 8 个 dummy。
		* 写成 SPI_MEM_OP_DUMMY(8, 1) 也是 8 个周期，效果一样。
		*/

		if (!spi_mem_supports_op(nor->mem, &op))
			return -EOPNOTSUPP;

		ret = spi_mem_adjust_op_size(nor->mem, &op);
		if (ret)
			return ret;

		/* adjust 后可能变成 0（控制器连一个字都排不下），要防死循环 */
		if (!op.data.nbytes)
			return -EINVAL;

		ret = spi_mem_exec_op(nor->mem, &op);
		if (ret)
			return ret;

		/* 只有这一步成功了，这 chunk 字节才算真读进 dst */
		done += op.data.nbytes;
	}

	return 0;
}

static int demo_nor_probe(struct spi_mem *mem)
{
	struct demo_nor *nor;
	int ret;

	/* 驱动私有数据用 devm_kzalloc：随设备销毁自动释放，remove 里不用管。
	* 它同时充当读 ID 的缓冲区 —— 堆内存满足“DMA 可用”的要求。 */
	nor = devm_kzalloc(&mem->spi->dev, sizeof(*nor), GFP_KERNEL);
	if (!nor)
		return -ENOMEM;

	nor->mem = mem;

	/* 把私有数据挂到 spi_mem 上，remove 里用 spi_mem_get_drvdata 取回。
	* 注意这个是 spi_mem 的 drvpriv，不是 spi_device 的 drvdata。 */
	spi_mem_set_drvdata(mem, nor);

	nor->buf = devm_kmalloc(&mem->spi->dev, DEMO_PAGE_SIZE, GFP_KERNEL);
	if (!nor->buf)
		return -ENOMEM;

	ret = demo_nor_read_id(nor);
	if (ret) {
		/* probe 返回负数，设备就不会绑定到这个驱动；dmesg 里能看到
		* 核心打的 "probe failed" 以及这里的错误码。 */
		dev_err(&mem->spi->dev, "read JEDEC ID failed: %d\n", ret);
		return ret;
	}

	dev_info(&mem->spi->dev, "JEDEC ID %02x %02x %02x\n",
		nor->id[0], nor->id[1], nor->id[2]);

	ret = demo_nor_read(nor, 0, nor->buf, DEMO_PAGE_SIZE);
	if (ret) {
		dev_err(&mem->spi->dev, "read page failed: %d\n", ret);
		return ret;
	}

	dev_info(&mem->spi->dev, "read %u bytes from 0x0, first byte %02x\n",
		(unsigned int)DEMO_PAGE_SIZE, nor->buf[0]);

	return 0;
}

/*
* 注意 remove 的返回类型：当前上游 struct spi_mem_driver 里是 int，
* core 的 spi_mem_remove() 直接调用并忽略返回值。若你手上的内核把它
* 改成了 void，就去掉这里的 return 0。以本机内核头文件为准。
*/
static int demo_nor_remove(struct spi_mem *mem)
{
	return 0;
}

/* id_table 用于没有设备树时按名字匹配；of_match_table 用于设备树。
* 两个都给上，示例里任选其一即可工作。 */
static const struct spi_device_id demo_nor_ids[] = {
	{ "demo-nor", 0 },
	{ }
};
MODULE_DEVICE_TABLE(spi, demo_nor_ids);

static const struct of_device_id demo_nor_of_match[] = {
	{ .compatible = "example,demo-nor" },
	{ }
};
MODULE_DEVICE_TABLE(of, demo_nor_of_match);

/*
* spi_mem_driver 只是 spi_driver 的薄包装：核心（spi_mem_probe）会替我们
* 分配 struct spi_mem，然后把 probe(mem)/remove(mem) 转发过来。
* spidrv.driver.name 是驱动名，spidrv.id_table / of_match_table 决定匹配。
*/
static struct spi_mem_driver demo_nor_driver = {
	.spidrv = {
		.driver = {
			.name = "demo-nor",
			.of_match_table = demo_nor_of_match,
		},
		.id_table = demo_nor_ids,
	},
	.probe = demo_nor_probe,
	.remove = demo_nor_remove,
};

/* module_spi_mem_driver 展开成 module_init/module_exit，内部调
* spi_mem_driver_register()（= spi_mem_driver_register_with_owner(THIS_MODULE)）
* 和 spi_mem_driver_unregister()。 */
module_spi_mem_driver(demo_nor_driver);

MODULE_DESCRIPTION("最小 spi-mem SPI NOR 读驱动示例");
MODULE_LICENSE("GPL");
