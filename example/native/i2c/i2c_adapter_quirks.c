/* 不在本机编译：这是 Linux 内核驱动内部 API（include/linux/i2c.h 的
* struct i2c_adapter_quirks / i2c_algorithm，以及 platform_driver 等），
* 用户态工具链没有这些头文件；而且内核只能用 C 写，不能改成 C++。
* 要验证请放进内核源码树（例如 drivers/i2c/busses/）或做成外部模块，
* 对着 6.18 的内核头文件编译。
*
* i2c_adapter_quirks.c —— 给 I2C 适配器声明“硬件限制”，让 i2c core 提前拒绝
*
* 这个接口是什么
*	struct i2c_adapter_quirks 是内核里适配器驱动描述自身硬件限制的一组字段：
*	最多几条 i2c_msg、单条写/读最长多少字节、组合事务两段各自最长多少字节、
*	以及一些布尔型限制（不支持 clock stretching、不接受零长度读、组合事务
*	必须是“先写后读”且同一地址等）。适配器驱动把它挂在 adapter->quirks 上，
*	i2c core 在每次 i2c_transfer() 真正调用 master_xfer 之前，用
*	i2c_check_for_quirks() 逐条检查消息；只要有一条超出限制，就直接返回
*	-EOPNOTSUPP，根本不会碰硬件。它解决的问题是：很多 I2C 控制器不是“任意
*	I2C 都能发”的通用硬件，与其让驱动在 master_xfer 里手忙脚乱地拆分/报错，
*	不如把限制声明出来，让 core 统一拒绝并打印原因。
*
* 形态（一次调用交几段内存、对应几个线上单元、提交几次、完成几次）
*	quirks 不是系统调用，没有“提交/完成”。它是挂在适配器上的只读常量，
*	对每一次 i2c_transfer(adap, msgs, num) 生效：
*	  - 一次 transfer 可以有 num 条 i2c_msg（每条形如 START + 地址 + 数据）；
*	  - core 先做一次整体检查（不产生任何线上时序），通过后才调用
*	    adap->algo->master_xfer() 产生真正的 START/数据/STOP；
*	  - 检查失败返回 -EOPNOTSUPP，dmesg 里会有
*	    “adapter quirk: ... (addr ..., size ..., read/write)” 的限速日志。
*	所以它是“每批消息一次静态检查”，不改变传输的提交/完成模型。
*
* 需要的内核/硬件条件
*	- 内核 6.18 的 include/linux/i2c.h（本机没有内核源码，字段名、类型和
*	  I2C_AQ_* 的取值已在线核对 kernel.org 上 v6.18 的 include/linux/i2c.h；
*	  检查逻辑对照 drivers/i2c/i2c-core-base.c，本机无法逐行核对，标“未核对”）。
*	- 一个 platform 设备（设备树 compatible = "example,my-i2c-quirks"）和
*	  一段 MMIO 寄存器；下面的寄存器模型是虚构的最小控制器，只用于展示
*	  master_xfer 与 quirks 的配合。
*
* 编译
*	作为内核模块（内核源码树内）：
*	  把本文件放到 drivers/i2c/busses/i2c-adapter-quirks-example.c，
*	  在 drivers/i2c/busses/Kconfig 里加一个 tristate 配置，
*	  在 drivers/i2c/busses/Makefile 里加
*	      obj-$(CONFIG_I2C_ADAPTER_QUIRKS_EXAMPLE) += i2c-adapter-quirks-example.o
*	  然后 make M=drivers/i2c/busses modules。
*	（本示例目录不放 Makefile，构建方式只写在注释里。）
*
* 运行/预期
*	insmod 后 dmesg 出现 “registered i2c adapter N”，/dev/i2c-N 出现。
*	在用户态：
*	  - 读 1 字节（SMBus read byte data，2 条 msg，长度 1/1）可以通过；
*	  - 写 20 字节（单条 msg 长度 20 > max_write_len=16）→ -EOPNOTSUPP；
*	  - 一次传 3 条 msg（> max_num_msgs=2）→ -EOPNOTSUPP；
*	  - 零长度读（I2C_M_RD 且 len=0）→ -EOPNOTSUPP（NO_ZERO_LEN_READ）；
*	  - 组合事务第一段是读（不是写）→ -EOPNOTSUPP（COMB_WRITE_FIRST）；
*	  - 组合事务两段地址不同 → -EOPNOTSUPP（COMB_SAME_ADDR）。
*	以上拒绝都发生在 master_xfer 之前，硬件不会收到任何时序。
*
* I2C_AQ_* 标志（6.18 取值，在线核对）
*	#define I2C_AQ_COMB			BIT(0)
*	#define I2C_AQ_COMB_WRITE_FIRST		BIT(1)
*	#define I2C_AQ_COMB_READ_SECOND		BIT(2)
*	#define I2C_AQ_COMB_SAME_ADDR		BIT(3)
*	#define I2C_AQ_COMB_WRITE_THEN_READ	(I2C_AQ_COMB | I2C_AQ_COMB_WRITE_FIRST | \
*						 I2C_AQ_COMB_READ_SECOND | I2C_AQ_COMB_SAME_ADDR)
*	#define I2C_AQ_NO_CLK_STRETCH		BIT(4)
*	#define I2C_AQ_NO_ZERO_LEN_READ		BIT(5)
*	#define I2C_AQ_NO_ZERO_LEN_WRITE	BIT(6)
*	#define I2C_AQ_NO_ZERO_LEN		(I2C_AQ_NO_ZERO_LEN_READ | \
*						 I2C_AQ_NO_ZERO_LEN_WRITE)
*	#define I2C_AQ_NO_REP_START		BIT(7)
*	注意：旧内核里这些标志的编号和组合方式不同（例如 COMB_WRITE_THEN_READ
*	曾是单个 BIT(3)，具体版本未核对）；源码里只用宏名就兼容，不要硬编码数值。
*	I2C_AQ_NO_CLK_STRETCH / I2C_AQ_NO_REP_START 在 core 中的具体检查位置
*	本机没有内核源码，未核对；下面不依赖它们的行为。
*/

#include <linux/bits.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/errno.h>
#include <linux/i2c.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/property.h>
#include <linux/slab.h>
#include <linux/string.h>

/* 虚构控制器的寄存器布局。真实驱动按数据手册改，但 master_xfer 的形状一样。 */
struct my_i2c_regs {
	u32 ctrl;	/* 控制：使能、START、STOP、读方向、ACK */
	u32 status;	/* 状态：字节完成、从机 NACK */
	u32 data;	/* 发送/接收数据 FIFO（这里是单字节） */
	u32 addr;	/* 本次消息的从机地址 + R/W 位 */
};

#define MY_I2C_CTRL_EN		BIT(0)
#define MY_I2C_CTRL_START	BIT(1)
#define MY_I2C_CTRL_STOP	BIT(2)
#define MY_I2C_CTRL_READ	BIT(3)
#define MY_I2C_CTRL_ACK		BIT(4)

#define MY_I2C_STATUS_DONE	BIT(0)
#define MY_I2C_STATUS_NACK	BIT(1)

#define MY_I2C_TIMEOUT_US	10000
#define MY_I2C_POLL_US		100

struct my_i2c {
	struct i2c_adapter adap;
	struct my_i2c_regs __iomem *regs;
	struct clk *clk;
	int bus_num;
};

/* 等一个字节的完成位；返回 0 成功，-ETIMEDOUT 超时。
* readl_poll_timeout 会在 delay_us 间隔轮询，直到 cond 为真或超时。 */
static int my_i2c_wait_done(struct my_i2c *priv)
{
	u32 status;

	return readl_poll_timeout(&priv->regs->status, status,
					status & MY_I2C_STATUS_DONE,
					MY_I2C_POLL_US, MY_I2C_TIMEOUT_US);
}

/* 发一条 i2c_msg。last 表示这是本次 transfer 的最后一条：
* 最后一条在末尾发 STOP，否则留给下一条发 repeated START。
*
* 这个控制器只会“固定长度、逐字节”传输，不认识 I2C_M_TEN / I2C_M_RECV_LEN
* 等扩展标志；遇到就返回 -EOPNOTSUPP。正常情况下 core 的 quirks 检查和
* functionality 已经挡住不支持的能力，这里是最后一道防线。 */
static int my_i2c_xfer_one(struct i2c_adapter *adap, struct i2c_msg *msg, bool last)
{
	struct my_i2c *priv = i2c_get_adapdata(adap);
	u32 ctrl;
	int i, ret;

	if (msg->flags & (I2C_M_TEN | I2C_M_RECV_LEN |
				I2C_M_IGNORE_NAK | I2C_M_NO_RD_ACK |
				I2C_M_REV_DIR_ADDR | I2C_M_NOSTART |
				I2C_M_STOP))
		return -EOPNOTSUPP;

	/* 地址字节：7 位地址左移 1 位，最低位是 R/W。 */
	writel((msg->addr << 1) | ((msg->flags & I2C_M_RD) ? 1 : 0),
		&priv->regs->addr);

	for (i = 0; i < msg->len; i++) {
		ctrl = MY_I2C_CTRL_EN | MY_I2C_CTRL_START;

		if (msg->flags & I2C_M_RD) {
			ctrl |= MY_I2C_CTRL_READ;
			/* 最后一个字节回 NACK，告诉从机别再发了。 */
			if (i != msg->len - 1)
				ctrl |= MY_I2C_CTRL_ACK;
		} else {
			writel(msg->buf[i], &priv->regs->data);
		}

		/* 只有最后一个字节且是最后一条消息时才发 STOP；
		* 否则下一段用 repeated START 接上。 */
		if (last && i == msg->len - 1)
			ctrl |= MY_I2C_CTRL_STOP;

		/* 本模型假定状态位是 W1C：先写 1 清掉上一字节的 DONE/NACK，
		* 再启动这一字节，轮询才是在等这一字节。真实硬件按手册清中断。 */
		writel(MY_I2C_STATUS_DONE | MY_I2C_STATUS_NACK,
			&priv->regs->status);
		writel(ctrl, &priv->regs->ctrl);

		ret = my_i2c_wait_done(priv);
		if (ret)
			return ret;
		if (readl(&priv->regs->status) & MY_I2C_STATUS_NACK)
			return -ENXIO;

		if (msg->flags & I2C_M_RD)
			msg->buf[i] = readl(&priv->regs->data) & 0xff;
	}

	/* 零长度消息：只有地址 + STOP（SMBus quick 的写方向就是这种形态）。
	* 上面的循环不会执行，这里补一次地址相位。 */
	if (msg->len == 0) {
		ctrl = MY_I2C_CTRL_EN | MY_I2C_CTRL_START;
		if (last)
			ctrl |= MY_I2C_CTRL_STOP;
		/* 本模型假定状态位是 W1C：先写 1 清掉上一字节的 DONE/NACK，
		* 再启动这一字节，轮询才是在等这一字节。真实硬件按手册清中断。 */
		writel(MY_I2C_STATUS_DONE | MY_I2C_STATUS_NACK,
			&priv->regs->status);
		writel(ctrl, &priv->regs->ctrl);

		ret = my_i2c_wait_done(priv);
		if (ret)
			return ret;
		if (readl(&priv->regs->status) & MY_I2C_STATUS_NACK)
			return -ENXIO;
	}

	return 0;
}

/* master_xfer：i2c core 真正调用硬件的入口。
* 约定：
*   - msgs 是 num 条消息的数组，由 core 分配/持有；
*   - 返回“成功处理的消息条数”（这里是 num），失败返回负 errno；
*   - 返回 0 在部分驱动里表示“没处理”，core 会转成 -EIO，别乱返回 0；
*   - 本函数被调用时，core 已经用 quirks 检查过 num 和每条 len。
* 状态位按上面的 W1C 假模型处理；真实硬件按手册清中断。 */
static int my_i2c_master_xfer(struct i2c_adapter *adap, struct i2c_msg *msgs,
				int num)
{
	int i, ret;

	for (i = 0; i < num; i++) {
		ret = my_i2c_xfer_one(adap, &msgs[i], i == num - 1);
		if (ret)
			return ret;
	}

	return num;
}

/* functionality 回调：告诉 i2c core 这个适配器会什么。
*	I2C_FUNC_I2C：支持任意 I2C 级 master_xfer；这是用户态 I2C_RDWR 的
*	              前提（i2c-dev 会检查 i2c_check_functionality(adap, I2C_FUNC_I2C)）。
*	I2C_FUNC_SMBUS_EMUL：允许 core 用普通 i2c_msg 模拟 SMBus 事务
*	              （quick/byte/byte data/word data/process call/write block/
*	              i2c block/PEC）。注意它不包含 READ_BLOCK_DATA；本控制器
*	              不支持 I2C_M_RECV_LEN，所以也不声明 READ_BLOCK_DATA。
*	不要在这里加 I2C_FUNC_10BIT_ADDR 之类没实现的能力：core 会信以为真，
*	把请求放进来，最后在 master_xfer 里失败。 */
static u32 my_i2c_functionality(struct i2c_adapter *adap)
{
	return I2C_FUNC_I2C | I2C_FUNC_SMBUS_EMUL;
}

static const struct i2c_algorithm my_i2c_algo = {
	.master_xfer = my_i2c_master_xfer,
	.functionality = my_i2c_functionality,
};

/* 关键：把硬件限制声明给 core。字段含义（6.18）：
*	flags：布尔型限制，见文件头。
*	max_num_msgs：一次 transfer 最多几条 i2c_msg；0 表示不限。
*	max_write_len：单条写消息的最大字节数；0 表示不限。
*	max_read_len ：单条读消息的最大字节数；0 表示不限。
*	max_comb_1st_msg_len / max_comb_2nd_msg_len：组合事务（恰好 2 条消息）
*	  第一段/第二段的最大字节数；0 表示不限。
*
* 这里声明：
*   - 只支持“先写后读、同一地址”的组合事务（COMB_WRITE_THEN_READ 是
*     COMB + WRITE_FIRST + READ_SECOND + SAME_ADDR 四个位的组合）；
*   - 不接受零长度读（硬件无法只发地址 + 读位就结束）；
*   - 最多 2 条消息（组合模式本身也强制 max_num=2，这里写出来更直观）；
*   - 普通单条写/读最多 16 字节；组合事务第一段最多 2 字节（典型是寄存器
*     地址），第二段最多 16 字节。
*
* core 的行为（drivers/i2c/i2c-core-base.c，本机无源码，按 master 核对，
* 6.18 未逐行核对）：
*   - num > max_num_msgs → -EOPNOTSUPP（“too many messages”）；
*   - 单条读/写 len > max_read_len/max_write_len → -EOPNOTSUPP；
*   - 有 COMB 标志时 max_num 强制为 2；只在 num==2 的组合事务里，
*     两条消息的 len 改由 max_comb_* 检查，普通 max_read_len/max_write_len
*     在这一批里不再检查（num==1 时仍按普通上限检查）；
*   - 第一段是读而声明了 WRITE_FIRST → -EOPNOTSUPP；
*   - 第二段不是读而声明了 READ_SECOND → -EOPNOTSUPP；
*   - 两段地址不同而声明了 SAME_ADDR → -EOPNOTSUPP；
*   - 读消息 len==0 且声明了 NO_ZERO_LEN_READ → -EOPNOTSUPP；
*   - 写消息 len==0 且声明了 NO_ZERO_LEN_WRITE → -EOPNOTSUPP。
* 注意 quirks 也会约束 core 自己模拟出来的 SMBus 事务：例如 SMBus word
* write 会变成“命令 + 2 字节”共 3 字节写，只要没超过 16 就放行；而 SMBus
* block write 可能到 33 字节，会被 max_write_len 直接拒掉，即使
* functionality 声明了 SMBUS_EMUL。用户态经 I2C_RDWR/I2C_SMBUS 提交时
* 看到的就是 ioctl 返回 -1、errno = EOPNOTSUPP。同理 SMBus process call
* 的第一段是 3 字节（命令 + 2 字节），超过 max_comb_1st_msg_len=2，也会被拒。 */
static const struct i2c_adapter_quirks my_i2c_quirks = {
	.flags = I2C_AQ_COMB_WRITE_THEN_READ | I2C_AQ_NO_ZERO_LEN_READ,
	.max_num_msgs = 2,
	.max_write_len = 16,
	.max_read_len = 16,
	.max_comb_1st_msg_len = 2,
	.max_comb_2nd_msg_len = 16,
};

static int my_i2c_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct my_i2c *priv;
	struct i2c_adapter *adap;
	int ret;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->regs = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(priv->regs))
		return PTR_ERR(priv->regs);

	/* 可选时钟：没有就返回 NULL，不是错误。真实驱动还要处理复位、
	* pinctrl、电源域等，这里省略以保持最小。 */
	priv->clk = devm_clk_get_optional_enabled(dev, NULL);
	if (IS_ERR(priv->clk))
		return dev_err_probe(dev, PTR_ERR(priv->clk), "failed to get clock\n");

	adap = &priv->adap;
	adap->owner = THIS_MODULE;
	adap->algo = &my_i2c_algo;
	/* 挂上限制表：core 之后每次 i2c_transfer 都会检查它。 */
	adap->quirks = &my_i2c_quirks;
	adap->retries = 2;
	adap->dev.parent = dev;
	/* 让 adapter 的设备节点指向 platform 设备，便于 sysfs/设备树关联。
	* 旧驱动写 adap->dev.of_node = pdev->dev.of_node; 新写法是下面这行。 */
	device_set_node(&adap->dev, dev_fwnode(dev));
	strscpy(adap->name, "my-i2c-quirks", sizeof(adap->name));
	i2c_set_adapdata(adap, priv);
	platform_set_drvdata(pdev, priv);

	/* i2c_add_adapter 动态分配总线号；成功时返回值就是总线号（>=0），
	* 失败返回负 errno。设备树里想固定编号可以改用 i2c_add_numbered_adapter
	* 并先设置 adap->nr。 */
	ret = i2c_add_adapter(adap);
	if (ret < 0)
		return dev_err_probe(dev, ret, "failed to add i2c adapter\n");

	priv->bus_num = ret;
	dev_info(dev, "registered i2c adapter %d\n", priv->bus_num);
	return 0;
}

/* 6.11 起 platform_driver.remove 的返回值改成 void，别照抄旧驱动的 int。 */
static void my_i2c_remove(struct platform_device *pdev)
{
	struct my_i2c *priv = platform_get_drvdata(pdev);

	i2c_del_adapter(&priv->adap);
}

static const struct of_device_id my_i2c_of_match[] = {
	{ .compatible = "example,my-i2c-quirks" },
	{ }
};
MODULE_DEVICE_TABLE(of, my_i2c_of_match);

static struct platform_driver my_i2c_driver = {
	.probe = my_i2c_probe,
	.remove = my_i2c_remove,
	.driver = {
		.name = "my-i2c-quirks",
		.of_match_table = my_i2c_of_match,
	},
};
module_platform_driver(my_i2c_driver);

MODULE_AUTHOR("ibus examples");
MODULE_DESCRIPTION("Minimal I2C adapter driver demonstrating struct i2c_adapter_quirks");
MODULE_LICENSE("GPL");
