// 不在本机编译：STM32 MCU 寄存器级代码（I2C v2 外设），需要 CMSIS 设备头文件和 arm-none-eabi 交叉编译器，且在真实芯片上运行。
//
// 这是什么
// --------
// STM32 的 I2C v2 外设（F0/F3/F7/L4/H7 等使用的那一代）如何把一次
// “写寄存器地址，然后 repeated START 读 N 字节”拆成两次硬件提交：
//
//   第一次提交（写阶段）
//     CR2 = SADD | RD_WRN=0 | NBYTES=1 | START=1 | AUTOEND=0
//     等 TXIS 后写 TXDR（要写的寄存器地址）；
//     NBYTES 个字节传完，TC=1，硬件把 SCL 拉低（时钟拉伸）并产生中断。
//     —— 写阶段的结束是单独上报的，驱动必须处理 TC，否则总线一直停住。
//
//   第二次提交（读阶段，在 TC 中断里做）
//     CR2 = SADD | RD_WRN=1 | NBYTES=N | START=1 | AUTOEND=1
//     START=1 在总线上是 repeated START，同时清掉 TC；
//     每来一个字节 RXNE=1，读 RXDR；
//     第 N 个字节后硬件自动回 NACK 并产生 STOP，STOPF=1（STOPF 在总线
//     检测到 STOP 且外设参与传输时置位；主模式 AUTOEND=1 这一条的原文
//     措辞未逐字核对）。
//     注意 AUTOEND=1 时 TC 不会置位（TC 的置位条件是 RELOAD=0 且
//     AUTOEND=0），第二阶段完成看 STOPF。
//
// 这是驱动实现 link 的手段，link 层看不到它：上层只看到“读一个寄存器”，
// 看不到 CR2 的两次编程、TC/STOPF 两次上报、SCL 被拉低的等待窗口。
//
// 形态（一次调用交几段内存、对应几个线上单元、提交几次、完成几次）
// ----------------------------------------------------------------
//   - 一次逻辑操作 = 2 次硬件提交（两次写 CR2），不是一次。
//   - 线上单元：START、地址+W、1 个写数据字节、repeated START、地址+R、
//     N 个读数据字节、STOP。
//   - 完成上报：2 次。写阶段 TC 一次（单独中断），读阶段 STOPF 一次。
//     上层的一次“完成”在驱动里是两段拼起来的。
//   - 内存：数据只经过 TXDR/RXDR 两个数据寄存器，没有 DMA 描述符；
//     要 DMA 时是另一套（CR1 TXDMAEN/RXDMAEN）。
//
// 需要的条件：I2C v2 外设、7 位从机地址、总线上有上拉、TIMINGR 按
// 总线时钟算好（本文件不猜时钟，timing 由调用者传入）。
//
// 怎么编译/怎么运行：不能在本机编译。用 STM32CubeL4（或对应系列）的
// CMSIS 头文件，arm-none-eabi-g++ -std=c++26 编译，烧到 Nucleo 板上；
// 接一个 I2C 从机（例如 WHO_AM_I 寄存器），调用 start_read()。
// 用逻辑分析仪预期看到：START, addr+W, reg, repeated START, addr+R,
// N 字节, STOP；TC 中断进一次，随后 STOPF 中断进一次；返回值 true。
//

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>

#include "stm32l4xx.h"	// CMSIS；F0/F3/F7/H7 换成对应头文件，I2C 寄存器名相同

namespace demo::i2c {

//
// 传输阶段。ISR 和主循环共享，单核 Cortex-M 上用 volatile 足够；
// 多核或带缓存一致性的场景要改成 std::atomic。
//
enum class Phase : std::uint8_t {
	kIdle,
	kWritePointer,	// 第一次提交：START + 地址写 + 1 字节寄存器地址
	kReadBody,	// 第二次提交：repeated START + 地址读 + N 字节
};

struct Transfer {
	volatile Phase		phase;
	volatile std::uint32_t	sadd;		// 7 位地址左移 1 位（SADD[7:1]）
	volatile std::uint8_t	reg;		// 要写的寄存器地址
	std::uint8_t		*buf;		// 读数据去处（主循环提供，ISR 填）
	volatile std::size_t	len;
	volatile std::size_t	idx;
	volatile bool		done;		// 收到 STOPF
	volatile bool		error;		// NACK / 总线错误
};

inline volatile Transfer g_xfer = {};

//
// 初始化。TIMINGR 必须按 I2C 内核时钟和目标速率计算（CubeMX 会给出），
// 不要照抄某个固定值。
//
inline void init(std::uint32_t timingr) noexcept
{
	I2C1->CR1 &= ~I2C_CR1_PE;	// 改 TIMINGR 前先关外设
	I2C1->TIMINGR = timingr;
	I2C1->CR1 |= I2C_CR1_PE;

	NVIC_EnableIRQ(I2C1_EV_IRQn);	// 事件中断：TXIS/RXNE/TC/STOPF/NACKF
	NVIC_EnableIRQ(I2C1_ER_IRQn);	// 错误中断：BERR/ARLO/OVR
}

//
// 发起读寄存器操作。
//
// CR2 字段（CMSIS 位名）：
//   SADD[9:0]    7 位地址时 SADD[7:1]=地址，SADD[0] 不用
//   RD_WRN[10]   0=写，1=读
//   START[13]    1=产生 START 或 repeated START；写 1 同时清 TC
//   STOP[14]     1=产生 STOP（软件结束模式下手动收尾）
//   NACK[15]     1=当前字节回 NACK
//   NBYTES[23:16] 本阶段要传的数据字节数（不含地址）
//   RELOAD[24]   1=允许 NBYTES 重装（>255 字节时分段）
//   AUTOEND[25]  1=NBYTES 传完自动发 STOP；0=TC 置位并拉低 SCL 等软件
//
// 第一次提交故意 AUTOEND=0：写完寄存器地址后硬件停在 TC，等软件决定
// 下一步（这里就是发 repeated START 读）。
//
[[nodiscard]] bool start_read(std::uint8_t addr7, std::uint8_t reg,
			std::span<std::uint8_t> buf) noexcept
{
	if (buf.empty() || buf.size() > 255) {
		return false;	// NBYTES 只有 8 位；更长的读要 RELOAD 分段
	}

	// 总线忙（BUSY=1）时不能写 START。
	while ((I2C1->ISR & I2C_ISR_BUSY) != 0u) {
	}

	g_xfer.phase = Phase::kWritePointer;
	g_xfer.sadd = static_cast<std::uint32_t>(addr7) << 1;
	g_xfer.reg = reg;
	g_xfer.buf = buf.data();
	g_xfer.len = buf.size();
	g_xfer.idx = 0;
	g_xfer.done = false;
	g_xfer.error = false;

	// 清掉上次遗留的标志，避免旧 NACKF/STOPF 干扰。
	I2C1->ICR = I2C_ICR_STOPCF | I2C_ICR_NACKCF |
		I2C_ICR_BERRCF | I2C_ICR_ARLOCF | I2C_ICR_OVRCF;

	// 第一次提交：START + 地址(写) + NBYTES=1 + AUTOEND=0。
	// 这里写 CR2 会把 TC 清掉（如果上次留下了 TC）。
	I2C1->CR2 = g_xfer.sadd |
		(1u << I2C_CR2_NBYTES_Pos) |
		I2C_CR2_START;

	// 中断使能放在写 CR2 之后：上面的 CR2 写已经清掉遗留 TC，
	// 不会一开 TCIE 就进一次旧中断，把读阶段提前排进去。
	// 只开本流程需要的中断：TXIS、RXNE、TC、STOPF、NACKF、错误。
	I2C1->CR1 |= I2C_CR1_TXIE | I2C_CR1_RXIE | I2C_CR1_TCIE |
		I2C_CR1_STOPIE | I2C_CR1_NACKIE | I2C_CR1_ERRIE;

	return true;
}

//
// 事件中断：I2C1_EV_IRQHandler。
// 处理顺序有讲究：先错误（NACKF），再结束（STOPF），再阶段切换（TC），
// 最后数据（RXNE/TXIS）。
//
extern "C" void I2C1_EV_IRQHandler(void)
{
	const std::uint32_t isr = I2C1->ISR;

	// ---- NACK：地址或写数据被从机拒绝 ----
	if ((isr & I2C_ISR_NACKF) != 0u) {
		I2C1->ICR = I2C_ICR_NACKCF;
		g_xfer.error = true;
		g_xfer.phase = Phase::kIdle;
		// 软件结束模式（第一阶段 AUTOEND=0）下硬件不会自己收尾，
		// 必须写 STOP 释放总线；读阶段 AUTOEND=1 时再写一次也无害。
		I2C1->CR2 |= I2C_CR2_STOP;
		return;
	}

	// ---- 第二阶段结束：自动 STOP ----
	if ((isr & I2C_ISR_STOPF) != 0u) {
		I2C1->ICR = I2C_ICR_STOPCF;
		g_xfer.done = true;
		g_xfer.phase = Phase::kIdle;
		return;
	}

	// ---- 第一阶段结束：TC ----
	// AUTOEND=0 且 NBYTES 传完时 TC=1，SCL 被拉低。TC 不能用 ICR 清，
	// 只能通过写 CR2 的 START 或 STOP 位清除——下面的 repeated START
	// 正好完成“清 TC + 发起第二阶段”两件事。
	if ((isr & I2C_ISR_TC) != 0u) {
		if (g_xfer.phase == Phase::kWritePointer) {
			g_xfer.phase = Phase::kReadBody;

			// 第二次提交：repeated START + 地址(读) + NBYTES=N
			// + AUTOEND=1。START=1 清 TC；AUTOEND=1 让硬件在
			// 第 N 个字节后自动 NACK + STOP。
			I2C1->CR2 = g_xfer.sadd | I2C_CR2_RD_WRN |
				(static_cast<std::uint32_t>(g_xfer.len)
					<< I2C_CR2_NBYTES_Pos) |
				I2C_CR2_START | I2C_CR2_AUTOEND;
		} else {
			// 不应该出现：读阶段 AUTOEND=1 不会置 TC。真出现了就
			// 写 STOP 清 TC 并收尾，避免 TCIE 反复进中断。
			I2C1->CR2 |= I2C_CR2_STOP;
		}
		return;
	}

	// ---- 读数据 ----
	if ((isr & I2C_ISR_RXNE) != 0u) {
		const auto b = static_cast<std::uint8_t>(I2C1->RXDR);
		if (g_xfer.idx < g_xfer.len) {
			g_xfer.buf[g_xfer.idx] = b;
			g_xfer.idx = g_xfer.idx + 1;
		}
		return;
	}

	// ---- 写数据：第一个数据字节（寄存器地址） ----
	if ((isr & I2C_ISR_TXIS) != 0u) {
		I2C1->TXDR = g_xfer.reg;
		return;
	}
}

//
// 错误中断：BERR（总线错误）、ARLO（仲裁丢失）、OVR（溢出）。
// 这些不是正常流程的一部分，一律停机收尾。
//
extern "C" void I2C1_ER_IRQHandler(void)
{
	const std::uint32_t isr = I2C1->ISR;

	if ((isr & I2C_ISR_BERR) != 0u) {
		I2C1->ICR = I2C_ICR_BERRCF;
	}
	if ((isr & I2C_ISR_ARLO) != 0u) {
		I2C1->ICR = I2C_ICR_ARLOCF;
	}
	if ((isr & I2C_ISR_OVR) != 0u) {
		I2C1->ICR = I2C_ICR_OVRCF;
	}

	g_xfer.error = true;
	g_xfer.phase = Phase::kIdle;
	I2C1->CR2 |= I2C_CR2_STOP;
}

//
// 等待整笔操作结束（演示用忙等；正式代码应进低功耗等待中断）。
// 返回 false 表示超时或出错。
//
[[nodiscard]] bool wait_done(std::uint32_t spin_limit) noexcept
{
	while (!g_xfer.done && !g_xfer.error) {
		if (--spin_limit == 0u) {
			return false;
		}
	}
	return g_xfer.done && !g_xfer.error && g_xfer.idx == g_xfer.len;
}

}	// namespace demo::i2c

//
// 使用示例（板级时钟/引脚初始化之后）：
//
//   demo::i2c::init(timingr);	// timingr 由 CubeMX 按内核时钟算出
//   std::array<std::uint8_t, 1> who{};
//   if (demo::i2c::start_read(0x68, 0x75, who) &&
//       demo::i2c::wait_done(1000000u)) {
//           // who[0] 是 WHO_AM_I
//   }
//
// 常见坑：
//   1. 第一阶段 AUTOEND=0 却不在 TC 中断里继续：SCL 一直被拉低，总线死锁。
//      除了写 repeated START，也可以在 TC 里写 STOP 结束。
//   2. 第二阶段 AUTOEND=1 却等 TC：TC 的置位条件是 RELOAD=0 且
//      AUTOEND=0，等 TC 会永远等不到；第二阶段结束标志是 STOPF。
//   3. 用 ICR 清 TC：TC 只能由写 CR2 的 START/STOP 位清除。
//   4. SADD 里又左移又或 R/W 位：7 位地址放 SADD[7:1]，方向放 RD_WRN。
//   5. NBYTES 超过 255 不用 RELOAD：一次提交最多 255 字节。
//   6. NACKF 后不写 STOP：第一阶段总线停在拉伸状态，后续操作全失败。
//   7. 读阶段 RXNE 处理太慢：OVR 置位、数据丢失；高吞吐用 DMA。
//   8. 本例假设寄存器地址是 1 字节；16 位地址要把第一阶段 NBYTES 改成 2，
//      在 TXIS 里按高字节、低字节写两次 TXDR，TC 仍在两字节都发完后置位。
//
