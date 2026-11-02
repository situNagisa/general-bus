// 不在本机编译：NXP i.MX RT 系列 MCU 的寄存器级代码（LPI2C 外设），需要 MCUXpresso SDK / CMSIS 头文件和 arm-none-eabi 交叉编译器，且在真实芯片上运行。
//
// 这是什么
// --------
// NXP LPI2C（Low Power I2C）主机的“命令 FIFO”用法：一次把整笔 I2C 事务
// （START+地址写 → 写寄存器地址 → repeated START+地址读 → 收 N 字节 →
// STOP）翻译成若干条 32 位命令字，依次写进 MTDR（Master Transmit Data
// Register）。硬件按 FIFO 顺序自动执行，中途不需要 CPU 干预。
//
// 这是驱动实现 link 的手段，link 层看不到它：上层只看到“读某个寄存器的
// N 个字节”，看不到 MTDR 命令字、命令 FIFO、START/STOP 命令、MSR 标志。
//
// 形态（一次调用交几段内存、对应几个线上单元、提交几次、完成几次）
// ----------------------------------------------------------------
//   - 一次“提交”= 往 MTDR 压入 5 条命令字（本例）：
//       1) CMD=100b START + 地址(写)
//       2) CMD=000b 发送 1 字节（寄存器地址）
//       3) CMD=100b repeated START + 地址(读)
//       4) CMD=001b 接收 N 字节（DATA 字段 = N-1）
//       5) CMD=010b 产生 STOP
//   - 线上单元：1 个 START、2 个地址字节、1 个写数据字节、N 个读数据字节、
//     1 个 repeated START、1 个 STOP。
//   - 完成：硬件不为每条命令单独报完成；驱动靠 MSR 的标志（EPF/SDF/NDF）
//     或对应中断知道进展。整笔事务完成看 SDF（STOP Detect）。
//   - 内存：命令和数据都走 MTDR / MRDR 这两个 32 位寄存器，没有 DMA 描述符
//     （要 DMA 时才另外配 MDER/MTDR 的 DMA 请求）。
//
// 为什么说它“动态”
// ----------------
// 命令条数不是固定的：有没有寄存器地址、读还是写、要不要 repeated START、
// 收多少字节，都会改变要压几条 MTDR；接收命令的 DATA 字段还编码了字节数。
// 所以它是一条由驱动按事务现场拼出来的“动态命令流”，而不是固定布局的
// 硬件描述符。对比 xHCI：TRB 也是 16 字节固定格式，但一个 TD 的 TRB 条数
// 随传输长度变化，同样属于动态描述符序列。
//
// 需要的条件：i.MX RT10xx/11xx（或其它带 LPI2C 的 NXP 芯片）、LPI2C 时钟
// 与引脚已配置、从设备地址已知（7 位）。
//
// 怎么编译/怎么运行：不能在本机编译。用 MCUXpresso SDK 建立 i.MX RT1050
// EVK 工程，把本文件加入工程，配置 LPI2C1 的引脚和时钟后调用
// demo::lpi2c::read_regs()；用逻辑分析仪看 SCL/SDA，预期看到
// “START, addr+W, reg, repeated START, addr+R, N 字节, STOP”，
// 函数返回 true，out 里是寄存器内容。
//

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>

#include "fsl_device_registers.h"	// 提供 LPI2C1 与 LPI2C_*_MASK 等 CMSIS 定义

namespace demo::lpi2c {

//
// MTDR 的布局（i.MX RT 参考手册 LPI2C 章）：
//   [7:0]   DATA：发送的数据 / 接收字节数减一 / 地址+读写位
//   [10:8]  CMD：命令编码
//   [31:11] 保留
//
// 命令编码（已对照 NXP 文档与 Linux i2c-imx-lpi2c.c 的 GEN_START 等定义）：
//   000b 发送 DATA[7:0]
//   001b 接收 (DATA[7:0] + 1) 字节
//   010b 产生 STOP
//   011b 接收并丢弃 (DATA[7:0] + 1) 字节
//   100b 产生 (repeated) START 并发送 DATA[7:0] 里的地址（含 R/W 位）
//   101b 同 100b，但本次传输期望从机回 NACK（NXP 文档原文；具体用途不展开）
//   110b 同 100b，高速模式
//   111b 同 101b，高速模式
//
enum class MtdrCmd : std::uint32_t {
	kTransmit		= 0b000u,
	kReceive		= 0b001u,
	kStop			= 0b010u,
	kReceiveDiscard		= 0b011u,
	kStartTransmit		= 0b100u,
	kStartTransmitNack	= 0b101u,
	kStartHighSpeed		= 0b110u,
	kStartHighSpeedNack	= 0b111u,
};

// 拼一条 MTDR 命令字：CMD 在 bit10:8，DATA 在 bit7:0。
// 注意 START 命令的 DATA 是 (7 位地址 << 1) | R/W，不是裸地址。
[[nodiscard]] constexpr std::uint32_t mtdr_word(MtdrCmd cmd,
						std::uint32_t data) noexcept
{
	return (static_cast<std::uint32_t>(cmd) << 8) | (data & 0xFFu);
}

//
// 等 TX FIFO 有空位再写 MTDR。
// MSR.TDF（Transmit Data Flag）表示“可以再写一条命令/数据”。
// i.MX RT 的 LPI2C TX FIFO 只有 4 级（未逐型号核对），要压 5 条命令时
// 必然会在中途等一次 TDF；先等再写是通用做法。
// 往满的 FIFO 写 MTDR 会丢命令（未核对参考手册对“满时写入”的确切规定），
// 表现为事务在中途停住或 NDF/FEF。
//
inline void wait_tx_space() noexcept
{
	while ((LPI2C1->MSR & LPI2C_MSR_TDF_MASK) == 0u) {
		// 忙等；正式驱动可以在这里 __WFI() 或改用 TDIE 中断
	}
}

//
// 把一笔“写寄存器地址后 repeated START 读 N 字节”的事务压进命令 FIFO。
// 这里一次性把 5 条命令都排好，硬件自己连续执行；CPU 只在 FIFO 满时
// 短暂等待 TDF。
//
inline void queue_read_transaction(std::uint8_t addr7, std::uint8_t reg,
				std::size_t n) noexcept
{
	// 1) START + 地址(写)：bit0=0 表示写。
	wait_tx_space();
	LPI2C1->MTDR = mtdr_word(MtdrCmd::kStartTransmit,
				(static_cast<std::uint32_t>(addr7) << 1) | 0u);

	// 2) 发送寄存器地址：CMD=0，DATA 在低 8 位，所以直接写字节即可。
	wait_tx_space();
	LPI2C1->MTDR = mtdr_word(MtdrCmd::kTransmit, reg);

	// 3) repeated START + 地址(读)：bit0=1。
	//    这里的 START 命令在总线上就是 repeated START（前面没有 STOP）。
	//    执行到这里时 MSR.EPF 会置位（主机产生了 repeated START）。
	wait_tx_space();
	LPI2C1->MTDR = mtdr_word(MtdrCmd::kStartTransmit,
				(static_cast<std::uint32_t>(addr7) << 1) | 1u);

	// 4) 接收 n 字节：DATA 字段是 n-1（命令 001b 表示收 DATA+1 字节）。
	//    接收命令的最后一个字节由主机自动回 NACK，这是 I2C 读的协议要求，
	//    不会置 NDF（NDF 只表示“非预期 NACK”）。LPI2C 命令 001b 是否
	//    明确保证自动 NACK 最后一个字节，未逐字核对参考手册。
	wait_tx_space();
	LPI2C1->MTDR = mtdr_word(MtdrCmd::kReceive,
				static_cast<std::uint32_t>(n - 1));

	// 5) STOP：执行后 SDF 置位，整笔事务结束。
	wait_tx_space();
	LPI2C1->MTDR = mtdr_word(MtdrCmd::kStop, 0);
}

//
// 初始化：使能主机、清 FIFO、关掉 AUTOSTOP。
//
// MCFGR1.AUTOSTOP 必须为 0：AUTOSTOP=1 时只要发送 FIFO 空、主机还忙，
// 硬件就自动发 STOP。我们的命令是分批压进去的（FIFO 只有 4 级），
// 一旦中途 FIFO 空就会被 AUTOSTOP 提前截断，repeated START 变成 STOP+START。
//
inline void init() noexcept
{
	LPI2C1->MCR = 0;			// 先关主机
	LPI2C1->MCR = LPI2C_MCR_RST_MASK;	// 软复位，自清除
	while ((LPI2C1->MCR & LPI2C_MCR_RST_MASK) != 0u) {
	}

	// 清 AUTOSTOP，保留 PRESCALE（这里不改，沿用板级初始化）。
	LPI2C1->MCFGR1 &= ~LPI2C_MCFGR1_AUTOSTOP_MASK;

	// 复位两个 FIFO：RTF=发送，RRF=接收。位自清除。
	LPI2C1->MCR |= LPI2C_MCR_RTF_MASK | LPI2C_MCR_RRF_MASK;

	// 本例用轮询；若改成中断，则在这里配 MIER：
	//   TDIE(bit0)  TX FIFO 有空位
	//   RDIE(bit1)  RX FIFO 有数据
	//   EPIE(bit8)  产生 STOP 或 repeated START
	//   SDIE(bit9)  产生 STOP（整笔完成）
	//   NDIE(bit10) 非预期 NACK
	LPI2C1->MIER = 0;

	LPI2C1->MCR |= LPI2C_MCR_MEN_MASK;	// 使能主机
}

//
// 执行“读寄存器”事务并收数据。
//
// 返回值：true 表示收到 n 字节且正常 STOP；false 表示 NACK/总线错误/短读。
//
// 标志位（MSR）：
//   EPF/SDF/NDF/ALF/FEF 是写 1 清零的 sticky 位；TDF/RDF 和忙标志只读。
//   TDF  bit0  发送 FIFO 有空位（可以写 MTDR）
//   RDF  bit1  接收 FIFO 有数据（可以读 MRDR）
//   EPF  bit8  主机产生了 STOP 或 repeated START —— 本例第 3 步
//              （repeated START）和第 5 步（STOP）各置一次
//   SDF  bit9  主机产生了 STOP —— 第 5 步置位，整笔事务完成
//   NDF  bit10 非预期 NACK —— 地址或写数据被从机 NACK，立即出错
//   ALF  bit11 仲裁丢失
//   FEF  bit12 FIFO 错误（例如没有 START 就发数据）
//   MBF  bit24 主机忙；BBF bit25 总线忙
//
// 注意：SDF 置位时 RX FIFO 里可能还有最后几个字节没读，所以循环里
// 先排空 RDF，再看 SDF。
//
[[nodiscard]] bool read_regs(std::uint8_t addr7, std::uint8_t reg,
			std::span<std::uint8_t> out) noexcept
{
	if (out.empty() || out.size() > 256) {
		return false;	// 接收命令的 DATA 只有 8 位，最多 256 字节
	}

	// 等总线空闲：MBF=主机忙，BBF=总线忙。上一次事务没收尾时不能开始。
	while ((LPI2C1->MSR & (LPI2C_MSR_MBF_MASK | LPI2C_MSR_BBF_MASK)) != 0u) {
	}

	// 清掉上一次留下的 sticky 标志，避免把旧 NDF/SDF 当成本次结果。
	LPI2C1->MSR = LPI2C_MSR_EPF_MASK | LPI2C_MSR_SDF_MASK |
		LPI2C_MSR_NDF_MASK | LPI2C_MSR_ALF_MASK |
		LPI2C_MSR_FEF_MASK;

	// 清空 FIFO，保证命令从干净状态开始。
	LPI2C1->MCR |= LPI2C_MCR_RTF_MASK | LPI2C_MCR_RRF_MASK;

	queue_read_transaction(addr7, reg, out.size());

	std::size_t got = 0;
	for (;;) {
		const std::uint32_t msr = LPI2C1->MSR;

		if ((msr & LPI2C_MSR_NDF_MASK) != 0u) {
			// 从机 NACK：地址不存在或从机忙。清标志、清 FIFO 收场。
			LPI2C1->MSR = LPI2C_MSR_NDF_MASK;
			LPI2C1->MCR |= LPI2C_MCR_RTF_MASK | LPI2C_MCR_RRF_MASK;
			return false;
		}

		if ((msr & (LPI2C_MSR_ALF_MASK | LPI2C_MSR_FEF_MASK)) != 0u) {
			// 仲裁丢失 / FIFO 错误：命令序列或总线出了问题。
			LPI2C1->MSR = LPI2C_MSR_ALF_MASK | LPI2C_MSR_FEF_MASK;
			LPI2C1->MCR |= LPI2C_MCR_RTF_MASK | LPI2C_MCR_RRF_MASK;
			return false;
		}

		if ((msr & LPI2C_MSR_RDF_MASK) != 0u) {
			// 读 MRDR 把数据移出 RX FIFO；先排空再看 STOP。
			const auto b = static_cast<std::uint8_t>(LPI2C1->MRDR & 0xFFu);
			if (got < out.size()) {
				out[got] = b;
				got = got + 1;
			}
			continue;
		}

		if ((msr & LPI2C_MSR_SDF_MASK) != 0u) {
			// STOP 已经出现在总线上，事务结束。
			LPI2C1->MSR = LPI2C_MSR_SDF_MASK;
			break;
		}
	}

	return got == out.size();
}

}	// namespace demo::lpi2c

//
// 使用示例（放在板级初始化之后）：
//
//   demo::lpi2c::init();
//   std::array<std::uint8_t, 1> who{};
//   if (demo::lpi2c::read_regs(0x68, 0x75, who)) {
//           // who[0] 现在是 WHO_AM_I
//   }
//
// 常见坑：
//   1. AUTOSTOP 忘记清零：命令 FIFO 一空就被硬件自动 STOP。
//   2. 接收命令写成 CMD=1 但 DATA 填 N 而不是 N-1，会多收一个字节。
//   3. 写满 FIFO 不等 TDF：命令丢失，事务卡住。
//   4. 用 START 命令时 DATA 填了裸地址，忘了左移并或上 R/W 位。
//   5. 收 N>4 字节时读 MRDR 太慢，RX FIFO 满后主机会停住总线等软件
//      （未核对 LPI2C 对 RX FIFO 满时的确切行为）；轮询循环要足够快，
//      或改用 RDIE 中断/DMA。
//   6. NDF 是 sticky 位，不清会一直误判。
//
