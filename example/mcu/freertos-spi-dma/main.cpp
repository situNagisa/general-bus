// FreeRTOS：一个任务周期性地用 SPI DMA 读传感器，等待时任务阻塞、让出 CPU。
// 任务里没有 sync_wait（它依赖 std::mutex / condition_variable），用任务通知代替：
//   receiver 在中断里拿到结果后 vTaskNotifyGiveFromISR 唤醒任务，任务用 ulTaskNotifyTake 等，超时由用户处理。
#include <ibus/stm32f4/spi.hpp>

#include <FreeRTOS.h>
#include <task.h>

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>

namespace spi = ibus::stm32f4::spi;

static spi::completion spi2_done;

extern "C" void DMA1_Stream3_IRQHandler() {
	std::uint32_t f = (DMA1->LISR >> 22) & 0x3Du;
	DMA1->LIFCR = f << 22;
	if (f & (spi::tcif | spi::teif | spi::dmeif)) spi::complete(spi2_done, f);
}

// 等结果的 receiver：结果写到任务栈上的 error 里（空 = 成功），然后通知任务。在中断里调用。
struct notify_receiver {
	using receiver_concept = stdexec::receiver_t;
	TaskHandle_t task;
	std::optional<std::uint32_t>* error;

	void wake() noexcept {
		BaseType_t woken = pdFALSE;
		vTaskNotifyGiveFromISR(task, &woken);
		portYIELD_FROM_ISR(woken);
	}
	void set_value() && noexcept { wake(); }
	void set_error(std::uint32_t flags) && noexcept {
		*error = flags;
		wake();
	}
};
static_assert(spi::transfer<spi::dma_port, notify_receiver>);

// 在任务里等一个 sender 完成：返回 false 表示超时（此时 DMA 可能还在跑，op state 不能销毁，这里直接停下）。
static bool wait(auto&& sender, std::optional<std::uint32_t>& error, TickType_t timeout) noexcept {
	auto op = stdexec::connect(static_cast<decltype(sender)&&>(sender), notify_receiver{xTaskGetCurrentTaskHandle(), &error});
	stdexec::start(op);
	if (ulTaskNotifyTake(pdTRUE, timeout) == 0) {
		for (;;) {}  // 超时：真实代码里要先停 DMA（清 EN、等 EN 变 0）再返回
	}
	return true;
}

static std::array<std::byte, 6> cmd{std::byte{0x80 | 0x28}};
static std::array<std::byte, 6> rx{};

static void sensor_task(void*) {
	spi::dma_port imu{SPI2, DMA1_Stream3, DMA1_Stream4, 0, 0, &spi2_done};
	for (;;) {
		std::optional<std::uint32_t> error;
		wait(communicate(imu, transceive_pack(imu, std::span<std::byte const>(cmd), std::span(rx))), error, pdMS_TO_TICKS(10));
		if (error) {
			// DMA 出错：*error 是流的中断标志
		}
		vTaskDelay(pdMS_TO_TICKS(100));
	}
}

int main() {
	// 时钟、引脚、SPI、NVIC（中断优先级要低于 configMAX_SYSCALL_INTERRUPT_PRIORITY）由用户配置。
	xTaskCreate(sensor_task, "sensor", 256, nullptr, 2, nullptr);
	vTaskStartScheduler();
	for (;;) {}
}
