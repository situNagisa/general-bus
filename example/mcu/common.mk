# STM32F4（Cortex-M4F）交叉编译。只编译、不链接上板：外设寄存器、HAL、FreeRTOS 的实现不在这里。
# 依赖的头文件（CMSIS Core、cmsis-device-f4、FreeRTOS-Kernel）放在 VENDOR 下，由用户准备。
VENDOR ?= $(HOME)/.cache/ibus-vendor
CXX = arm-none-eabi-g++
SIZE = arm-none-eabi-size
CXXFLAGS = -std=c++26 -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -Os -DNDEBUG -DSTM32F407xx \
	-fno-exceptions -fno-rtti -ffunction-sections -fdata-sections -Wall -Wextra \
	-I../../../include -I$(HOME)/stdexec/include \
	-I$(VENDOR)/CMSIS_5/CMSIS/Core/Include -I$(VENDOR)/cmsis-device-f4/Include $(EXTRA_INC)

all: main.o

%.o: %.cpp $(wildcard ../../../include/ibus/stm32f4/*.hpp)
	$(CXX) $(CXXFLAGS) -c $< -o $@

# make size：和同目录下同场景的手写版本（hand.cpp）比 .text 总大小（含冷路径）。
size: main.o $(if $(wildcard hand.cpp),hand.o)
	@for o in $^; do printf '%-8s %s\n' $$o $$($(SIZE) -A $$o | awk '/^\.text/ {s += $$2} END {print s}'); done

clean:
	rm -f main.o hand.o

.PHONY: all size clean
