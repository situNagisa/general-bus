# 只构建能在 Linux 宿主机上编译的示例；driver/、windows/、spi/spi_mem.c、spi/quadspi_stm32.cpp、
# i2c/i2c_adapter_quirks.c 是内核 / MCU / Windows 代码，不在这里构建（见各文件头注释）。
CXX      ?= g++
CXXFLAGS ?= -std=c++26 -O2 -Wall -Wextra
OUT      ?= build

SRCS := $(wildcard socket/*.cpp io_uring/*.cpp rdma/*.cpp af_xdp/*.cpp usb/*.cpp i2c/*.cpp) \
	spi/spidev_read_write.cpp spi/spi_ioc_message.cpp spi/spidev_cs_change.cpp spi/sd_card_spi.cpp
BINS := $(patsubst %.cpp,$(OUT)/%,$(SRCS))

all: $(BINS)

$(OUT)/io_uring/%: io_uring/%.cpp
	@mkdir -p $(dir $@)
	$(CXX) $(CXXFLAGS) $< -o $@ -luring

$(OUT)/rdma/%: rdma/%.cpp
	@mkdir -p $(dir $@)
	$(CXX) $(CXXFLAGS) $< -o $@ -libverbs

$(OUT)/%: %.cpp
	@mkdir -p $(dir $@)
	$(CXX) $(CXXFLAGS) $< -o $@ -pthread

clean:
	rm -rf $(OUT)

.PHONY: all clean
