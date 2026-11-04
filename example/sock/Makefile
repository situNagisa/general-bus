CXX      ?= g++
STDEXEC  ?= $(HOME)/stdexec/include
CXXFLAGS ?= -std=c++26 -O3 -march=native -Wall -Wextra
NAGISA_ROOT ?= $(HOME)/nagisa
CPPFLAGS += -I../../include -I$(STDEXEC) -I$(NAGISA_ROOT)/include
LDLIBS   += -lstdc++exp  # nagisa 的 contract_assert 在 GCC 下需要

sock: main.cpp $(wildcard ../../include/ibus/*.hpp)
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) main.cpp -o $@ $(LDLIBS)

clean:
	rm -f sock

.PHONY: clean
