CXX      ?= g++
STDEXEC  ?= $(HOME)/stdexec/include
CXXFLAGS ?= -std=c++26 -O3 -march=native -Wall -Wextra
CPPFLAGS += -I../../include -I$(STDEXEC)
LDLIBS   += 

ioctl-i2c: main.cpp $(wildcard ../../include/ibus/*.hpp)
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) main.cpp -o $@ $(LDLIBS)

clean:
	rm -f ioctl-i2c

.PHONY: clean
