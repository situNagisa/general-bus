CXX      ?= g++
STDEXEC  ?= $(HOME)/stdexec/include
CXXFLAGS ?= -std=c++26 -O3 -march=native -Wall -Wextra
CPPFLAGS += -I../../include -I$(STDEXEC)
LDLIBS   += -luring

io_uring: main.cpp $(wildcard ../../include/ibus/*.hpp)
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) main.cpp -o $@ $(LDLIBS)

clean:
	rm -f io_uring

.PHONY: clean
