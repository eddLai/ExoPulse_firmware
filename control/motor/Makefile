CC      = g++
CFLAGS  = -Wall -Wextra -O2 -fPIC -std=c++11 -I.. -I.
LDFLAGS = -shared

LIB     = libexo_motor.so
TEST    = test_motor

# pybind11
PYBIND11_INC = $(shell python3 -m pybind11 --includes)
PYTHON_INC   = $(shell python3-config --includes)
EXT_SUFFIX   = $(shell python3-config --extension-suffix)
PYBIND_LIB   = exo_motor$(EXT_SUFFIX)

# Source files
CAN_SRCS    = mcp2515.cpp ../mcp2515_can.cpp
MOTOR_SRCS  = lktech/lktech_motor.cpp motor_protection.cpp
ALL_SRCS    = $(CAN_SRCS) $(MOTOR_SRCS)
BIND_SRC    = bind_motor.cpp

# Header files (for dependency tracking)
HDRS = mcp2515.h motor_base.h motor_protection.h \
       lktech/lktech_motor.h lktech/lktech_protocol.h \
       ../can_bus.h ../mcp2515_can.h

.PHONY: all clean test pybind

all: $(PYBIND_LIB) $(TEST)

# pybind11 module (primary build target)
$(PYBIND_LIB): $(BIND_SRC) $(ALL_SRCS) $(HDRS)
	$(CC) $(CFLAGS) -shared $(PYBIND11_INC) $(PYTHON_INC) -o $@ $(BIND_SRC) $(ALL_SRCS) -lm -lrt

pybind: $(PYBIND_LIB)

# Legacy shared library (deprecated — use pybind11 module instead)
$(LIB): $(ALL_SRCS) $(HDRS)
	$(CC) $(CFLAGS) $(LDFLAGS) -o $@ $(ALL_SRCS) -lm

$(TEST): test_motor.cpp $(ALL_SRCS) $(HDRS)
	$(CC) $(CFLAGS) -o $@ test_motor.cpp $(ALL_SRCS) -lrt -lm

clean:
	rm -f $(LIB) $(TEST) $(PYBIND_LIB)

test: $(TEST)
	./$(TEST)
