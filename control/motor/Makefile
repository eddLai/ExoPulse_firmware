CC      = g++
CFLAGS  = -Wall -Wextra -O2 -fPIC -std=c++11 -I.. -I.
LDFLAGS = -shared

LIB     = libexo_motor.so
TEST    = test_motor

# Source files
CAN_SRCS    = mcp2515.cpp ../mcp2515_can.cpp
MOTOR_SRCS  = lktech/lktech_motor.cpp motor_protection.cpp
ALL_SRCS    = $(CAN_SRCS) $(MOTOR_SRCS)

# Header files (for dependency tracking)
HDRS = mcp2515.h motor_base.h motor_protection.h \
       lktech/lktech_motor.h lktech/lktech_protocol.h \
       ../can_bus.h ../mcp2515_can.h

.PHONY: all clean test

all: $(LIB) $(TEST)

$(LIB): $(ALL_SRCS) $(HDRS)
	$(CC) $(CFLAGS) $(LDFLAGS) -o $@ $(ALL_SRCS) -lm

$(TEST): test_motor.cpp $(ALL_SRCS) $(HDRS)
	$(CC) $(CFLAGS) -o $@ test_motor.cpp $(ALL_SRCS) -lrt -lm

clean:
	rm -f $(LIB) $(TEST)

test: $(TEST)
	./$(TEST)
