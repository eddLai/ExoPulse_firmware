CC      = g++
CFLAGS  = -Wall -Wextra -O2 -fPIC -std=c++11
LDFLAGS = -shared

LIB     = libexo_motor.so
TEST    = test_motor

SRCS    = mcp2515.cpp lktech_motor.cpp motor_protection.cpp

.PHONY: all clean test

all: $(LIB) $(TEST)

$(LIB): $(SRCS) mcp2515.h lktech_motor.h motor_protection.h
	$(CC) $(CFLAGS) $(LDFLAGS) -o $@ $(SRCS) -lm

$(TEST): test_motor.cpp $(SRCS) mcp2515.h lktech_motor.h motor_protection.h
	$(CC) $(CFLAGS) -o $@ test_motor.cpp $(SRCS) -lrt -lm

clean:
	rm -f $(LIB) $(TEST)

test: $(TEST)
	./$(TEST)
