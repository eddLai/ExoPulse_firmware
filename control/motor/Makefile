CC      = g++
CFLAGS  = -Wall -Wextra -O2 -fPIC -std=c++11
LDFLAGS = -shared

LIB     = libexo_motor.so
TEST    = test_motor

SRCS    = mcp2515.cpp lktech_motor.cpp
HDRS    = mcp2515.h lktech_motor.h

.PHONY: all clean test

all: $(LIB) $(TEST)

$(LIB): $(SRCS) $(HDRS)
	$(CC) $(CFLAGS) $(LDFLAGS) -o $@ $(SRCS) -lm

$(TEST): test_motor.cpp $(SRCS) $(HDRS)
	$(CC) $(CFLAGS) -o $@ test_motor.cpp $(SRCS) -lrt -lm

clean:
	rm -f $(LIB) $(TEST)

test: $(TEST)
	./$(TEST)
