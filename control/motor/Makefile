CC      = g++
CFLAGS  = -Wall -Wextra -O2 -fPIC -std=c++11
LDFLAGS = -shared

LIB     = libexo_motor.so
TEST    = test_motor

SRCS    = mcp2515.cpp lktech_motor.cpp

.PHONY: all clean test

all: $(LIB) $(TEST)

$(LIB): $(SRCS) mcp2515.h lktech_motor.h
	$(CC) $(CFLAGS) $(LDFLAGS) -o $@ $(SRCS)

$(TEST): test_motor.cpp $(SRCS) mcp2515.h lktech_motor.h
	$(CC) $(CFLAGS) -o $@ test_motor.cpp $(SRCS) -lrt

clean:
	rm -f $(LIB) $(TEST)

test: $(TEST)
	./$(TEST)
