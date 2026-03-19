CC      = g++
CFLAGS  = -Wall -Wextra -O2 -fPIC -std=c++11
LDFLAGS = -shared

LIB     = libexo_imu.so
TEST    = test_mpu6050

.PHONY: all clean test

all: $(LIB) $(TEST)

$(LIB): mpu6050.cpp mpu6050.h
	$(CC) $(CFLAGS) $(LDFLAGS) -o $@ mpu6050.cpp -lm

$(TEST): test_mpu6050.cpp mpu6050.cpp mpu6050.h
	$(CC) $(CFLAGS) -o $@ test_mpu6050.cpp mpu6050.cpp -lrt -lm

clean:
	rm -f $(LIB) $(TEST)

test: $(TEST)
	./$(TEST)
