CC      = g++
CFLAGS  = -Wall -Wextra -O2 -fPIC -std=c++11
LDFLAGS = -shared

LIB     = libexo_imu.so
TEST    = test_mpu6050

# pybind11
PYBIND11_INC = $(shell python3 -m pybind11 --includes)
PYTHON_INC   = $(shell python3-config --includes)
EXT_SUFFIX   = $(shell python3-config --extension-suffix)
PYBIND_LIB   = exo_imu$(EXT_SUFFIX)

BIND_SRC = bind_imu.cpp

.PHONY: all clean test pybind

all: $(PYBIND_LIB) $(TEST)

# pybind11 module (primary build target)
$(PYBIND_LIB): $(BIND_SRC) mpu6050.cpp mpu6050.h
	$(CC) $(CFLAGS) -shared $(PYBIND11_INC) $(PYTHON_INC) -o $@ $(BIND_SRC) mpu6050.cpp -lm -lrt

pybind: $(PYBIND_LIB)

# Legacy shared library (deprecated — use pybind11 module instead)
$(LIB): mpu6050.cpp mpu6050.h
	$(CC) $(CFLAGS) $(LDFLAGS) -o $@ mpu6050.cpp -lm

$(TEST): test_mpu6050.cpp mpu6050.cpp mpu6050.h
	$(CC) $(CFLAGS) -o $@ test_mpu6050.cpp mpu6050.cpp -lrt -lm

clean:
	rm -f $(LIB) $(TEST) $(PYBIND_LIB)

test: $(TEST)
	./$(TEST)
