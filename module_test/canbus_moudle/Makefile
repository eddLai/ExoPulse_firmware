# Makefile for MCP2515 CAN Bus Module Test on Jetson Orin

CXX = g++
CXXFLAGS = -Wall -std=c++11 -pthread
TARGET = mcp2515_canbus
SRC = mcp2515_canbus.cpp

all: $(TARGET)

$(TARGET): $(SRC)
	$(CXX) $(CXXFLAGS) -o $(TARGET) $(SRC)

clean:
	rm -f $(TARGET)

# Run in receive mode (default)
run: $(TARGET)
	./$(TARGET)

# Run loopback self-test (no CAN bus needed)
test: $(TARGET)
	./$(TARGET) -m loopback

# Run in send mode
send: $(TARGET)
	./$(TARGET) -m send

# Run at 500Kbps
run-500k: $(TARGET)
	./$(TARGET) -b 500000

.PHONY: all clean run test send run-500k
