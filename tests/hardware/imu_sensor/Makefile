# Makefile for MPU6050 IMU Reader on Jetson Orin

CXX = g++
CXXFLAGS = -Wall -std=c++11
TARGET = mpu6050_reader
SRC = mpu6050_reader.cpp

all: $(TARGET)

$(TARGET): $(SRC)
	$(CXX) $(CXXFLAGS) -o $(TARGET) $(SRC)

clean:
	rm -f $(TARGET)

run: $(TARGET)
	sudo ./$(TARGET)

.PHONY: all clean run
