# Makefile for ADXL345 IMU Reader on Jetson Orin

CXX = g++
CXXFLAGS = -Wall -std=c++11
TARGET = adxl345_reader
SRC = adxl345_reader.cpp

all: $(TARGET)

$(TARGET): $(SRC)
	$(CXX) $(CXXFLAGS) -o $(TARGET) $(SRC)

clean:
	rm -f $(TARGET)

run: $(TARGET)
	sudo ./$(TARGET)

.PHONY: all clean run
