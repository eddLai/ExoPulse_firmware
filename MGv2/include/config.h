#pragma once

/*
 * Hardware and Motor Configuration
 * All system-wide constants and settings
 */

// MCP2515 SPI Configuration
constexpr int CAN_CS   = 5;
constexpr int SPI_SCK  = 18;
constexpr int SPI_MISO = 19;
constexpr int SPI_MOSI = 23;

// LED Pin
constexpr int LED_PIN = 2;

// Motor IDs
constexpr uint8_t MOTOR_ID_1 = 1;
constexpr uint8_t MOTOR_ID_2 = 2;

// CAN IDs (0x140 + Motor ID)
const uint32_t CAN_ID_1 = 0x140 + MOTOR_ID_1;
const uint32_t CAN_ID_2 = 0x140 + MOTOR_ID_2;

// Update rate configuration
constexpr uint32_t UPDATE_INTERVAL_MS = 100;  // 10Hz update rate
