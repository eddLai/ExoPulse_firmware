#pragma once
#include <cstdint>

/*
 * LK-TECH Motor Protocol Definitions
 * Command codes and motor status data structure
 */

// Read-only commands (no motor control)
enum MotorReadCommand : uint8_t {
    READ_PID_PARAMS         = 0x30,
    READ_ACCELERATION       = 0x33,
    READ_ENCODER            = 0x90,
    READ_MULTI_ANGLE        = 0x92,
    READ_SINGLE_ANGLE       = 0x94,
    READ_STATUS_1_ERROR     = 0x9A,
    READ_STATUS_2           = 0x9C,
    READ_STATUS_3           = 0x9D,
};

// Motor status data structure
struct MotorStatus {
    uint8_t motorID;         // Motor ID (1 or 2)
    int8_t temperature;      // °C
    uint16_t voltage;        // 0.1V/LSB
    uint8_t errorState;      // Error flags
    int16_t torqueCurrent;   // iq: -2048~2048 → -33A~33A
    int16_t speed;           // dps (degrees per second)
    int32_t acceleration;    // dps/s (degrees per second squared) - INT32!
    uint16_t encoder;        // 0~16383 (14-bit)
    int64_t motorAngle;      // 0.01°/LSB (single-turn angle, 0-360°×gear_ratio)
    uint32_t timestamp;      // millis() when read
};
