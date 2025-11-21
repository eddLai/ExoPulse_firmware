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
    READ_MULTI_ANGLE        = 0x92,  // Multi-turn angle (int64_t, no wrap)
    READ_SINGLE_ANGLE       = 0x94,  // Single-turn angle (uint32_t, wraps at 360°)
    READ_STATUS_1_ERROR     = 0x9A,
    READ_STATUS_2           = 0x9C,
    READ_STATUS_3           = 0x9D,
};

// Write/Control commands
enum MotorWriteCommand : uint8_t {
    MOTOR_SHUTDOWN          = 0x80,  // Shut down motor, clear running state
    MOTOR_STOP              = 0x81,  // Stop motor (keep motor enabled)
    MOTOR_RUN               = 0x88,  // Motor run command
    WRITE_ENCODER_OFFSET    = 0x91,  // Write encoder offset to ROM as zero point
};

// Position/Motion control commands
enum MotorControlCommand : uint8_t {
    TORQUE_OPEN_LOOP        = 0xA0,  // Torque open-loop control
    TORQUE_CLOSED_LOOP      = 0xA1,  // Torque closed-loop control
    SPEED_CLOSED_LOOP       = 0xA2,  // Speed closed-loop control
    POS_CTRL_MULTI_1        = 0xA3,  // Position control (multi-turn) mode 1
    POS_CTRL_MULTI_2        = 0xA4,  // Position control (multi-turn) mode 2
    POS_CTRL_SINGLE_1       = 0xA5,  // Position control (single-turn) mode 1
    POS_CTRL_SINGLE_2       = 0xA6,  // Position control (single-turn) mode 2 - with direction
    POS_CTRL_INCREMENT_1    = 0xA7,  // Position control (increment) mode 1
    POS_CTRL_INCREMENT_2    = 0xA8,  // Position control (increment) mode 2
};

// Spin direction for position control commands
enum SpinDirection : uint8_t {
    SPIN_CLOCKWISE          = 0x00,  // Clockwise rotation
    SPIN_COUNTER_CLOCKWISE  = 0x01,  // Counter-clockwise rotation
};

// Error state flags (from 0x9A response byte 7)
enum MotorErrorFlag : uint8_t {
    ERROR_NONE              = 0x00,
    ERROR_LOW_VOLTAGE       = 0x01,  // Bit 0: Low voltage protection
    ERROR_OVER_TEMP         = 0x08,  // Bit 3: Over-temperature protection
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
    uint32_t encoder;        // 0~262143 (18-bit)
    int64_t motorAngle;      // 0.01°/LSB (single-turn angle, 0-360°×gear_ratio)
    uint32_t timestamp;      // millis() when read
};
