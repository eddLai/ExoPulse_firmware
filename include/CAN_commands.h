#ifndef CAN_COMMANDS_H
#define CAN_COMMANDS_H

#include <stdint.h>

// Base CAN ID for single motor control
#define CAN_BASE_ID 0x140

// CAN Control Command Codes (Single Motor)

// Start/Stop Class
#define CAN_CMD_MOTOR_OFF               0x80
#define CAN_CMD_MOTOR_STOP              0x81
#define CAN_CMD_RESTART_DEVICE          0x76
#define CAN_CMD_BRAKE_RELEASE           0x77
#define CAN_CMD_BRAKE_LOCK              0x78

// Control Class
#define CAN_CMD_TORQUE_CONTROL          0xA1
#define CAN_CMD_SPEED_CONTROL           0xA2
#define CAN_CMD_ABS_POSITION            0xA4
#define CAN_CMD_SINGLE_TURN_POSITION    0xA6
#define CAN_CMD_INCREMENTAL_POSITION    0xA8

// Query Class
#define CAN_CMD_QUERY_STATUS1_ERROR         0x9A
#define CAN_CMD_QUERY_STATUS2               0x9C
#define CAN_CMD_QUERY_STATUS3               0x9D
#define CAN_CMD_QUERY_MULTI_TURN_POSITION   0x60
#define CAN_CMD_QUERY_RAW_MULTI_TURN        0x61
#define CAN_CMD_QUERY_ZERO_OFFSET_POSITION  0x62
#define CAN_CMD_QUERY_SINGLE_TURN_ENCODER   0x90
#define CAN_CMD_QUERY_MULTI_TURN_ANGLE      0x92
#define CAN_CMD_QUERY_SINGLE_TURN_ANGLE     0x94
#define CAN_CMD_QUERY_RUNNING_TIME          0xB1
#define CAN_CMD_QUERY_SOFTWARE_VERSION      0xB2

// PID Setting
#define CAN_CMD_READ_PID                0x30
#define CAN_CMD_WRITE_PID_RAM           0x31
#define CAN_CMD_WRITE_PID_ROM           0x32

// Acceleration/Deceleration Setting
#define CAN_CMD_READ_ACCELERATION           0x42
#define CAN_CMD_SET_ACCELERATION_DECELERATION 0x43

// Zero Point Setting
#define CAN_CMD_SET_ZERO_POINT_SPECIFIED    0x63
#define CAN_CMD_SET_ZERO_POINT_CURRENT      0x64

// System Settings
#define CAN_CMD_READ_RUNNING_MODE       0x70
#define CAN_CMD_READ_MOTOR_POWER        0x71
#define CAN_CMD_SET_PROTECTION_TIMEOUT  0xB3
#define CAN_CMD_SET_BAUD_RATE           0xB4
#define CAN_CMD_READ_MODEL_CODE         0xB5

// Acceleration/Deceleration Index (for 0x43)
#define ACCEL_DECEL_INDEX_POS_ACCEL     0x00
#define ACCEL_DECEL_INDEX_POS_DECEL     0x01
#define ACCEL_DECEL_INDEX_SPD_ACCEL     0x02
#define ACCEL_DECEL_INDEX_SPD_DECEL     0x03

// Struct for Servo Command
typedef struct {
  uint16_t can_id;  // e.g. 0x140 + ID
  uint8_t cmd;
  uint8_t data[7];
} ServoCommand;

#endif // CAN_COMMANDS_H
