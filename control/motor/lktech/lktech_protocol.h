/*
 * LK-TECH Motor Protocol Definitions
 * CAN command codes for MG/MF/MS/MH series motors.
 *
 * Extracted from MGv2/include/motor_protocol.h (ESP32 version).
 * Used only inside LktechMotor — upper layers do NOT include this.
 */

#ifndef LKTECH_PROTOCOL_H
#define LKTECH_PROTOCOL_H

#include <cstdint>

namespace lktech {

/* Read-only commands */
constexpr uint8_t CMD_READ_PID_PARAMS       = 0x30;
constexpr uint8_t CMD_READ_ACCELERATION     = 0x33;
constexpr uint8_t CMD_READ_ENCODER          = 0x90;
constexpr uint8_t CMD_READ_MULTI_ANGLE      = 0x92;
constexpr uint8_t CMD_READ_SINGLE_ANGLE     = 0x94;
constexpr uint8_t CMD_READ_STATUS_1_ERROR   = 0x9A;
constexpr uint8_t CMD_READ_STATUS_2         = 0x9C;
constexpr uint8_t CMD_READ_STATUS_3         = 0x9D;

/* Write/Control commands */
constexpr uint8_t CMD_MOTOR_SHUTDOWN        = 0x80;
constexpr uint8_t CMD_MOTOR_STOP            = 0x81;
constexpr uint8_t CMD_MOTOR_RUN             = 0x88;
constexpr uint8_t CMD_WRITE_ENCODER_OFFSET  = 0x91;
constexpr uint8_t CMD_SET_ZERO_ROM          = 0x19;

/* Position/Motion control commands */
constexpr uint8_t CMD_TORQUE_OPEN_LOOP      = 0xA0;
constexpr uint8_t CMD_TORQUE_CLOSED_LOOP    = 0xA1;
constexpr uint8_t CMD_SPEED_CLOSED_LOOP     = 0xA2;
constexpr uint8_t CMD_POS_MULTI_1           = 0xA3;
constexpr uint8_t CMD_POS_MULTI_2           = 0xA4;
constexpr uint8_t CMD_POS_SINGLE_1          = 0xA5;
constexpr uint8_t CMD_POS_SINGLE_2          = 0xA6;
constexpr uint8_t CMD_POS_INCREMENT_1       = 0xA7;
constexpr uint8_t CMD_POS_INCREMENT_2       = 0xA8;

/* Spin direction for position control */
constexpr uint8_t SPIN_CW  = 0x00;
constexpr uint8_t SPIN_CCW = 0x01;

/* Error state flags (from 0x9A response byte 7) */
constexpr uint8_t ERROR_LOW_VOLTAGE = 0x01;
constexpr uint8_t ERROR_OVER_TEMP   = 0x08;

/* Safety limits */
constexpr int16_t TORQUE_IQ_LIMIT = 800;  /* +/-800 iq (~12.8A) */

/* CAN response timeout */
constexpr uint32_t RESPONSE_TIMEOUT_MS = 100;

} /* namespace lktech */

#endif /* LKTECH_PROTOCOL_H */
