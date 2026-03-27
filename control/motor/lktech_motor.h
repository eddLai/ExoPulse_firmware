/*
 * LK-TECH Motor Control Module Interface
 * Controls MGv2 motors via MCP2515 CAN bus
 *
 * Motor: LK-TECH MG series, 18-bit encoder, CAN protocol
 * CAN IDs: 0x141 (Motor 1), 0x142 (Motor 2)
 *
 * Refactored from MGv2/include/motor_protocol.h, motor_control.h, motor_operations.h
 * Ported from ESP32/FreeRTOS to Jetson Orin Linux
 */

#ifndef LKTECH_MOTOR_H
#define LKTECH_MOTOR_H

#include "mcp2515.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    int8_t temperature;        /* Celsius */
    int16_t torque_current;    /* iq: -2048~2048 maps to -33A~33A */
    int16_t speed;             /* deg/s */
    int32_t acceleration;      /* deg/s^2 (computed from speed delta) */
    int64_t angle;             /* 0.01 deg/LSB, multi-turn */
    uint32_t encoder;          /* 18-bit: 0~262143 */
    uint16_t voltage;          /* 0.1V/LSB */
    uint8_t error_state;       /* Error flags (bit0: low voltage, bit3: over-temp) */
} motor_status_t;

/* Opaque handle */
typedef struct lktech_motor lktech_motor_t;

/*
 * Create motor instance on a CAN bus.
 * can: MCP2515 handle (must be already initialized)
 * can_id: Motor CAN ID (0x141 or 0x142)
 * Returns NULL on failure.
 */
lktech_motor_t* motor_create(mcp2515_t* can, uint16_t can_id);

/*
 * Initialize motor: verify communication by reading status.
 * Returns 0 on success, -1 on failure.
 */
int motor_init(lktech_motor_t* dev);

/*
 * Torque closed-loop control (command 0xA1).
 * iq: -2000~2000, clamped to +/-800 for safety (~12.8A)
 * status_out: optional, filled with motor response. Pass NULL to ignore.
 * Returns 0 on success, -1 on failure.
 */
int motor_set_torque(lktech_motor_t* dev, int16_t iq, motor_status_t* status_out);

/*
 * Read motor status (0x9A + 0x9C + 0x92).
 * Fills: temperature, voltage, error_state, torque_current, speed, encoder, angle.
 * Returns 0 on success, -1 on failure (partial reads may still fill some fields).
 */
int motor_read_status(lktech_motor_t* dev, motor_status_t* out);

/*
 * Position closed-loop control, multi-turn (command 0xA4).
 * Matches MGv2 sendPositionControl_A4 protocol.
 * angle_001deg: target angle in 0.01 deg units (int32_t), e.g. 36000 = 360°
 * max_speed: maximum speed in dps (uint16_t), 0 = no limit.
 * status_out: optional, filled with motor response. Pass NULL to ignore.
 * Returns 0 on success, -1 on failure.
 */
int motor_set_position(lktech_motor_t* dev, int32_t angle_001deg,
                       uint16_t max_speed, motor_status_t* status_out);

/*
 * Stop motor (command 0x81). Motor stays enabled but stops moving.
 * Returns 0 on success, -1 on failure.
 */
int motor_stop(lktech_motor_t* dev);

/*
 * Shutdown motor (command 0x80). Clears running state.
 * Returns 0 on success, -1 on failure.
 */
int motor_shutdown(lktech_motor_t* dev);

/*
 * Destroy instance, free memory. Does NOT stop the motor — call motor_stop first.
 */
void motor_destroy(lktech_motor_t* dev);

#ifdef __cplusplus
}
#endif

#endif /* LKTECH_MOTOR_H */
