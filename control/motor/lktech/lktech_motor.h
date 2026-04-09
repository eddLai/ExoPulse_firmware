/*
 * LK-TECH Motor Implementation
 * ExoPulse Firmware Layer
 *
 * Motor: LK-TECH MG/MF/MS/MH series, 18-bit encoder, CAN protocol
 * CAN IDs: 0x140 + motor_id (e.g., 0x141, 0x142)
 * Baud rate: 1 Mbps
 *
 * All CAN communication protocol details are encapsulated here.
 * Upper layers only use the Motor base interface.
 *
 * Refactored from:
 *   - control/motor/lktech_motor.h/.cpp (old C API)
 *   - MGv2/include/motor_protocol.h, motor_operations.h, motor_control.h
 */

#ifndef LKTECH_MOTOR_H
#define LKTECH_MOTOR_H

#include "../motor_base.h"
#include <cstdint>
#include <time.h>

class LktechMotor : public Motor {
public:
    LktechMotor(CanBus& can, uint16_t can_id, const char* name);
    ~LktechMotor() override = default;

    /* ================================================================
     * Motor base interface (override)
     * ================================================================ */

    int init() override;
    int setTorque(int16_t iq, MotorStatus* status_out = nullptr) override;
    int setPosition(int32_t angle_001deg, uint16_t max_speed,
                    MotorStatus* status_out = nullptr) override;
    int readStatus(MotorStatus* out) override;
    int stop() override;
    int shutdown() override;
    int calibrateZero() override;

    int16_t torqueLimit() const override;
    bool hasError(const MotorStatus& status) const override;

    /* ================================================================
     * LK-TECH specific commands (only available on this subclass)
     * ================================================================ */

    /*
     * Position control, single-turn with direction (command 0xA6).
     * angle_001deg: 0~35999 (0.01 deg/LSB, represents 0~359.99 deg)
     * direction: 0x00 = CW, 0x01 = CCW
     * max_speed: 1 dps/LSB (0 = no limit)
     */
    int setPositionSingle(uint16_t angle_001deg, uint8_t direction,
                          uint16_t max_speed,
                          MotorStatus* status_out = nullptr);

    /*
     * Speed closed-loop control (command 0xA2).
     * speed_001dps: target speed in 0.01 dps/LSB (int32_t)
     */
    int setSpeed(int32_t speed_001dps, MotorStatus* status_out = nullptr);

    /*
     * Write encoder offset to ROM (command 0x91, PERMANENT).
     * encoder_offset: 0~65535 (lower 16 bits of 18-bit encoder)
     */
    int writeEncoderOffset(uint16_t encoder_offset);

    /*
     * Set current position as zero in ROM (command 0x19, PERMANENT).
     * Requires MCU reboot to take effect.
     */
    int setZeroToROM();

    /*
     * Read PID parameters (command 0x30).
     * Fills angle_kp/ki, speed_kp/ki, torque_kp/ki.
     */
    int readPID(uint8_t* angle_kp, uint8_t* angle_ki,
                uint8_t* speed_kp, uint8_t* speed_ki,
                uint8_t* torque_kp, uint8_t* torque_ki);

    /*
     * Read raw encoder value (command 0x90).
     */
    int readEncoder(uint32_t* encoder_raw);

private:
    /* LK-TECH specific: send 8-byte command, wait for matching echo */
    int sendAndRecv(uint8_t* tx_data, uint8_t* rx_data);

    /* Shortcut: send command byte + 7 zeros, wait for response */
    int sendCmd(uint8_t cmd, uint8_t* rx_data);

    /* Parse common control response (0xA1/0xA4/0xA6): temp, iq, speed, encoder */
    static void parseControlResponse(const uint8_t* rx, MotorStatus* s);

    /* Acceleration calculation state */
    int16_t  last_speed_;
    struct timespec last_speed_time_;
    bool     has_last_speed_;
};

#endif /* LKTECH_MOTOR_H */
