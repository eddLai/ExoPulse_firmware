/*
 * LK-TECH Motor Implementation
 *
 * All CAN protocol details (frame format, command codes, response parsing,
 * timeout handling, byte-level packing) are encapsulated here.
 *
 * Refactored from control/motor/lktech_motor.cpp (old C API).
 */

#include "lktech_motor.h"
#include "lktech_protocol.h"

#include <cstdlib>
#include <cstring>
#include <cstdio>
#include <unistd.h>
#include <time.h>

/* ================================================================
 * Constructor
 * ================================================================ */

LktechMotor::LktechMotor(CanBus& can, uint16_t can_id, const char* name)
    : Motor(can, can_id, name)
    , last_speed_(0)
    , last_speed_time_{}
    , has_last_speed_(false)
{
}

/* ================================================================
 * Private helpers — LK-TECH specific CAN communication
 * ================================================================ */

int LktechMotor::sendAndRecv(uint8_t* tx_data, uint8_t* rx_data) {
    if (can_.send(can_id_, tx_data, 8) < 0) {
        return -1;
    }

    /* Poll for response matching our CAN ID and command byte.
     * LK-TECH protocol: motor echoes back with same command byte in data[0]. */
    uint16_t rx_id;
    uint8_t rx_buf[8];
    uint8_t rx_len;
    uint32_t elapsed = 0;

    while (elapsed < lktech::RESPONSE_TIMEOUT_MS) {
        if (can_.recv(&rx_id, rx_buf, &rx_len, 0) == 0) {
            if (rx_id == can_id_ && rx_buf[0] == tx_data[0]) {
                if (rx_data) memcpy(rx_data, rx_buf, 8);
                return 0;
            }
        }
        usleep(1000);
        elapsed++;
    }

    return -1;
}

int LktechMotor::sendCmd(uint8_t cmd, uint8_t* rx_data) {
    uint8_t tx[8] = {cmd, 0, 0, 0, 0, 0, 0, 0};
    return sendAndRecv(tx, rx_data);
}

void LktechMotor::parseControlResponse(const uint8_t* rx, MotorStatus* s) {
    s->temperature    = (int8_t)rx[1];
    s->torque_current = (int16_t)(rx[2] | (rx[3] << 8));
    s->speed          = (int16_t)(rx[4] | (rx[5] << 8));
    s->encoder        = (uint16_t)(rx[6] | (rx[7] << 8));
}

/* ================================================================
 * Motor base interface implementation
 * ================================================================ */

int LktechMotor::init() {
    /* Verify communication by reading status2 (0x9C) */
    uint8_t rx[8];
    if (sendCmd(lktech::CMD_READ_STATUS_2, rx) < 0) {
        fprintf(stderr, "[LktechMotor %s 0x%03X] Init failed: no response\n",
                name_, can_id_);
        return -1;
    }

    int8_t temp = (int8_t)rx[1];
    int16_t speed = (int16_t)(rx[4] | (rx[5] << 8));
    fprintf(stderr, "[LktechMotor %s 0x%03X] Init OK: temp=%d C, speed=%d dps\n",
            name_, can_id_, temp, speed);

    return 0;
}

int LktechMotor::setTorque(int16_t iq, MotorStatus* status_out) {
    /* Clamp to LK-TECH safety limit */
    if (iq > lktech::TORQUE_IQ_LIMIT)  iq = lktech::TORQUE_IQ_LIMIT;
    if (iq < -lktech::TORQUE_IQ_LIMIT) iq = -lktech::TORQUE_IQ_LIMIT;

    /* Build 0xA1 command:
     * [0xA1] [0x00] [0x00] [0x00] [iq_L] [iq_H] [0x00] [0x00] */
    uint8_t tx[8] = {
        lktech::CMD_TORQUE_CLOSED_LOOP,
        0x00, 0x00, 0x00,
        (uint8_t)(iq & 0xFF),
        (uint8_t)((iq >> 8) & 0xFF),
        0x00, 0x00
    };

    uint8_t rx[8];
    if (sendAndRecv(tx, rx) < 0) {
        return -1;
    }

    if (status_out) {
        memset(status_out, 0, sizeof(MotorStatus));
        parseControlResponse(rx, status_out);
    }

    return 0;
}

int LktechMotor::setPosition(int32_t angle_001deg, uint16_t max_speed,
                             MotorStatus* status_out) {
    /* Build 0xA4 command (multi-turn absolute):
     * [0xA4] [0x00] [maxSpeed_L] [maxSpeed_H]
     * [angle_L] [angle_ML] [angle_MH] [angle_H] */
    uint8_t tx[8] = {
        lktech::CMD_POS_MULTI_2,
        0x00,
        (uint8_t)(max_speed & 0xFF),
        (uint8_t)((max_speed >> 8) & 0xFF),
        (uint8_t)(angle_001deg & 0xFF),
        (uint8_t)((angle_001deg >> 8) & 0xFF),
        (uint8_t)((angle_001deg >> 16) & 0xFF),
        (uint8_t)((angle_001deg >> 24) & 0xFF)
    };

    uint8_t rx[8];
    if (sendAndRecv(tx, rx) < 0) {
        return -1;
    }

    if (status_out) {
        memset(status_out, 0, sizeof(MotorStatus));
        parseControlResponse(rx, status_out);
    }

    return 0;
}

int LktechMotor::readStatus(MotorStatus* out) {
    if (!out) return -1;
    memset(out, 0, sizeof(MotorStatus));

    int success = 0;
    uint8_t rx[8];

    /* 0x9A: temperature, voltage, error state */
    if (sendCmd(lktech::CMD_READ_STATUS_1_ERROR, rx) == 0) {
        out->temperature = (int8_t)rx[1];
        out->voltage     = (uint16_t)(rx[3] | (rx[4] << 8));
        out->error_state = rx[7];
        success++;
    }

    /* 0x9C: temperature, torque current, speed, encoder */
    if (sendCmd(lktech::CMD_READ_STATUS_2, rx) == 0) {
        out->temperature    = (int8_t)rx[1];
        out->torque_current = (int16_t)(rx[2] | (rx[3] << 8));
        out->speed          = (int16_t)(rx[4] | (rx[5] << 8));
        out->encoder        = (uint16_t)(rx[6] | (rx[7] << 8));

        /* Compute acceleration from speed delta */
        struct timespec now;
        clock_gettime(CLOCK_MONOTONIC, &now);
        if (has_last_speed_) {
            double dt = (now.tv_sec - last_speed_time_.tv_sec)
                      + (now.tv_nsec - last_speed_time_.tv_nsec) * 1e-9;
            if (dt > 0.001) {
                out->acceleration = (int32_t)((out->speed - last_speed_) / dt);
            }
        }
        last_speed_ = out->speed;
        last_speed_time_ = now;
        has_last_speed_ = true;

        success++;
    }

    /* 0x92: multi-turn angle (int64_t, 7 bytes little-endian in rx[1..7]) */
    if (sendCmd(lktech::CMD_READ_MULTI_ANGLE, rx) == 0) {
        int64_t angle = 0;
        memcpy(&angle, &rx[1], 7);
        /* Sign extend if bit 55 set */
        if (rx[7] & 0x80) {
            angle |= (int64_t)0xFF << 56;
        }
        out->angle = angle;
        success++;
    }

    return (success > 0) ? 0 : -1;
}

int LktechMotor::stop() {
    uint8_t rx[8];
    return sendCmd(lktech::CMD_MOTOR_STOP, rx);
}

int LktechMotor::shutdown() {
    uint8_t rx[8];
    return sendCmd(lktech::CMD_MOTOR_SHUTDOWN, rx);
}

int LktechMotor::calibrateZero() {
    return setZeroToROM();
}

int16_t LktechMotor::torqueLimit() const {
    return lktech::TORQUE_IQ_LIMIT;
}

bool LktechMotor::hasError(const MotorStatus& status) const {
    return (status.error_state & lktech::ERROR_LOW_VOLTAGE)
        || (status.error_state & lktech::ERROR_OVER_TEMP);
}

/* ================================================================
 * LK-TECH specific commands
 * ================================================================ */

int LktechMotor::setPositionSingle(uint16_t angle_001deg, uint8_t direction,
                                   uint16_t max_speed,
                                   MotorStatus* status_out) {
    /* Build 0xA6 command (single-turn with direction):
     * [0xA6] [spinDir] [maxSpeed_L] [maxSpeed_H]
     * [angle_L] [angle_H] [0x00] [0x00] */
    uint8_t tx[8] = {
        lktech::CMD_POS_SINGLE_2,
        direction,
        (uint8_t)(max_speed & 0xFF),
        (uint8_t)((max_speed >> 8) & 0xFF),
        (uint8_t)(angle_001deg & 0xFF),
        (uint8_t)((angle_001deg >> 8) & 0xFF),
        0x00, 0x00
    };

    uint8_t rx[8];
    if (sendAndRecv(tx, rx) < 0) return -1;

    if (status_out) {
        memset(status_out, 0, sizeof(MotorStatus));
        parseControlResponse(rx, status_out);
    }
    return 0;
}

int LktechMotor::setSpeed(int32_t speed_001dps, MotorStatus* status_out) {
    /* Build 0xA2 command:
     * [0xA2] [0x00] [0x00] [0x00]
     * [speed_L] [speed_ML] [speed_MH] [speed_H] */
    uint8_t tx[8] = {
        lktech::CMD_SPEED_CLOSED_LOOP,
        0x00, 0x00, 0x00,
        (uint8_t)(speed_001dps & 0xFF),
        (uint8_t)((speed_001dps >> 8) & 0xFF),
        (uint8_t)((speed_001dps >> 16) & 0xFF),
        (uint8_t)((speed_001dps >> 24) & 0xFF)
    };

    uint8_t rx[8];
    if (sendAndRecv(tx, rx) < 0) return -1;

    if (status_out) {
        memset(status_out, 0, sizeof(MotorStatus));
        parseControlResponse(rx, status_out);
    }
    return 0;
}

int LktechMotor::writeEncoderOffset(uint16_t encoder_offset) {
    /* Build 0x91 command:
     * [0x91] [0x00] [0x00] [0x00] [0x00] [0x00] [offset_L] [offset_H] */
    uint8_t tx[8] = {
        lktech::CMD_WRITE_ENCODER_OFFSET,
        0x00, 0x00, 0x00, 0x00, 0x00,
        (uint8_t)(encoder_offset & 0xFF),
        (uint8_t)((encoder_offset >> 8) & 0xFF)
    };

    uint8_t rx[8];
    return sendAndRecv(tx, rx);
}

int LktechMotor::setZeroToROM() {
    /* Command 0x19: set current position as zero in ROM.
     * Requires MCU reboot to take effect. */
    uint8_t rx[8];
    return sendCmd(lktech::CMD_SET_ZERO_ROM, rx);
}

int LktechMotor::readPID(uint8_t* angle_kp, uint8_t* angle_ki,
                         uint8_t* speed_kp, uint8_t* speed_ki,
                         uint8_t* torque_kp, uint8_t* torque_ki) {
    uint8_t rx[8];
    if (sendCmd(lktech::CMD_READ_PID_PARAMS, rx) < 0) return -1;

    if (angle_kp)  *angle_kp  = rx[2];
    if (angle_ki)  *angle_ki  = rx[3];
    if (speed_kp)  *speed_kp  = rx[4];
    if (speed_ki)  *speed_ki  = rx[5];
    if (torque_kp) *torque_kp = rx[6];
    if (torque_ki) *torque_ki = rx[7];

    return 0;
}

int LktechMotor::readEncoder(uint32_t* encoder_raw) {
    uint8_t rx[8];
    if (sendCmd(lktech::CMD_READ_ENCODER, rx) < 0) return -1;

    if (encoder_raw) {
        /* 18-bit encoder in rx[2..4] (little-endian, 3 bytes) */
        *encoder_raw = rx[2] | (rx[3] << 8) | (rx[4] << 16);
        *encoder_raw &= 0x3FFFF;  /* mask to 18 bits */
    }

    return 0;
}
