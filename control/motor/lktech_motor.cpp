/*
 * LK-TECH Motor Control Implementation
 * Refactored from MGv2/include/motor_control.h + motor_operations.h
 *
 * Ported from ESP32 (FreeRTOS, MCP_CAN lib) to Jetson Orin Linux (mcp2515 module).
 * Protocol unchanged — same CAN frame format as MGv2 firmware.
 */

#include "lktech_motor.h"

#include <cstdlib>
#include <cstring>
#include <cstdio>
#include <unistd.h>
#include <time.h>

/* ---- LK-TECH Protocol Commands ---- */
#define CMD_MOTOR_SHUTDOWN       0x80
#define CMD_MOTOR_STOP           0x81
#define CMD_READ_STATUS_1_ERROR  0x9A
#define CMD_READ_STATUS_2        0x9C
#define CMD_READ_MULTI_ANGLE     0x92
#define CMD_TORQUE_CLOSED_LOOP   0xA1

/* Safety limit: iq +/-800 (~12.8A) */
#define TORQUE_IQ_LIMIT 800

/* CAN response timeout */
#define RESPONSE_TIMEOUT_MS 100

struct lktech_motor {
    mcp2515_t* can;
    uint16_t can_id;
    int16_t last_speed;
    struct timespec last_speed_time;
    int has_last_speed;
};

/* ---- Helpers ---- */

/*
 * Send an 8-byte CAN frame and wait for the motor's response.
 * The motor echoes back with the same command byte in rxData[0].
 * Returns 0 on success, -1 on timeout.
 */
static int send_and_recv(lktech_motor_t* dev, uint8_t* tx_data, uint8_t* rx_data) {
    if (mcp2515_send(dev->can, dev->can_id, tx_data, 8) < 0) {
        return -1;
    }

    /* Poll for response matching our CAN ID and command byte */
    uint16_t rx_id;
    uint8_t rx_buf[8];
    uint8_t rx_len;
    uint32_t elapsed = 0;

    while (elapsed < RESPONSE_TIMEOUT_MS) {
        if (mcp2515_recv(dev->can, &rx_id, rx_buf, &rx_len, 0) == 0) {
            if (rx_id == dev->can_id && rx_buf[0] == tx_data[0]) {
                if (rx_data) memcpy(rx_data, rx_buf, 8);
                return 0;
            }
        }
        usleep(1000);
        elapsed++;
    }

    return -1;
}

/* Send a read command (command byte + 7 zeros) */
static int send_read_cmd(lktech_motor_t* dev, uint8_t cmd, uint8_t* rx_data) {
    uint8_t tx[8] = {cmd, 0, 0, 0, 0, 0, 0, 0};
    return send_and_recv(dev, tx, rx_data);
}

/* Parse 0xA1/0xA6 control response: temp, iq, speed, encoder */
static void parse_control_response(const uint8_t* rx, motor_status_t* s) {
    s->temperature    = (int8_t)rx[1];
    s->torque_current = (int16_t)(rx[2] | (rx[3] << 8));
    s->speed          = (int16_t)(rx[4] | (rx[5] << 8));
    s->encoder        = (uint16_t)(rx[6] | (rx[7] << 8));
}

/* ---- Public API ---- */

lktech_motor_t* motor_create(mcp2515_t* can, uint16_t can_id) {
    if (!can) return NULL;

    lktech_motor_t* dev = (lktech_motor_t*)malloc(sizeof(lktech_motor_t));
    if (!dev) return NULL;

    dev->can = can;
    dev->can_id = can_id;
    dev->last_speed = 0;
    dev->has_last_speed = 0;
    memset(&dev->last_speed_time, 0, sizeof(dev->last_speed_time));

    return dev;
}

int motor_init(lktech_motor_t* dev) {
    if (!dev) return -1;

    /* Verify communication by reading status2 */
    uint8_t rx[8];
    if (send_read_cmd(dev, CMD_READ_STATUS_2, rx) < 0) {
        fprintf(stderr, "[motor 0x%03X] Init failed: no response\n", dev->can_id);
        return -1;
    }

    int8_t temp = (int8_t)rx[1];
    int16_t speed = (int16_t)(rx[4] | (rx[5] << 8));
    fprintf(stderr, "[motor 0x%03X] Init OK: temp=%d C, speed=%d dps\n",
            dev->can_id, temp, speed);

    return 0;
}

int motor_set_torque(lktech_motor_t* dev, int16_t iq, motor_status_t* status_out) {
    if (!dev) return -1;

    /* Clamp to safety limit */
    if (iq > TORQUE_IQ_LIMIT)  iq = TORQUE_IQ_LIMIT;
    if (iq < -TORQUE_IQ_LIMIT) iq = -TORQUE_IQ_LIMIT;

    /* Build 0xA1 command:
     * [0xA1] [0x00] [0x00] [0x00] [iq_L] [iq_H] [0x00] [0x00] */
    uint8_t tx[8] = {
        CMD_TORQUE_CLOSED_LOOP,
        0x00, 0x00, 0x00,
        (uint8_t)(iq & 0xFF),
        (uint8_t)((iq >> 8) & 0xFF),
        0x00, 0x00
    };

    uint8_t rx[8];
    if (send_and_recv(dev, tx, rx) < 0) {
        return -1;
    }

    if (status_out) {
        memset(status_out, 0, sizeof(motor_status_t));
        parse_control_response(rx, status_out);
    }

    return 0;
}

int motor_read_status(lktech_motor_t* dev, motor_status_t* out) {
    if (!dev || !out) return -1;
    memset(out, 0, sizeof(motor_status_t));

    int success = 0;
    uint8_t rx[8];

    /* 0x9A: temperature, voltage, error state */
    if (send_read_cmd(dev, CMD_READ_STATUS_1_ERROR, rx) == 0) {
        out->temperature = (int8_t)rx[1];
        out->voltage     = (uint16_t)(rx[3] | (rx[4] << 8));
        out->error_state = rx[7];
        success++;
    }

    /* 0x9C: temperature, torque current, speed, encoder */
    if (send_read_cmd(dev, CMD_READ_STATUS_2, rx) == 0) {
        out->temperature    = (int8_t)rx[1];
        out->torque_current = (int16_t)(rx[2] | (rx[3] << 8));
        out->speed          = (int16_t)(rx[4] | (rx[5] << 8));
        out->encoder        = (uint16_t)(rx[6] | (rx[7] << 8));

        /* Compute acceleration from speed delta */
        struct timespec now;
        clock_gettime(CLOCK_MONOTONIC, &now);
        if (dev->has_last_speed) {
            double dt = (now.tv_sec - dev->last_speed_time.tv_sec)
                      + (now.tv_nsec - dev->last_speed_time.tv_nsec) * 1e-9;
            if (dt > 0.001) {
                out->acceleration = (int32_t)((out->speed - dev->last_speed) / dt);
            }
        }
        dev->last_speed = out->speed;
        dev->last_speed_time = now;
        dev->has_last_speed = 1;

        success++;
    }

    /* 0x92: multi-turn angle (int64_t, 7 bytes little-endian in rx[1..7]) */
    if (send_read_cmd(dev, CMD_READ_MULTI_ANGLE, rx) == 0) {
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

int motor_stop(lktech_motor_t* dev) {
    if (!dev) return -1;
    uint8_t rx[8];
    return send_read_cmd(dev, CMD_MOTOR_STOP, rx);
}

int motor_shutdown(lktech_motor_t* dev) {
    if (!dev) return -1;
    uint8_t rx[8];
    return send_read_cmd(dev, CMD_MOTOR_SHUTDOWN, rx);
}

void motor_destroy(lktech_motor_t* dev) {
    if (!dev) return;
    free(dev);
}
