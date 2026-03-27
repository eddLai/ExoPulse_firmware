/*
 * Motor Control Module Test
 * Standalone test: init CAN -> test motor 0x141 and 0x142
 *
 * Usage: ./test_motor [spi_device]
 *   Default: /dev/spidev0.0, tests both motors (0x141 and 0x142)
 *
 * WARNING: This test sends a small torque command. Ensure the motors
 * are properly mounted and the path is clear before running.
 */

#include "mcp2515.h"
#include "lktech_motor.h"

#include <cstdio>
#include <cstdlib>
#include <csignal>
#include <unistd.h>

static volatile int running = 1;

static void sig_handler(int sig) {
    (void)sig;
    running = 0;
}

static void print_status(const motor_status_t* s) {
    printf("  Temp: %d C\n", s->temperature);
    printf("  Voltage: %.1f V\n", s->voltage * 0.1);
    printf("  Torque current (iq): %d\n", s->torque_current);
    printf("  Speed: %d deg/s\n", s->speed);
    printf("  Acceleration: %d deg/s^2\n", s->acceleration);
    printf("  Encoder: %u\n", s->encoder);
    printf("  Angle: %.2f deg (multi-turn)\n", s->angle * 0.01);
    printf("  Error: 0x%02X%s%s\n", s->error_state,
           (s->error_state & 0x01) ? " [LOW_VOLTAGE]" : "",
           (s->error_state & 0x08) ? " [OVER_TEMP]" : "");
}

static void test_motor(mcp2515_t* can, uint16_t can_id) {
    printf("\n========== Testing Motor 0x%03X ==========\n", can_id);

    /* Create motor instance */
    printf("[1] Creating motor instance (CAN ID 0x%03X)...\n", can_id);
    lktech_motor_t* motor = motor_create(can, can_id);
    if (!motor) {
        fprintf(stderr, "Failed to create motor 0x%03X\n", can_id);
        return;
    }

    printf("[2] Initializing motor...\n");
    if (motor_init(motor) < 0) {
        fprintf(stderr, "Motor 0x%03X init failed (no CAN response)\n", can_id);
        motor_destroy(motor);
        return;
    }
    printf("    Motor 0x%03X communication OK\n", can_id);

    /* Read status */
    printf("[3] Reading motor status...\n");
    motor_status_t status;
    if (motor_read_status(motor, &status) == 0) {
        print_status(&status);
    } else {
        printf("    Status read failed\n");
    }

    /* Send small torque */
    if (running) {
        printf("\n[4] Sending small torque (iq=50) for 1 second...\n");
        motor_status_t torque_status;
        if (motor_set_torque(motor, 50, &torque_status) == 0) {
            printf("    Torque response:\n");
            printf("    iq=%d, speed=%d dps, encoder=%u\n",
                   torque_status.torque_current, torque_status.speed,
                   torque_status.encoder);
        } else {
            printf("    Torque command failed\n");
        }

        sleep(1);
    }

    /* Stop motor */
    printf("\n[5] Stopping motor...\n");
    if (motor_stop(motor) == 0) {
        printf("    Motor 0x%03X stopped\n", can_id);
    } else {
        printf("    Stop command failed\n");
    }

    /* Final status */
    printf("[6] Final status:\n");
    if (motor_read_status(motor, &status) == 0) {
        print_status(&status);
    }

    motor_destroy(motor);
}

int main(int argc, char* argv[]) {
    const char* spi_device = "/dev/spidev0.0";

    if (argc > 1) spi_device = argv[1];

    signal(SIGINT, sig_handler);

    printf("=== Motor Control Module Test (0x141 & 0x142) ===\n");
    printf("SPI: %s\n\n", spi_device);

    /* 1. Initialize CAN bus */
    printf("[CAN] Creating MCP2515 CAN controller...\n");
    mcp2515_t* can = mcp2515_create(spi_device, 1000000);
    if (!can) {
        fprintf(stderr, "Failed to create MCP2515\n");
        return 1;
    }

    printf("[CAN] Initializing CAN (1Mbps)...\n");
    if (mcp2515_init(can, 1000000) < 0) {
        fprintf(stderr, "Failed to initialize MCP2515\n");
        mcp2515_destroy(can);
        return 1;
    }
    printf("[CAN] CAN bus ready\n");

    /* Test both motors */
    test_motor(can, 0x141);
    test_motor(can, 0x142);

    /* Cleanup */
    printf("\n========== Done ==========\n");
    mcp2515_destroy(can);

    return 0;
}
