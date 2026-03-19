/*
 * Motor Control Module Test
 * Standalone test: init CAN -> read status -> small torque -> stop
 *
 * Usage: ./test_motor [spi_device] [can_id_hex]
 *   Default: /dev/spidev0.0, motor at 0x141
 *
 * WARNING: This test sends a small torque command. Ensure the motor
 * is properly mounted and the path is clear before running.
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

int main(int argc, char* argv[]) {
    const char* spi_device = "/dev/spidev0.0";
    uint16_t can_id = 0x141;

    if (argc > 1) spi_device = argv[1];
    if (argc > 2) can_id = (uint16_t)strtol(argv[2], NULL, 16);

    signal(SIGINT, sig_handler);

    printf("=== Motor Control Module Test ===\n");
    printf("SPI: %s  CAN ID: 0x%03X\n\n", spi_device, can_id);

    /* 1. Initialize CAN bus */
    printf("[1] Creating MCP2515 CAN controller...\n");
    mcp2515_t* can = mcp2515_create(spi_device, 1000000);
    if (!can) {
        fprintf(stderr, "Failed to create MCP2515\n");
        return 1;
    }

    printf("[2] Initializing CAN (1Mbps)...\n");
    if (mcp2515_init(can, 1000000) < 0) {
        fprintf(stderr, "Failed to initialize MCP2515\n");
        mcp2515_destroy(can);
        return 1;
    }
    printf("    CAN bus ready\n\n");

    /* 2. Create motor instance */
    printf("[3] Creating motor instance (CAN ID 0x%03X)...\n", can_id);
    lktech_motor_t* motor = motor_create(can, can_id);
    if (!motor) {
        fprintf(stderr, "Failed to create motor\n");
        mcp2515_destroy(can);
        return 1;
    }

    printf("[4] Initializing motor...\n");
    if (motor_init(motor) < 0) {
        fprintf(stderr, "Motor init failed (no CAN response)\n");
        motor_destroy(motor);
        mcp2515_destroy(can);
        return 1;
    }
    printf("    Motor communication OK\n\n");

    /* 3. Read full status */
    printf("[5] Reading motor status...\n");
    motor_status_t status;
    if (motor_read_status(motor, &status) == 0) {
        print_status(&status);
    } else {
        printf("    Status read failed\n");
    }

    /* 4. Send small torque (iq=50, very gentle) */
    if (running) {
        printf("\n[6] Sending small torque (iq=50) for 1 second...\n");
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

    /* 5. Stop motor */
    printf("\n[7] Stopping motor...\n");
    if (motor_stop(motor) == 0) {
        printf("    Motor stopped\n");
    } else {
        printf("    Stop command failed\n");
    }

    /* 6. Final status */
    printf("\n[8] Final status:\n");
    if (motor_read_status(motor, &status) == 0) {
        print_status(&status);
    }

    /* Cleanup */
    printf("\nCleaning up...\n");
    motor_destroy(motor);
    mcp2515_destroy(can);
    printf("Done.\n");

    return 0;
}
