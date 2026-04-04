/*
 * Motor Control Module Test (Inheritance-based)
 * Tests the new Motor base class with LktechMotor subclass.
 *
 * Usage: ./test_motor [spi_device]
 *   Default: /dev/spidev0.0, tests both motors (0x141 and 0x142)
 *
 * Demonstrates:
 *   - CanBus / Mcp2515Can
 *   - Motor* base pointer (brand-independent)
 *   - LktechMotor-specific commands (downcast)
 *   - MotorProtection wrapper
 */

#include "motor_base.h"
#include "lktech/lktech_motor.h"
#include "motor_protection.h"
#include "../mcp2515_can.h"

#include <cstdio>
#include <cstdlib>
#include <csignal>
#include <unistd.h>

static volatile int running = 1;

static void sig_handler(int sig) {
    (void)sig;
    running = 0;
}

static void print_status(const MotorStatus* s) {
    printf("  Temp: %d C\n", s->temperature);
    printf("  Voltage: %.1f V\n", s->voltage * 0.1);
    printf("  Torque current (iq): %d\n", s->torque_current);
    printf("  Speed: %d deg/s\n", s->speed);
    printf("  Acceleration: %d deg/s^2\n", s->acceleration);
    printf("  Encoder: %u\n", s->encoder);
    printf("  Angle: %.2f deg (multi-turn)\n", s->angle * 0.01);
    printf("  Error: 0x%02X\n", s->error_state);
}

/* Test using Motor* base pointer (brand-independent) */
static void test_motor_generic(Motor* m) {
    printf("\n========== Testing %s (0x%03X) via Motor* ==========\n",
           m->name(), m->canId());

    printf("[1] Init...\n");
    if (m->init() < 0) {
        fprintf(stderr, "    Init failed\n");
        return;
    }

    printf("[2] Read status...\n");
    MotorStatus status;
    if (m->readStatus(&status) == 0) {
        print_status(&status);
    }

    if (running) {
        printf("[3] Small torque (iq=50) for 1 second...\n");
        MotorStatus ts;
        if (m->setTorque(50, &ts) == 0) {
            printf("    iq=%d, speed=%d dps\n", ts.torque_current, ts.speed);
        }
        sleep(1);
    }

    printf("[4] Stop...\n");
    m->stop();

    printf("[5] Final status:\n");
    if (m->readStatus(&status) == 0) {
        print_status(&status);
    }
}

/* Test LktechMotor-specific commands (requires downcast) */
static void test_lktech_specific(LktechMotor* lk) {
    printf("\n--- LK-TECH specific: readPID ---\n");
    uint8_t akp, aki, skp, ski, tkp, tki;
    if (lk->readPID(&akp, &aki, &skp, &ski, &tkp, &tki) == 0) {
        printf("  Angle PID: Kp=%u Ki=%u\n", akp, aki);
        printf("  Speed PID: Kp=%u Ki=%u\n", skp, ski);
        printf("  Torque PID: Kp=%u Ki=%u\n", tkp, tki);
    }

    printf("--- LK-TECH specific: readEncoder ---\n");
    uint32_t enc;
    if (lk->readEncoder(&enc) == 0) {
        printf("  Raw encoder: %u (0x%05X)\n", enc, enc);
    }
}

/* Test MotorProtection wrapper */
static void test_protection(Motor* m) {
    printf("\n========== Testing MotorProtection on %s ==========\n", m->name());

    ProtectConfig cfg;
    cfg.angle_min_deg = -30.0;
    cfg.angle_max_deg =  30.0;
    cfg.torque_max_nm = 100.0;

    MotorProtection prot(*m, cfg);
    if (prot.init() != MOTOR_OK) {
        fprintf(stderr, "    Protection init failed\n");
        return;
    }

    ProtectedStatus ps;
    if (prot.readStatus(&ps) == MOTOR_OK) {
        printf("  Angle: %.2f deg, Torque: %.2f Nm\n",
               ps.angle_deg, ps.torque_nm);
        printf("  In range: %s\n", prot.angleInRange() ? "YES" : "NO");
    }

    prot.stop();
}

int main(int argc, char* argv[]) {
    const char* spi_device = "/dev/spidev0.0";
    if (argc > 1) spi_device = argv[1];

    signal(SIGINT, sig_handler);

    printf("=== Motor Control Test (C++ Inheritance) ===\n");
    printf("SPI: %s\n\n", spi_device);

    /* 1. Create CAN bus (brand-independent) */
    printf("[CAN] Creating Mcp2515Can...\n");
    Mcp2515Can can(spi_device, 1000000);
    if (can.init(1000000) < 0) {
        fprintf(stderr, "Failed to initialize CAN\n");
        return 1;
    }
    printf("[CAN] Ready\n");

    /* 2. Create motors (concrete types) */
    LktechMotor hip_right(can, 0x141, "hip_right");
    LktechMotor hip_left(can, 0x142, "hip_left");

    /* 3. Test via Motor* base pointer (brand-independent) */
    Motor* motors[] = { &hip_right, &hip_left };
    for (auto* m : motors) {
        test_motor_generic(m);
    }

    /* 4. Test LK-TECH specific commands (downcast) */
    test_lktech_specific(&hip_right);

    /* 5. Test MotorProtection */
    test_protection(&hip_right);

    printf("\n========== Done ==========\n");
    return 0;
}
