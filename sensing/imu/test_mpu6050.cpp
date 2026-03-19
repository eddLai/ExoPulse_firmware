/*
 * MPU6050 Module Test
 * Standalone test: init -> continuous read -> print accel/gyro/quaternion/euler
 *
 * Usage: ./test_mpu6050 [i2c_device] [address_hex] [sample_rate_hz]
 *   Default: /dev/i2c-7 0x68 200
 */

#include "mpu6050.h"
#include <cstdio>
#include <cstdlib>
#include <csignal>
#include <unistd.h>

static volatile int running = 1;

static void sig_handler(int sig) {
    (void)sig;
    running = 0;
}

int main(int argc, char* argv[]) {
    const char* device = "/dev/i2c-7";
    uint8_t address = 0x68;
    uint16_t rate = 200;

    if (argc > 1) device = argv[1];
    if (argc > 2) address = (uint8_t)strtol(argv[2], NULL, 16);
    if (argc > 3) rate = (uint16_t)atoi(argv[3]);

    signal(SIGINT, sig_handler);

    printf("=== MPU6050 Module Test ===\n");
    printf("Device: %s  Address: 0x%02X  Rate: %d Hz\n\n", device, address, rate);

    mpu6050_t* imu = mpu6050_create(device, address);
    if (!imu) {
        fprintf(stderr, "Failed to create MPU6050 instance\n");
        return 1;
    }

    mpu6050_config_t cfg;
    mpu6050_default_config(&cfg);
    cfg.sample_rate_hz = rate;

    if (mpu6050_init_config(imu, &cfg) < 0) {
        fprintf(stderr, "Failed to initialize MPU6050\n");
        mpu6050_destroy(imu);
        return 1;
    }

    printf("MPU6050 initialized. Reading data (Ctrl+C to stop)...\n\n");

    uint32_t read_delay_us = 1000000 / rate;

    imu_data_t data;
    int count = 0;
    int print_every = (rate > 20) ? (rate / 20) : 1;

    while (running) {
        if (mpu6050_read(imu, &data) == 0) {
            count++;
            if (count % print_every == 0) {
                float roll, pitch, yaw;
                quat_to_euler(data.quat, &roll, &pitch, &yaw);

                printf("\033[2K\r"
                       "A:[%6.2f %6.2f %6.2f]g  "
                       "G:[%7.1f %7.1f %7.1f]d/s  "
                       "Q:[%5.2f %5.2f %5.2f %5.2f]  "
                       "RPY:[%6.1f %6.1f %6.1f]",
                       data.accel[0], data.accel[1], data.accel[2],
                       data.gyro[0], data.gyro[1], data.gyro[2],
                       data.quat[0], data.quat[1], data.quat[2], data.quat[3],
                       roll, pitch, yaw);
                fflush(stdout);
            }
        } else {
            fprintf(stderr, "\nRead failed\n");
        }
        usleep(read_delay_us);
    }

    printf("\n\nShutting down...\n");
    mpu6050_destroy(imu);
    return 0;
}
