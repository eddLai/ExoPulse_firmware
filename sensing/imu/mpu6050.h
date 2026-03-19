/*
 * MPU6050 IMU Module Interface
 * 6-axis: 3-axis accelerometer + 3-axis gyroscope + quaternion attitude
 *
 * Hardware: MPU6050 (GY-521) via I2C
 * Platform: Jetson Orin (I2C bus 7)
 *
 * Features:
 *   - Configurable sample rate (1 Hz ~ 1 kHz)
 *   - Gyro bias calibration at init
 *   - Madgwick AHRS filter for quaternion estimation
 */

#ifndef MPU6050_H
#define MPU6050_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ---- Data Structures ---- */

typedef struct {
    float accel[3];    /* [ax, ay, az] in g */
    float gyro[3];     /* [gx, gy, gz] in deg/s */
    float quat[4];     /* [qw, qx, qy, qz] orientation quaternion */
    float temperature; /* Celsius */
    double timestamp;  /* seconds (CLOCK_MONOTONIC) */
} imu_data_t;

typedef struct {
    uint16_t sample_rate_hz;  /* Desired ODR: 4~1000 Hz (default 200) */
    uint16_t calibration_samples; /* Gyro bias samples at init (default 500) */
    float    madgwick_beta;   /* Madgwick filter gain (default 0.1) */
} mpu6050_config_t;

/* Opaque handle */
typedef struct mpu6050 mpu6050_t;

/* ---- Sensor API ---- */

/*
 * Create MPU6050 instance.
 * i2c_device: e.g. "/dev/i2c-7"
 * address: 0x68 (AD0=GND) or 0x69 (AD0=VCC)
 * Returns NULL on failure.
 */
mpu6050_t* mpu6050_create(const char* i2c_device, uint8_t address);

/*
 * Initialize with default config: 200 Hz, 500 cal samples, beta=0.1
 * Returns 0 on success, -1 on failure.
 */
int mpu6050_init(mpu6050_t* dev);

/*
 * Initialize with custom config.
 * Returns 0 on success, -1 on failure.
 */
int mpu6050_init_config(mpu6050_t* dev, const mpu6050_config_t* cfg);

/*
 * Read one sample: accel(3) + gyro(3) + quat(4) + temperature + timestamp.
 * Quaternion is updated internally via Madgwick filter on each read.
 * Returns 0 on success, -1 on failure.
 */
int mpu6050_read(mpu6050_t* dev, imu_data_t* data);

/*
 * Destroy instance, close I2C fd.
 */
void mpu6050_destroy(mpu6050_t* dev);

/* ---- Quaternion API ---- */

/*
 * Get the current orientation quaternion [qw, qx, qy, qz].
 * Does NOT read the sensor — returns the last fused value.
 */
int mpu6050_get_quaternion(mpu6050_t* dev, float quat_out[4]);

/*
 * Reset the quaternion to identity [1, 0, 0, 0].
 * Call this when the sensor is in a known reference pose.
 */
int mpu6050_reset_quaternion(mpu6050_t* dev);

/*
 * Convert quaternion to Euler angles (roll, pitch, yaw) in degrees.
 */
void quat_to_euler(const float q[4], float* roll, float* pitch, float* yaw);

/*
 * Populate a default config struct.
 */
void mpu6050_default_config(mpu6050_config_t* cfg);

#ifdef __cplusplus
}
#endif

#endif /* MPU6050_H */
