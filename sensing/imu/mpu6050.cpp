/*
 * MPU6050 IMU Module Implementation
 * - Configurable sample rate via SMPLRT_DIV + DLPF
 * - Gyro bias calibration at init (static hold)
 * - Madgwick AHRS filter for quaternion from accel + gyro
 *
 * I2C communication self-contained (open + ioctl(I2C_SLAVE))
 */

#include "mpu6050.h"

#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <cstdlib>
#include <cstring>
#include <cstdio>
#include <cmath>
#include <time.h>

/* ---- Register addresses ---- */
#define MPU6050_WHO_AM_I     0x75
#define MPU6050_PWR_MGMT_1   0x6B
#define MPU6050_SMPLRT_DIV   0x19
#define MPU6050_CONFIG       0x1A  /* DLPF config */
#define MPU6050_GYRO_CONFIG  0x1B
#define MPU6050_ACCEL_CONFIG 0x1C
#define MPU6050_ACCEL_XOUT_H 0x3B

/* Scale factors */
#define ACCEL_SCALE_2G  16384.0f  /* LSB/g   for +/-2g */
#define GYRO_SCALE_250  131.0f    /* LSB/(deg/s) for +/-250 deg/s */
#define DEG_TO_RAD      0.01745329251994329577f  /* M_PI / 180 */

struct mpu6050 {
    int fd;
    uint8_t addr;

    /* Madgwick filter state */
    float q[4];       /* quaternion [w, x, y, z] */
    float beta;       /* filter gain */
    double last_ts;   /* timestamp of last update */

    /* Gyro bias from calibration */
    float gyro_bias[3]; /* deg/s */

    /* Config */
    uint16_t sample_rate_hz;
};

/* ---- Low-level I2C helpers ---- */

static int i2c_write_byte(int fd, uint8_t reg, uint8_t value) {
    uint8_t buf[2] = {reg, value};
    return (write(fd, buf, 2) == 2) ? 0 : -1;
}

static int i2c_read_bytes(int fd, uint8_t reg, uint8_t* buf, int count) {
    if (write(fd, &reg, 1) != 1) return -1;
    if (read(fd, buf, count) != count) return -1;
    return 0;
}

static double get_time_sec(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec + ts.tv_nsec * 1e-9;
}

/* ---- Madgwick AHRS filter ---- */

static float inv_sqrt(float x) {
    return 1.0f / sqrtf(x);
}

/*
 * Madgwick filter update (6-axis: accel + gyro, no magnetometer).
 * gx, gy, gz in rad/s; ax, ay, az in g (normalized internally).
 */
static void madgwick_update(float q[4], float beta, float dt,
                            float gx, float gy, float gz,
                            float ax, float ay, float az)
{
    float q0 = q[0], q1 = q[1], q2 = q[2], q3 = q[3];

    /* Normalise accelerometer */
    float norm = sqrtf(ax * ax + ay * ay + az * az);
    if (norm < 1e-6f) {
        /* Accel too small — gyro-only integration */
        float halfdt = 0.5f * dt;
        float dq0 = (-q1 * gx - q2 * gy - q3 * gz) * halfdt;
        float dq1 = ( q0 * gx + q2 * gz - q3 * gy) * halfdt;
        float dq2 = ( q0 * gy - q1 * gz + q3 * gx) * halfdt;
        float dq3 = ( q0 * gz + q1 * gy - q2 * gx) * halfdt;
        q[0] += dq0; q[1] += dq1; q[2] += dq2; q[3] += dq3;
        norm = inv_sqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
        q[0] *= norm; q[1] *= norm; q[2] *= norm; q[3] *= norm;
        return;
    }
    float recip = inv_sqrt(ax * ax + ay * ay + az * az);
    ax *= recip;
    ay *= recip;
    az *= recip;

    /* Auxiliary variables */
    float _2q0 = 2.0f * q0, _2q1 = 2.0f * q1;
    float _2q2 = 2.0f * q2, _2q3 = 2.0f * q3;
    float _4q0 = 4.0f * q0, _4q1 = 4.0f * q1;
    float _4q2 = 4.0f * q2;
    float _8q1 = 8.0f * q1, _8q2 = 8.0f * q2;
    float q0q0 = q0 * q0, q1q1 = q1 * q1;
    float q2q2 = q2 * q2, q3q3 = q3 * q3;

    /* Gradient descent corrective step */
    float s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
    float s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay
               - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
    float s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay
               - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
    float s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;

    /* Normalise step magnitude */
    recip = inv_sqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
    s0 *= recip; s1 *= recip; s2 *= recip; s3 *= recip;

    /* Apply feedback step */
    float qDot0 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz) - beta * s0;
    float qDot1 = 0.5f * ( q0 * gx + q2 * gz - q3 * gy) - beta * s1;
    float qDot2 = 0.5f * ( q0 * gy - q1 * gz + q3 * gx) - beta * s2;
    float qDot3 = 0.5f * ( q0 * gz + q1 * gy - q2 * gx) - beta * s3;

    /* Integrate */
    q[0] = q0 + qDot0 * dt;
    q[1] = q1 + qDot1 * dt;
    q[2] = q2 + qDot2 * dt;
    q[3] = q3 + qDot3 * dt;

    /* Normalise quaternion */
    recip = inv_sqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
    q[0] *= recip; q[1] *= recip; q[2] *= recip; q[3] *= recip;
}

/* ---- DLPF bandwidth selection ---- */

/*
 * Pick DLPF_CFG for the requested sample rate.
 * Gyro output rate = 1 kHz when DLPF enabled (DLPF_CFG 1~6),
 *                    8 kHz when DLPF disabled (DLPF_CFG 0 or 7).
 * Returns DLPF_CFG value and the gyro base rate via *base_rate.
 */
static uint8_t pick_dlpf(uint16_t sample_rate_hz, uint16_t* base_rate) {
    /*
     * DLPF_CFG | Accel BW | Gyro BW | Gyro Rate
     *    0     |  260 Hz  | 256 Hz  |  8 kHz
     *    1     |  184 Hz  | 188 Hz  |  1 kHz
     *    2     |   94 Hz  |  98 Hz  |  1 kHz
     *    3     |   44 Hz  |  42 Hz  |  1 kHz
     *    4     |   21 Hz  |  20 Hz  |  1 kHz
     *    5     |   10 Hz  |  10 Hz  |  1 kHz
     *    6     |    5 Hz  |   5 Hz  |  1 kHz
     */
    if (sample_rate_hz > 500) {
        *base_rate = 8000;
        return 0;           /* No DLPF, 8 kHz base */
    } else if (sample_rate_hz > 100) {
        *base_rate = 1000;
        return 1;           /* 188 Hz BW */
    } else if (sample_rate_hz > 50) {
        *base_rate = 1000;
        return 2;           /* 98 Hz BW */
    } else if (sample_rate_hz > 20) {
        *base_rate = 1000;
        return 3;           /* 42 Hz BW */
    } else {
        *base_rate = 1000;
        return 4;           /* 20 Hz BW */
    }
}

/* ---- Public API ---- */

void mpu6050_default_config(mpu6050_config_t* cfg) {
    if (!cfg) return;
    cfg->sample_rate_hz = 200;
    cfg->calibration_samples = 500;
    cfg->madgwick_beta = 0.1f;
}

mpu6050_t* mpu6050_create(const char* i2c_device, uint8_t address) {
    int fd = open(i2c_device, O_RDWR);
    if (fd < 0) {
        fprintf(stderr, "[mpu6050] Failed to open %s\n", i2c_device);
        return NULL;
    }

    if (ioctl(fd, I2C_SLAVE, address) < 0) {
        fprintf(stderr, "[mpu6050] Failed to set I2C address 0x%02X\n", address);
        close(fd);
        return NULL;
    }

    mpu6050_t* dev = (mpu6050_t*)calloc(1, sizeof(mpu6050_t));
    if (!dev) {
        close(fd);
        return NULL;
    }

    dev->fd = fd;
    dev->addr = address;

    /* Identity quaternion */
    dev->q[0] = 1.0f; dev->q[1] = 0.0f; dev->q[2] = 0.0f; dev->q[3] = 0.0f;
    dev->beta = 0.1f;
    dev->last_ts = 0.0;
    dev->sample_rate_hz = 200;

    return dev;
}

int mpu6050_init(mpu6050_t* dev) {
    mpu6050_config_t cfg;
    mpu6050_default_config(&cfg);
    return mpu6050_init_config(dev, &cfg);
}

int mpu6050_init_config(mpu6050_t* dev, const mpu6050_config_t* cfg) {
    if (!dev || !cfg) return -1;

    /* Check WHO_AM_I */
    uint8_t who = 0;
    if (i2c_read_bytes(dev->fd, MPU6050_WHO_AM_I, &who, 1) < 0) {
        fprintf(stderr, "[mpu6050] Failed to read WHO_AM_I\n");
        return -1;
    }
    if (who != 0x68 && who != 0x72) {
        fprintf(stderr, "[mpu6050] Unexpected WHO_AM_I: 0x%02X (expected 0x68 or 0x72)\n", who);
        return -1;
    }

    /* Wake up (clear sleep bit), use X-axis gyro PLL */
    if (i2c_write_byte(dev->fd, MPU6050_PWR_MGMT_1, 0x01) < 0) {
        fprintf(stderr, "[mpu6050] Failed to wake up\n");
        return -1;
    }
    usleep(100000); /* 100ms stabilization */

    /* Configure DLPF and sample rate */
    uint16_t base_rate;
    uint8_t dlpf = pick_dlpf(cfg->sample_rate_hz, &base_rate);
    i2c_write_byte(dev->fd, MPU6050_CONFIG, dlpf);

    uint8_t divider = (uint8_t)((base_rate / cfg->sample_rate_hz) - 1);
    i2c_write_byte(dev->fd, MPU6050_SMPLRT_DIV, divider);

    /* Accel +/-2g, Gyro +/-250 deg/s */
    i2c_write_byte(dev->fd, MPU6050_ACCEL_CONFIG, 0x00);
    i2c_write_byte(dev->fd, MPU6050_GYRO_CONFIG, 0x00);

    dev->sample_rate_hz = cfg->sample_rate_hz;
    dev->beta = cfg->madgwick_beta;

    /* Reset quaternion */
    dev->q[0] = 1.0f; dev->q[1] = 0.0f; dev->q[2] = 0.0f; dev->q[3] = 0.0f;
    dev->last_ts = 0.0;

    /* ---- Gyro bias calibration ---- */
    fprintf(stderr, "[mpu6050] Calibrating gyro (%d samples, keep sensor still)...\n",
            cfg->calibration_samples);

    float bx = 0.0f, by = 0.0f, bz = 0.0f;
    int good = 0;
    uint32_t delay_us = 1000000 / cfg->sample_rate_hz;

    for (int i = 0; i < cfg->calibration_samples; i++) {
        uint8_t buf[14];
        if (i2c_read_bytes(dev->fd, MPU6050_ACCEL_XOUT_H, buf, 14) == 0) {
            int16_t gx = (int16_t)((buf[8] << 8) | buf[9]);
            int16_t gy = (int16_t)((buf[10] << 8) | buf[11]);
            int16_t gz = (int16_t)((buf[12] << 8) | buf[13]);
            bx += gx / GYRO_SCALE_250;
            by += gy / GYRO_SCALE_250;
            bz += gz / GYRO_SCALE_250;
            good++;
        }
        usleep(delay_us);
    }

    if (good > 0) {
        dev->gyro_bias[0] = bx / good;
        dev->gyro_bias[1] = by / good;
        dev->gyro_bias[2] = bz / good;
        fprintf(stderr, "[mpu6050] Gyro bias: [%.3f, %.3f, %.3f] deg/s (%d samples)\n",
                dev->gyro_bias[0], dev->gyro_bias[1], dev->gyro_bias[2], good);
    } else {
        dev->gyro_bias[0] = dev->gyro_bias[1] = dev->gyro_bias[2] = 0.0f;
        fprintf(stderr, "[mpu6050] Warning: gyro calibration failed, bias = 0\n");
    }

    return 0;
}

int mpu6050_read(mpu6050_t* dev, imu_data_t* data) {
    if (!dev || !data) return -1;

    uint8_t buf[14];
    if (i2c_read_bytes(dev->fd, MPU6050_ACCEL_XOUT_H, buf, 14) < 0) {
        return -1;
    }

    double now = get_time_sec();

    /* Accel (g) */
    int16_t ax = (int16_t)((buf[0] << 8) | buf[1]);
    int16_t ay = (int16_t)((buf[2] << 8) | buf[3]);
    int16_t az = (int16_t)((buf[4] << 8) | buf[5]);
    data->accel[0] = ax / ACCEL_SCALE_2G;
    data->accel[1] = ay / ACCEL_SCALE_2G;
    data->accel[2] = az / ACCEL_SCALE_2G;

    /* Temperature */
    int16_t temp_raw = (int16_t)((buf[6] << 8) | buf[7]);
    data->temperature = temp_raw / 340.0f + 36.53f;

    /* Gyro (deg/s) with bias removed */
    int16_t gx = (int16_t)((buf[8] << 8) | buf[9]);
    int16_t gy = (int16_t)((buf[10] << 8) | buf[11]);
    int16_t gz = (int16_t)((buf[12] << 8) | buf[13]);
    data->gyro[0] = gx / GYRO_SCALE_250 - dev->gyro_bias[0];
    data->gyro[1] = gy / GYRO_SCALE_250 - dev->gyro_bias[1];
    data->gyro[2] = gz / GYRO_SCALE_250 - dev->gyro_bias[2];

    /* Madgwick filter update */
    float dt;
    if (dev->last_ts > 0.0) {
        dt = (float)(now - dev->last_ts);
        if (dt <= 0.0f || dt > 1.0f) dt = 1.0f / dev->sample_rate_hz;
    } else {
        dt = 1.0f / dev->sample_rate_hz;
    }
    dev->last_ts = now;

    /* Feed Madgwick with gyro in rad/s */
    madgwick_update(dev->q, dev->beta, dt,
                    data->gyro[0] * DEG_TO_RAD,
                    data->gyro[1] * DEG_TO_RAD,
                    data->gyro[2] * DEG_TO_RAD,
                    data->accel[0], data->accel[1], data->accel[2]);

    /* Copy quaternion to output */
    data->quat[0] = dev->q[0];
    data->quat[1] = dev->q[1];
    data->quat[2] = dev->q[2];
    data->quat[3] = dev->q[3];

    data->timestamp = now;
    return 0;
}

void mpu6050_destroy(mpu6050_t* dev) {
    if (!dev) return;
    if (dev->fd >= 0) close(dev->fd);
    free(dev);
}

/* ---- Quaternion API ---- */

int mpu6050_get_quaternion(mpu6050_t* dev, float quat_out[4]) {
    if (!dev || !quat_out) return -1;
    quat_out[0] = dev->q[0];
    quat_out[1] = dev->q[1];
    quat_out[2] = dev->q[2];
    quat_out[3] = dev->q[3];
    return 0;
}

int mpu6050_reset_quaternion(mpu6050_t* dev) {
    if (!dev) return -1;
    dev->q[0] = 1.0f;
    dev->q[1] = 0.0f;
    dev->q[2] = 0.0f;
    dev->q[3] = 0.0f;
    dev->last_ts = 0.0;
    return 0;
}

void quat_to_euler(const float q[4], float* roll, float* pitch, float* yaw) {
    float qw = q[0], qx = q[1], qy = q[2], qz = q[3];

    /* Roll (X) */
    float sinr = 2.0f * (qw * qx + qy * qz);
    float cosr = 1.0f - 2.0f * (qx * qx + qy * qy);
    *roll = atan2f(sinr, cosr) / DEG_TO_RAD;

    /* Pitch (Y) */
    float sinp = 2.0f * (qw * qy - qz * qx);
    if (fabsf(sinp) >= 1.0f)
        *pitch = copysignf(90.0f, sinp);
    else
        *pitch = asinf(sinp) / DEG_TO_RAD;

    /* Yaw (Z) */
    float siny = 2.0f * (qw * qz + qx * qy);
    float cosy = 1.0f - 2.0f * (qy * qy + qz * qz);
    *yaw = atan2f(siny, cosy) / DEG_TO_RAD;
}
