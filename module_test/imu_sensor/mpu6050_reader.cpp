/*
 * MPU6050 (GY-521) I2C Reader for Jetson Orin
 * 6-axis IMU: 3-axis accelerometer + 3-axis gyroscope
 */

#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <cstring>
#include <cstdint>
#include <iomanip>
#include <cmath>

// MPU6050 Register addresses
#define MPU6050_WHO_AM_I       0x75
#define MPU6050_PWR_MGMT_1     0x6B
#define MPU6050_GYRO_CONFIG    0x1B
#define MPU6050_ACCEL_CONFIG   0x1C
#define MPU6050_ACCEL_XOUT_H   0x3B
#define MPU6050_GYRO_XOUT_H    0x43
#define MPU6050_TEMP_OUT_H     0x41
#define MPU6050_SIGNAL_PATH_RESET 0x68
#define MPU6050_USER_CTRL      0x6A

// Configuration values
#define MPU6050_DEVICE_ID      0x68
#define MPU6050_I2C_ADDR       0x68  // AD0 = GND
#define MPU6050_I2C_ADDR_ALT   0x69  // AD0 = VCC

// Scale factors
#define ACCEL_SCALE_2G         16384.0  // LSB/g
#define GYRO_SCALE_250         131.0    // LSB/(°/s)

class MPU6050 {
private:
    int i2c_fd;
    uint8_t i2c_addr;
    float gx_offset, gy_offset, gz_offset;
    float ax_offset, ay_offset, az_offset;

    bool writeByte(uint8_t reg, uint8_t value) {
        uint8_t buffer[2] = {reg, value};
        if (write(i2c_fd, buffer, 2) != 2) {
            return false;
        }
        return true;
    }

    bool readBytes(uint8_t reg, uint8_t count, uint8_t* buffer) {
        if (write(i2c_fd, &reg, 1) != 1) {
            return false;
        }
        if (read(i2c_fd, buffer, count) != (ssize_t)count) {
            return false;
        }
        return true;
    }

    uint8_t readByte(uint8_t reg) {
        uint8_t value = 0;
        readBytes(reg, 1, &value);
        return value;
    }

public:
    MPU6050(const char* device, uint8_t addr)
        : gx_offset(0), gy_offset(0), gz_offset(0),
          ax_offset(0), ay_offset(0), az_offset(0) {
        i2c_addr = addr;
        i2c_fd = open(device, O_RDWR);
        if (i2c_fd < 0) {
            std::cerr << "Failed to open I2C device" << std::endl;
            exit(1);
        }

        if (ioctl(i2c_fd, I2C_SLAVE, i2c_addr) < 0) {
            std::cerr << "Failed to set I2C address" << std::endl;
            close(i2c_fd);
            exit(1);
        }
    }

    ~MPU6050() {
        if (i2c_fd >= 0) {
            close(i2c_fd);
        }
    }

    bool begin() {
        // Check WHO_AM_I register
        uint8_t who_am_i = readByte(MPU6050_WHO_AM_I);

        std::cout << "WHO_AM_I: 0x" << std::hex << (int)who_am_i << std::dec << std::endl;

        if (who_am_i != MPU6050_DEVICE_ID && who_am_i != 0x72) {
            std::cerr << "MPU6050 not found. Expected 0x68 or 0x72, got 0x"
                      << std::hex << (int)who_am_i << std::dec << std::endl;
            return false;
        }

        std::cout << "MPU6050 detected!" << std::endl;

        // Wake up the MPU6050 (clear sleep bit)
        if (!writeByte(MPU6050_PWR_MGMT_1, 0x00)) {
            std::cerr << "Failed to wake up MPU6050" << std::endl;
            return false;
        }
        usleep(100000);

        // Configure accelerometer (±2g)
        writeByte(MPU6050_ACCEL_CONFIG, 0x00);

        // Configure gyroscope (±250°/s)
        writeByte(MPU6050_GYRO_CONFIG, 0x00);

        usleep(50000);

        // Gyro offset calibration: sample 200 readings while stationary
        calibrate(200);

        return true;
    }

    void calibrate(int samples) {
        std::cout << "Calibrating (keep sensor still)... " << std::flush;

        double sum_gx = 0, sum_gy = 0, sum_gz = 0;
        double sum_ax = 0, sum_ay = 0, sum_az = 0;
        int valid = 0;

        for (int i = 0; i < samples; i++) {
            uint8_t buffer[14];
            if (!readBytes(MPU6050_ACCEL_XOUT_H, 14, buffer)) continue;

            int16_t ax_raw = (buffer[0] << 8) | buffer[1];
            int16_t ay_raw = (buffer[2] << 8) | buffer[3];
            int16_t az_raw = (buffer[4] << 8) | buffer[5];
            int16_t gx_raw = (buffer[8] << 8) | buffer[9];
            int16_t gy_raw = (buffer[10] << 8) | buffer[11];
            int16_t gz_raw = (buffer[12] << 8) | buffer[13];

            sum_ax += ax_raw / ACCEL_SCALE_2G;
            sum_ay += ay_raw / ACCEL_SCALE_2G;
            sum_az += az_raw / ACCEL_SCALE_2G;
            sum_gx += gx_raw / GYRO_SCALE_250;
            sum_gy += gy_raw / GYRO_SCALE_250;
            sum_gz += gz_raw / GYRO_SCALE_250;
            valid++;

            usleep(10000); // 10ms between samples
        }

        if (valid > 0) {
            // Gyro: offset = average (should be 0 at rest)
            gx_offset = sum_gx / valid;
            gy_offset = sum_gy / valid;
            gz_offset = sum_gz / valid;

            // Accel: offset = average - gravity component
            // We don't know orientation, so just store raw average
            // User can interpret calibrated accel relative to startup pose
            ax_offset = sum_ax / valid;
            ay_offset = sum_ay / valid;
            az_offset = sum_az / valid;

            std::cout << "done (" << valid << " samples)" << std::endl;
            std::cout << "  Gyro offset:  X=" << std::fixed << std::setprecision(2)
                      << gx_offset << " Y=" << gy_offset << " Z=" << gz_offset << " deg/s" << std::endl;
            std::cout << "  Accel offset: X=" << ax_offset << " Y=" << ay_offset << " Z=" << az_offset << " g" << std::endl;
        }
    }

    void readAccelGyro(float& ax, float& ay, float& az,
                       float& gx, float& gy, float& gz) {
        uint8_t buffer[14];

        if (!readBytes(MPU6050_ACCEL_XOUT_H, 14, buffer)) {
            std::cerr << "Failed to read sensor data" << std::endl;
            return;
        }

        // Combine high and low bytes
        int16_t ax_raw = (buffer[0] << 8) | buffer[1];
        int16_t ay_raw = (buffer[2] << 8) | buffer[3];
        int16_t az_raw = (buffer[4] << 8) | buffer[5];
        int16_t gx_raw = (buffer[8] << 8) | buffer[9];
        int16_t gy_raw = (buffer[10] << 8) | buffer[11];
        int16_t gz_raw = (buffer[12] << 8) | buffer[13];

        // Convert to physical units and subtract calibration offset
        ax = ax_raw / ACCEL_SCALE_2G - ax_offset;
        ay = ay_raw / ACCEL_SCALE_2G - ay_offset;
        az = az_raw / ACCEL_SCALE_2G - az_offset;
        gx = gx_raw / GYRO_SCALE_250 - gx_offset;
        gy = gy_raw / GYRO_SCALE_250 - gy_offset;
        gz = gz_raw / GYRO_SCALE_250 - gz_offset;
    }

    float readTemperature() {
        uint8_t buffer[2];
        if (!readBytes(MPU6050_TEMP_OUT_H, 2, buffer)) {
            return 0.0;
        }
        int16_t temp_raw = (buffer[0] << 8) | buffer[1];
        return temp_raw / 340.0 + 36.53;
    }
};

void scanI2CBus(const char* device, int bus_num) {
    std::cout << "\n=== Scanning I2C Bus " << bus_num << " ===" << std::endl;

    int fd = open(device, O_RDWR);
    if (fd < 0) {
        std::cerr << "Cannot open " << device << std::endl;
        return;
    }

    bool found = false;
    for (uint8_t addr = 0x68; addr <= 0x69; addr++) {
        if (ioctl(fd, I2C_SLAVE, addr) >= 0) {
            uint8_t reg = 0x75; // WHO_AM_I
            if (write(fd, &reg, 1) == 1) {
                uint8_t data;
                if (read(fd, &data, 1) == 1) {
                    std::cout << "  Found device at 0x" << std::hex << (int)addr
                              << " (WHO_AM_I: 0x" << (int)data << ")" << std::dec << std::endl;
                    found = true;
                }
            }
        }
    }

    if (!found) {
        std::cout << "  No MPU6050 found" << std::endl;
    }

    close(fd);
}

int main() {
    std::cout << "=== MPU6050 (GY-521) IMU Reader for Jetson Orin ===" << std::endl;
    std::cout << "I2C Pin Connections:" << std::endl;
    std::cout << "  VCC  -> Pin 1 (3.3V)" << std::endl;
    std::cout << "  GND  -> Pin 6 (GND)" << std::endl;
    std::cout << "  SCL  -> Pin 5 (I2C1_SCL)" << std::endl;
    std::cout << "  SDA  -> Pin 3 (I2C1_SDA)" << std::endl;
    std::cout << "  AD0  -> GND (I2C address = 0x68)" << std::endl;
    std::cout << "====================================================" << std::endl;

    // Scan common I2C buses
    std::cout << "\nScanning for MPU6050..." << std::endl;
    scanI2CBus("/dev/i2c-0", 0);
    scanI2CBus("/dev/i2c-1", 1);
    scanI2CBus("/dev/i2c-2", 2);
    scanI2CBus("/dev/i2c-7", 7);
    scanI2CBus("/dev/i2c-8", 8);

    // Try to initialize MPU6050
    std::cout << "\n=== Attempting to initialize MPU6050 ===" << std::endl;

    const char* i2c_devices[] = {"/dev/i2c-1", "/dev/i2c-0", "/dev/i2c-2", "/dev/i2c-7", "/dev/i2c-8"};
    const uint8_t i2c_addrs[] = {MPU6050_I2C_ADDR, MPU6050_I2C_ADDR_ALT};

    MPU6050* sensor = nullptr;
    bool initialized = false;

    for (const char* dev : i2c_devices) {
        for (uint8_t addr : i2c_addrs) {
            std::cout << "\nTrying " << dev << " at address 0x" << std::hex
                      << (int)addr << std::dec << "..." << std::endl;

            try {
                sensor = new MPU6050(dev, addr);
                if (sensor->begin()) {
                    initialized = true;
                    std::cout << "✓ MPU6050 initialized successfully!" << std::endl;
                    break;
                }
                delete sensor;
                sensor = nullptr;
            } catch (...) {
                if (sensor) {
                    delete sensor;
                    sensor = nullptr;
                }
            }
        }
        if (initialized) break;
    }

    if (!initialized) {
        std::cerr << "\n✗ Failed to initialize MPU6050!" << std::endl;
        std::cerr << "\nTroubleshooting:" << std::endl;
        std::cerr << "1. Check power: VCC should be 3.3V" << std::endl;
        std::cerr << "2. Check ground: GND connected" << std::endl;
        std::cerr << "3. Verify I2C connections (Pin 3 = SDA, Pin 5 = SCL)" << std::endl;
        std::cerr << "4. Check AD0 pin: connect to GND for address 0x68" << std::endl;
        return 1;
    }

    // Read and display sensor data
    std::cout << "\n=== Reading IMU Data (Ctrl+C to stop) ===" << std::endl;
    std::cout << std::fixed << std::setprecision(3);

    while (true) {
        float ax, ay, az, gx, gy, gz;
        sensor->readAccelGyro(ax, ay, az, gx, gy, gz);

        std::cout << "\rAccel: X=" << std::setw(6) << ax << "g "
                  << "Y=" << std::setw(6) << ay << "g "
                  << "Z=" << std::setw(6) << az << "g  |  "
                  << "Gyro: X=" << std::setw(7) << gx << "°/s "
                  << "Y=" << std::setw(7) << gy << "°/s "
                  << "Z=" << std::setw(7) << gz << "°/s    " << std::flush;

        usleep(100000); // 100ms delay (10 Hz)
    }

    delete sensor;
    return 0;
}
