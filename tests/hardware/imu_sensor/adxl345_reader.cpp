/*
 * ADXL345 Reader for Jetson Orin
 * Based on SparkFun ADXL345 Arduino Library
 * Adapted for Linux SPI
 */

#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include <cstring>
#include <cstdint>
#include <iomanip>

// ADXL345 Register addresses
#define ADXL345_DEVID          0x00
#define ADXL345_POWER_CTL      0x2D
#define ADXL345_DATA_FORMAT    0x31
#define ADXL345_DATAX0         0x32
#define ADXL345_BW_RATE        0x2C

// Configuration values
#define ADXL345_DEVICE_ID      0xE5
#define ADXL345_MEASURE        0x08
#define ADXL345_RANGE_2G       0x00
#define ADXL345_RANGE_4G       0x01
#define ADXL345_RANGE_8G       0x02
#define ADXL345_RANGE_16G      0x03

class ADXL345 {
private:
    int spi_fd;
    uint8_t spi_mode;
    uint8_t spi_bits;
    uint32_t spi_speed;
    uint8_t range;
    float scale_factor;

    void writeTo(uint8_t reg, uint8_t value) {
        uint8_t tx[2] = {reg, value};
        uint8_t rx[2] = {0};

        struct spi_ioc_transfer tr = {};
        tr.tx_buf = (unsigned long)tx;
        tr.rx_buf = (unsigned long)rx;
        tr.len = 2;
        tr.speed_hz = spi_speed;
        tr.bits_per_word = spi_bits;

        if (ioctl(spi_fd, SPI_IOC_MESSAGE(1), &tr) < 0) {
            std::cerr << "SPI write failed" << std::endl;
        }
    }

    void readFrom(uint8_t reg, uint8_t count, uint8_t* buffer) {
        // For SPI read, set bit 7 high and bit 6 high for multi-byte read
        uint8_t tx[1 + count];
        uint8_t rx[1 + count];

        memset(tx, 0, sizeof(tx));
        memset(rx, 0, sizeof(rx));

        tx[0] = reg | 0x80; // Read bit only (no multi-byte for single read)

        struct spi_ioc_transfer tr = {};
        tr.tx_buf = (unsigned long)tx;
        tr.rx_buf = (unsigned long)rx;
        tr.len = 1 + count;
        tr.speed_hz = spi_speed;
        tr.bits_per_word = spi_bits;

        if (ioctl(spi_fd, SPI_IOC_MESSAGE(1), &tr) < 0) {
            std::cerr << "SPI read failed" << std::endl;
        }

        // Debug output
        if (reg == ADXL345_DEVID) {
            std::cout << "DEBUG - Reading Device ID:" << std::endl;
            std::cout << "  TX: ";
            for (int i = 0; i < 1 + count; i++) {
                std::cout << "0x" << std::hex << std::setw(2) << std::setfill('0')
                          << (int)tx[i] << " ";
            }
            std::cout << std::endl << "  RX: ";
            for (int i = 0; i < 1 + count; i++) {
                std::cout << "0x" << std::hex << std::setw(2) << std::setfill('0')
                          << (int)rx[i] << " ";
            }
            std::cout << std::dec << std::endl;
        }

        // Copy data (skip first byte which is dummy)
        memcpy(buffer, rx + 1, count);
    }

    void setScaleFactor() {
        switch(range) {
            case ADXL345_RANGE_2G:
                scale_factor = 0.0039; // 3.9 mg/LSB
                break;
            case ADXL345_RANGE_4G:
                scale_factor = 0.0078; // 7.8 mg/LSB
                break;
            case ADXL345_RANGE_8G:
                scale_factor = 0.0156; // 15.6 mg/LSB
                break;
            case ADXL345_RANGE_16G:
                scale_factor = 0.0312; // 31.2 mg/LSB
                break;
            default:
                scale_factor = 0.0039;
        }
    }

public:
    ADXL345(const char* device = "/dev/spidev0.0") {
        spi_mode = SPI_MODE_0;  // Try Mode 0 (some modules need this)
        spi_bits = 8;
        spi_speed = 1000000;    // 1 MHz (slower speed)
        range = ADXL345_RANGE_2G;
        scale_factor = 0.0039;

        // Open SPI device
        spi_fd = open(device, O_RDWR);
        if (spi_fd < 0) {
            std::cerr << "Failed to open SPI device: " << device << std::endl;
            std::cerr << "Make sure SPI is enabled on your Jetson Orin" << std::endl;
            exit(1);
        }

        // Set SPI mode
        if (ioctl(spi_fd, SPI_IOC_WR_MODE, &spi_mode) < 0) {
            std::cerr << "Failed to set SPI mode" << std::endl;
            close(spi_fd);
            exit(1);
        }

        // Set bits per word
        if (ioctl(spi_fd, SPI_IOC_WR_BITS_PER_WORD, &spi_bits) < 0) {
            std::cerr << "Failed to set SPI bits per word" << std::endl;
            close(spi_fd);
            exit(1);
        }

        // Set max speed
        if (ioctl(spi_fd, SPI_IOC_WR_MAX_SPEED_HZ, &spi_speed) < 0) {
            std::cerr << "Failed to set SPI speed" << std::endl;
            close(spi_fd);
            exit(1);
        }
    }

    ~ADXL345() {
        if (spi_fd >= 0) {
            close(spi_fd);
        }
    }

    bool begin() {
        // Check device ID
        uint8_t devid;
        readFrom(ADXL345_DEVID, 1, &devid);

        if (devid != ADXL345_DEVICE_ID) {
            std::cerr << "ADXL345 not found. Expected 0x" << std::hex
                      << (int)ADXL345_DEVICE_ID << ", got 0x" << (int)devid << std::endl;
            return false;
        }

        std::cout << "ADXL345 detected! Device ID: 0x" << std::hex
                  << (int)devid << std::dec << std::endl;

        // Set range to ±2g
        setRange(ADXL345_RANGE_2G);

        // Set data rate to 100 Hz
        writeTo(ADXL345_BW_RATE, 0x0A);

        // Enable measurement mode
        writeTo(ADXL345_POWER_CTL, ADXL345_MEASURE);

        usleep(10000); // Wait 10ms for sensor to stabilize

        return true;
    }

    void setRange(uint8_t newRange) {
        range = newRange;
        writeTo(ADXL345_DATA_FORMAT, range);
        setScaleFactor();
    }

    void readAccel(int16_t& x, int16_t& y, int16_t& z) {
        uint8_t buffer[6];
        readFrom(ADXL345_DATAX0, 6, buffer);

        x = (int16_t)((buffer[1] << 8) | buffer[0]);
        y = (int16_t)((buffer[3] << 8) | buffer[2]);
        z = (int16_t)((buffer[5] << 8) | buffer[4]);
    }

    void readAccelG(float& x, float& y, float& z) {
        int16_t x_raw, y_raw, z_raw;
        readAccel(x_raw, y_raw, z_raw);

        x = x_raw * scale_factor;
        y = y_raw * scale_factor;
        z = z_raw * scale_factor;
    }
};

int main() {
    std::cout << "=== ADXL345 IMU Reader for Jetson Orin ===" << std::endl;
    std::cout << "SPI Pin Connections:" << std::endl;
    std::cout << "  CS   -> Pin 24 (SPI0_CS0)" << std::endl;
    std::cout << "  SCL  -> Pin 23 (SPI0_SCK)" << std::endl;
    std::cout << "  SDA  -> Pin 19 (SPI0_MOSI)" << std::endl;
    std::cout << "  SDO  -> Pin 21 (SPI0_MISO)" << std::endl;
    std::cout << "===========================================" << std::endl << std::endl;

    ADXL345 accel;

    if (!accel.begin()) {
        std::cerr << "Failed to initialize ADXL345!" << std::endl;
        std::cerr << "\nTroubleshooting:" << std::endl;
        std::cerr << "1. Check if SPI is enabled: ls /dev/spidev*" << std::endl;
        std::cerr << "2. Enable SPI: sudo /opt/nvidia/jetson-io/jetson-io.py" << std::endl;
        std::cerr << "3. Verify wiring connections" << std::endl;
        std::cerr << "4. Check power supply to ADXL345 (3.3V)" << std::endl;
        return 1;
    }

    std::cout << "\nReading accelerometer data (Ctrl+C to stop)...\n" << std::endl;
    std::cout << std::fixed << std::setprecision(3);

    while (true) {
        float x, y, z;
        accel.readAccelG(x, y, z);

        std::cout << "\rX: " << std::setw(7) << x << " g  "
                  << "Y: " << std::setw(7) << y << " g  "
                  << "Z: " << std::setw(7) << z << " g    " << std::flush;

        usleep(100000); // 100ms delay (10 Hz update)
    }

    return 0;
}
