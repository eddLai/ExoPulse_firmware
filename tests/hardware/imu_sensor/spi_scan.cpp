#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include <cstring>

void test_spi_device(const char* device) {
    std::cout << "\n=== Testing " << device << " ===" << std::endl;

    int spi_fd = open(device, O_RDWR);
    if (spi_fd < 0) {
        std::cerr << "Failed to open " << device << std::endl;
        return;
    }

    // Configure SPI - try both Mode 0 and Mode 3
    for (int m = 0; m <= 3; m++) {
        std::cout << "  Mode " << m << ": ";

        uint8_t mode = m;
        uint8_t bits = 8;
        uint32_t speed = 1000000;

        ioctl(spi_fd, SPI_IOC_WR_MODE, &mode);
        ioctl(spi_fd, SPI_IOC_WR_BITS_PER_WORD, &bits);
        ioctl(spi_fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed);

        // Try to read ADXL345 Device ID (register 0x00)
        uint8_t tx[2] = {0x80, 0x00};  // Read register 0x00
        uint8_t rx[2] = {0};

        struct spi_ioc_transfer tr = {};
        tr.tx_buf = (unsigned long)tx;
        tr.rx_buf = (unsigned long)rx;
        tr.len = 2;
        tr.speed_hz = speed;
        tr.bits_per_word = bits;

        if (ioctl(spi_fd, SPI_IOC_MESSAGE(1), &tr) >= 0) {
            if (rx[1] == 0xE5) {
                std::cout << "✓ ADXL345 FOUND! (0xE5)" << std::endl;
            } else {
                std::cout << "RX=0x" << std::hex << (int)rx[1] << std::dec << std::endl;
            }
        } else {
            std::cout << "Transfer failed" << std::endl;
        }
    }

    close(spi_fd);
}

int main() {
    std::cout << "=== ADXL345 SPI Device Scanner ===" << std::endl;

    test_spi_device("/dev/spidev0.0");
    test_spi_device("/dev/spidev0.1");
    test_spi_device("/dev/spidev2.0");
    test_spi_device("/dev/spidev2.1");

    std::cout << "\n=== Scan Complete ===" << std::endl;
    return 0;
}
