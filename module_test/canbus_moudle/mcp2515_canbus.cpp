/*
 * MCP2515 CAN Bus Reader/Writer for Jetson Orin
 * Uses SPI interface via /dev/spidev0.0
 *
 * Hardware: MCP2515 + TJA1050 CAN Transceiver Module
 * Reference: ESP32 MGv2 project (uses coryjfowler/MCP_CAN_lib)
 *
 * SPI Pin Connections (Jetson Orin 40-pin header):
 *   CS   -> Pin 24 (SPI0_CS0)
 *   SCK  -> Pin 23 (SPI0_SCK)
 *   MOSI -> Pin 19 (SPI0_MOSI)
 *   MISO -> Pin 21 (SPI0_MISO)
 *   VCC  -> Pin 2 or 4 (5V)
 *   GND  -> Pin 25 (or Pin 6, 9)
 */

#include <iostream>
#include <iomanip>
#include <cstdint>
#include <cstring>
#include <cstdlib>
#include <csignal>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include <chrono>
#include <thread>

// ============================================================
// MCP2515 Register Definitions
// ============================================================

// SPI Instructions
#define MCP2515_RESET       0xC0
#define MCP2515_READ        0x03
#define MCP2515_WRITE       0x02
#define MCP2515_READ_STATUS 0xA0
#define MCP2515_RX_STATUS   0xB0
#define MCP2515_BIT_MODIFY  0x05
#define MCP2515_READ_RX0    0x90
#define MCP2515_READ_RX1    0x94
#define MCP2515_LOAD_TX0    0x40
#define MCP2515_LOAD_TX1    0x42
#define MCP2515_LOAD_TX2    0x44
#define MCP2515_RTS_TX0     0x81
#define MCP2515_RTS_TX1     0x82
#define MCP2515_RTS_TX2     0x84

// Control Registers
#define CANSTAT     0x0E
#define CANCTRL     0x0F
#define CNF1        0x2A
#define CNF2        0x29
#define CNF3        0x28
#define CANINTE     0x2B
#define CANINTF     0x2C
#define EFLG        0x2D
#define TEC         0x1C
#define REC         0x1D

// TX Buffer 0 Registers
#define TXB0CTRL    0x30
#define TXB0SIDH    0x31
#define TXB0SIDL    0x32
#define TXB0DLC     0x35
#define TXB0D0      0x36

// RX Buffer 0/1 Registers
#define RXB0CTRL    0x60
#define RXB0SIDH    0x61
#define RXB0SIDL    0x62
#define RXB0DLC     0x65
#define RXB0D0      0x66
#define RXB1CTRL    0x70

// Filter/Mask Registers
#define RXF0SIDH    0x00
#define RXF0SIDL    0x01
#define RXM0SIDH    0x20
#define RXM0SIDL    0x21

// MCP2515 Modes
#define MODE_NORMAL     0x00
#define MODE_SLEEP      0x20
#define MODE_LOOPBACK   0x40
#define MODE_LISTENONLY 0x60
#define MODE_CONFIG     0x80
#define MODE_MASK       0xE0

// Interrupt Flags
#define RX0IF   0x01
#define RX1IF   0x02
#define TX0IF   0x04
#define ERRIF   0x20
#define MERRF   0x80

// ============================================================
// CAN Baud Rate Configurations (for 8MHz crystal)
// CNF1, CNF2, CNF3 values
// ============================================================
struct BaudConfig {
    const char* name;
    uint32_t baud;
    uint8_t cnf1, cnf2, cnf3;
};

// Common baud rates for 8MHz oscillator
static const BaudConfig BAUD_CONFIGS_8MHZ[] = {
    {"1Mbps",   1000000, 0x00, 0x80, 0x00},
    {"500Kbps",  500000, 0x00, 0x90, 0x02},
    {"250Kbps",  250000, 0x00, 0xB1, 0x05},
    {"125Kbps",  125000, 0x01, 0xB1, 0x05},
    {"100Kbps",  100000, 0x01, 0xB4, 0x06},
};

static volatile bool running = true;

void signalHandler(int sig) {
    (void)sig;
    running = false;
}

// ============================================================
// MCP2515 Driver Class
// ============================================================
class MCP2515 {
private:
    int spi_fd;
    uint32_t spi_speed;
    const char* spi_device;

    bool spiTransfer(uint8_t* tx, uint8_t* rx, size_t len) {
        struct spi_ioc_transfer transfer = {};
        transfer.tx_buf = (unsigned long)tx;
        transfer.rx_buf = (unsigned long)rx;
        transfer.len = len;
        transfer.speed_hz = spi_speed;
        transfer.bits_per_word = 8;

        if (ioctl(spi_fd, SPI_IOC_MESSAGE(1), &transfer) < 0) {
            std::cerr << "SPI transfer failed" << std::endl;
            return false;
        }
        return true;
    }

    uint8_t readRegister(uint8_t addr) {
        uint8_t tx[3] = {MCP2515_READ, addr, 0x00};
        uint8_t rx[3] = {0};
        spiTransfer(tx, rx, 3);
        return rx[2];
    }

    void writeRegister(uint8_t addr, uint8_t value) {
        uint8_t tx[3] = {MCP2515_WRITE, addr, value};
        uint8_t rx[3] = {0};
        spiTransfer(tx, rx, 3);
    }

    void bitModify(uint8_t addr, uint8_t mask, uint8_t data) {
        uint8_t tx[4] = {MCP2515_BIT_MODIFY, addr, mask, data};
        uint8_t rx[4] = {0};
        spiTransfer(tx, rx, 4);
    }

    void reset() {
        uint8_t tx[1] = {MCP2515_RESET};
        uint8_t rx[1] = {0};
        spiTransfer(tx, rx, 1);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    bool setMode(uint8_t mode) {
        bitModify(CANCTRL, MODE_MASK, mode);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));

        uint8_t actual = readRegister(CANSTAT) & MODE_MASK;
        if (actual != mode) {
            std::cerr << "  Mode set failed: expected 0x" << std::hex << (int)mode
                      << " got 0x" << (int)actual << std::dec << std::endl;
            return false;
        }
        return true;
    }

public:
    MCP2515(const char* device, uint32_t speed = 1000000)
        : spi_fd(-1), spi_speed(speed), spi_device(device) {}

    ~MCP2515() {
        if (spi_fd >= 0) {
            close(spi_fd);
        }
    }

    bool begin() {
        // Open SPI device
        spi_fd = open(spi_device, O_RDWR);
        if (spi_fd < 0) {
            std::cerr << "Failed to open SPI device: " << spi_device << std::endl;
            std::cerr << "Ensure SPI is enabled: sudo /opt/nvidia/jetson-io/jetson-io.py" << std::endl;
            return false;
        }

        // Configure SPI
        uint8_t mode = SPI_MODE_0;
        uint8_t bits = 8;

        ioctl(spi_fd, SPI_IOC_WR_MODE, &mode);
        ioctl(spi_fd, SPI_IOC_WR_BITS_PER_WORD, &bits);
        ioctl(spi_fd, SPI_IOC_WR_MAX_SPEED_HZ, &spi_speed);

        std::cout << "  SPI device opened: " << spi_device << std::endl;
        std::cout << "  SPI speed: " << spi_speed / 1000 << " KHz" << std::endl;
        std::cout << "  SPI mode: " << (int)mode << std::endl;

        // Reset MCP2515
        std::cout << "\n[2] Resetting MCP2515..." << std::endl;
        reset();

        // Verify we're in config mode after reset
        uint8_t stat = readRegister(CANSTAT) & MODE_MASK;
        if (stat != MODE_CONFIG) {
            std::cerr << "  MCP2515 not in config mode after reset!" << std::endl;
            std::cerr << "  CANSTAT = 0x" << std::hex << (int)stat << std::dec << std::endl;
            std::cerr << "  Check SPI wiring and power!" << std::endl;
            return false;
        }
        std::cout << "  MCP2515 reset OK (in config mode)" << std::endl;

        return true;
    }

    bool configureBaudRate(uint32_t baud) {
        // Must be in config mode
        if ((readRegister(CANSTAT) & MODE_MASK) != MODE_CONFIG) {
            if (!setMode(MODE_CONFIG)) return false;
        }

        // Find matching baud rate config
        const BaudConfig* cfg = nullptr;
        for (const auto& bc : BAUD_CONFIGS_8MHZ) {
            if (bc.baud == baud) {
                cfg = &bc;
                break;
            }
        }

        if (!cfg) {
            std::cerr << "  Unsupported baud rate: " << baud << std::endl;
            return false;
        }

        std::cout << "  Setting CAN baud rate: " << cfg->name << " (8MHz crystal)" << std::endl;

        writeRegister(CNF1, cfg->cnf1);
        writeRegister(CNF2, cfg->cnf2);
        writeRegister(CNF3, cfg->cnf3);

        // Verify
        uint8_t r1 = readRegister(CNF1);
        uint8_t r2 = readRegister(CNF2);
        uint8_t r3 = readRegister(CNF3);

        if (r1 != cfg->cnf1 || r2 != cfg->cnf2 || r3 != cfg->cnf3) {
            std::cerr << "  Baud rate config verification failed!" << std::endl;
            return false;
        }

        std::cout << "  CNF1=0x" << std::hex << (int)r1
                  << " CNF2=0x" << (int)r2
                  << " CNF3=0x" << (int)r3 << std::dec << std::endl;

        return true;
    }

    void configureFilters(bool acceptAll) {
        if (acceptAll) {
            // Accept all messages: set mask to 0x000
            writeRegister(RXM0SIDH, 0x00);
            writeRegister(RXM0SIDL, 0x00);
            // RXB0: receive any message, rollover to RXB1
            writeRegister(RXB0CTRL, 0x64);
            writeRegister(RXB1CTRL, 0x60);
            std::cout << "  Filters: Accept ALL messages" << std::endl;
        }
    }

    void enableInterrupts() {
        // Enable RX0 and RX1 interrupts
        writeRegister(CANINTE, RX0IF | RX1IF);
        // Clear all interrupt flags
        writeRegister(CANINTF, 0x00);
    }

    bool setNormalMode() {
        std::cout << "\n[4] Setting NORMAL mode..." << std::endl;
        if (!setMode(MODE_NORMAL)) return false;
        std::cout << "  Mode set to NORMAL" << std::endl;
        return true;
    }

    bool setLoopbackMode() {
        std::cout << "\n[4] Setting LOOPBACK mode (self-test)..." << std::endl;
        if (!setMode(MODE_LOOPBACK)) return false;
        std::cout << "  Mode set to LOOPBACK" << std::endl;
        return true;
    }

    bool setListenOnlyMode() {
        std::cout << "\n[4] Setting LISTEN-ONLY mode..." << std::endl;
        if (!setMode(MODE_LISTENONLY)) return false;
        std::cout << "  Mode set to LISTEN-ONLY" << std::endl;
        return true;
    }

    // Send a standard CAN frame (11-bit ID)
    bool sendMessage(uint16_t id, uint8_t* data, uint8_t len) {
        if (len > 8) len = 8;

        // Check if TX buffer 0 is available
        uint8_t ctrl = readRegister(TXB0CTRL);
        if (ctrl & 0x08) {
            // TX pending, wait a bit
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            ctrl = readRegister(TXB0CTRL);
            if (ctrl & 0x08) {
                std::cerr << "TX buffer busy" << std::endl;
                return false;
            }
        }

        // Set standard ID
        writeRegister(TXB0SIDH, (id >> 3) & 0xFF);
        writeRegister(TXB0SIDL, (id << 5) & 0xE0);  // Standard frame

        // Set data length
        writeRegister(TXB0DLC, len);

        // Write data bytes
        for (uint8_t i = 0; i < len; i++) {
            writeRegister(TXB0D0 + i, data[i]);
        }

        // Request to send
        uint8_t tx[1] = {MCP2515_RTS_TX0};
        uint8_t rx[1] = {0};
        spiTransfer(tx, rx, 1);

        // Wait for transmit complete
        for (int i = 0; i < 10; i++) {
            ctrl = readRegister(TXB0CTRL);
            if (!(ctrl & 0x08)) {
                // Clear TX interrupt flag
                bitModify(CANINTF, TX0IF, 0x00);
                return true;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }

        std::cerr << "TX timeout" << std::endl;
        return false;
    }

    // Receive a CAN frame
    bool receiveMessage(uint16_t& id, uint8_t* data, uint8_t& len) {
        uint8_t intf = readRegister(CANINTF);

        if (intf & RX0IF) {
            // Read from RX buffer 0
            uint8_t sidh = readRegister(RXB0SIDH);
            uint8_t sidl = readRegister(RXB0SIDL);
            id = (sidh << 3) | (sidl >> 5);

            len = readRegister(RXB0DLC) & 0x0F;
            if (len > 8) len = 8;

            for (uint8_t i = 0; i < len; i++) {
                data[i] = readRegister(RXB0D0 + i);
            }

            // Clear RX0 interrupt flag
            bitModify(CANINTF, RX0IF, 0x00);
            return true;
        }

        if (intf & RX1IF) {
            // Read from RX buffer 1
            uint8_t sidh = readRegister(0x71);  // RXB1SIDH
            uint8_t sidl = readRegister(0x72);  // RXB1SIDL
            id = (sidh << 3) | (sidl >> 5);

            len = readRegister(0x75) & 0x0F;   // RXB1DLC
            if (len > 8) len = 8;

            for (uint8_t i = 0; i < len; i++) {
                data[i] = readRegister(0x76 + i);  // RXB1D0+
            }

            // Clear RX1 interrupt flag
            bitModify(CANINTF, RX1IF, 0x00);
            return true;
        }

        return false;  // No message available
    }

    // Get error counts
    void getErrorCounts(uint8_t& txErr, uint8_t& rxErr) {
        txErr = readRegister(TEC);
        rxErr = readRegister(REC);
    }

    // Get error flags
    uint8_t getErrorFlags() {
        return readRegister(EFLG);
    }

    // Read status register
    uint8_t readStatus() {
        uint8_t tx[2] = {MCP2515_READ_STATUS, 0x00};
        uint8_t rx[2] = {0};
        spiTransfer(tx, rx, 2);
        return rx[1];
    }

    void printStatus() {
        uint8_t txErr, rxErr;
        getErrorCounts(txErr, rxErr);
        uint8_t eflg = getErrorFlags();

        std::cout << "  TXErr=" << (int)txErr
                  << " RXErr=" << (int)rxErr
                  << " EFLG=0x" << std::hex << (int)eflg << std::dec;

        if (eflg & 0x01) std::cout << " [EWARN]";
        if (eflg & 0x02) std::cout << " [RXWAR]";
        if (eflg & 0x04) std::cout << " [TXWAR]";
        if (eflg & 0x08) std::cout << " [RXEP]";
        if (eflg & 0x10) std::cout << " [TXEP]";
        if (eflg & 0x20) std::cout << " [TXBO]";
        if (eflg & 0x40) std::cout << " [RX0OVR]";
        if (eflg & 0x80) std::cout << " [RX1OVR]";

        std::cout << std::endl;
    }
};

// ============================================================
// Usage & Help
// ============================================================
void printUsage(const char* progName) {
    std::cout << "Usage: " << progName << " [OPTIONS]\n"
              << "\nOptions:\n"
              << "  -d <device>   SPI device (default: /dev/spidev0.0)\n"
              << "  -b <baud>     CAN baud rate: 1000000, 500000, 250000, 125000, 100000\n"
              << "                (default: 1000000)\n"
              << "  -m <mode>     Mode: receive, send, loopback (default: receive)\n"
              << "  -i <id>       CAN ID for send mode (hex, default: 0x100)\n"
              << "  -h            Show this help\n"
              << "\nExamples:\n"
              << "  " << progName << "                        # Receive all CAN messages at 1Mbps\n"
              << "  " << progName << " -b 500000              # Receive at 500Kbps\n"
              << "  " << progName << " -m send -i 0x141       # Send test messages to ID 0x141\n"
              << "  " << progName << " -m loopback            # Self-test (no CAN bus needed)\n"
              << std::endl;
}

// ============================================================
// Main
// ============================================================
int main(int argc, char* argv[]) {
    // Default parameters
    const char* spiDevice = "/dev/spidev0.0";
    uint32_t baudRate = 1000000;
    std::string mode = "receive";
    uint16_t sendId = 0x100;

    // Parse arguments
    for (int i = 1; i < argc; i++) {
        std::string arg = argv[i];
        if (arg == "-d" && i + 1 < argc) {
            spiDevice = argv[++i];
        } else if (arg == "-b" && i + 1 < argc) {
            baudRate = std::stoul(argv[++i]);
        } else if (arg == "-m" && i + 1 < argc) {
            mode = argv[++i];
        } else if (arg == "-i" && i + 1 < argc) {
            sendId = std::stoul(argv[++i], nullptr, 16);
        } else if (arg == "-h") {
            printUsage(argv[0]);
            return 0;
        }
    }

    signal(SIGINT, signalHandler);

    std::cout << "=== MCP2515 CAN Bus Module Test for Jetson Orin ===" << std::endl;
    std::cout << "SPI Pin Connections:" << std::endl;
    std::cout << "  CS   -> Pin 24 (SPI0_CS0)" << std::endl;
    std::cout << "  SCK  -> Pin 23 (SPI0_SCK)" << std::endl;
    std::cout << "  MOSI -> Pin 19 (SPI0_MOSI)" << std::endl;
    std::cout << "  MISO -> Pin 21 (SPI0_MISO)" << std::endl;
    std::cout << "  VCC  -> Pin 2/4 (5V)" << std::endl;
    std::cout << "  GND  -> Pin 25 (or 6, 9)" << std::endl;
    std::cout << "====================================================" << std::endl;

    // Initialize MCP2515
    std::cout << "\n[1] Opening SPI device..." << std::endl;
    MCP2515 can(spiDevice);

    if (!can.begin()) {
        return 1;
    }

    // Configure baud rate
    std::cout << "\n[3] Configuring CAN baud rate..." << std::endl;
    if (!can.configureBaudRate(baudRate)) {
        return 1;
    }

    // Configure filters and interrupts
    can.configureFilters(true);  // Accept all
    can.enableInterrupts();

    // Set operating mode
    if (mode == "loopback") {
        if (!can.setLoopbackMode()) return 1;
    } else if (mode == "listen") {
        if (!can.setListenOnlyMode()) return 1;
    } else {
        if (!can.setNormalMode()) return 1;
    }

    std::cout << "\n[OK] MCP2515 CAN Bus ready!" << std::endl;
    std::cout << "Mode: " << mode << " | Baud: " << baudRate << std::endl;
    std::cout << "========================================\n" << std::endl;

    // ---- Receive Mode ----
    if (mode == "receive" || mode == "listen") {
        std::cout << "Waiting for CAN messages (Ctrl+C to stop)...\n" << std::endl;

        uint32_t rxCount = 0;
        auto startTime = std::chrono::steady_clock::now();

        while (running) {
            uint16_t id;
            uint8_t data[8];
            uint8_t len;

            if (can.receiveMessage(id, data, len)) {
                rxCount++;
                auto now = std::chrono::steady_clock::now();
                auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - startTime).count();

                std::cout << "[RX] #" << std::setw(5) << rxCount
                          << " @" << std::setw(8) << elapsed << "ms"
                          << " ID=0x" << std::hex << std::setw(3) << std::setfill('0') << id
                          << std::setfill(' ') << std::dec
                          << " LEN=" << (int)len << " Data: ";

                for (uint8_t i = 0; i < len; i++) {
                    std::cout << std::hex << std::setw(2) << std::setfill('0')
                              << (int)data[i] << std::setfill(' ') << std::dec;
                    if (i < len - 1) std::cout << " ";
                }
                std::cout << std::endl;
            }

            std::this_thread::sleep_for(std::chrono::microseconds(100));
        }

        std::cout << "\n--- Results ---" << std::endl;
        std::cout << "Total messages received: " << rxCount << std::endl;
        can.printStatus();
    }

    // ---- Send Mode ----
    else if (mode == "send") {
        std::cout << "Sending CAN messages to ID=0x" << std::hex << sendId
                  << std::dec << " every 500ms (Ctrl+C to stop)...\n" << std::endl;

        uint32_t counter = 0;
        uint32_t successCount = 0;
        uint32_t failCount = 0;

        while (running) {
            // Prepare test data (matches ESP32 test_can_sender format)
            uint8_t txData[8];
            txData[0] = 0xAA;
            txData[1] = 0x55;
            txData[2] = (counter >> 24) & 0xFF;
            txData[3] = (counter >> 16) & 0xFF;
            txData[4] = (counter >> 8) & 0xFF;
            txData[5] = counter & 0xFF;
            txData[6] = 0x00;
            txData[7] = 0x00;

            bool ok = can.sendMessage(sendId, txData, 8);

            if (ok) {
                successCount++;
                std::cout << "[TX] #" << counter
                          << " ID=0x" << std::hex << sendId << std::dec
                          << " Data: ";
                for (int i = 0; i < 8; i++) {
                    std::cout << std::hex << std::setw(2) << std::setfill('0')
                              << (int)txData[i] << std::setfill(' ') << std::dec;
                    if (i < 7) std::cout << " ";
                }
                std::cout << " OK" << std::endl;
            } else {
                failCount++;
                std::cout << "[TX] #" << counter << " FAILED!  ";
                can.printStatus();
            }

            // Statistics every 10 messages
            if (counter > 0 && counter % 10 == 0) {
                float rate = (float)successCount / (float)(successCount + failCount) * 100.0f;
                std::cout << "\n--- Statistics ---" << std::endl;
                std::cout << "Total: " << counter
                          << " | Success: " << successCount
                          << " | Failed: " << failCount
                          << " | Rate: " << std::fixed << std::setprecision(1) << rate << "%"
                          << std::endl;
                can.printStatus();
                std::cout << "------------------\n" << std::endl;
            }

            counter++;
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }
    }

    // ---- Loopback Mode (Self-Test) ----
    else if (mode == "loopback") {
        std::cout << "Running loopback self-test...\n" << std::endl;

        uint32_t passed = 0;
        uint32_t failed = 0;

        for (int test = 0; test < 10 && running; test++) {
            uint16_t testId = 0x7FF & (0x100 + test);
            uint8_t txData[8] = {0xAA, 0x55, (uint8_t)(test & 0xFF),
                                 0x01, 0x02, 0x03, 0x04, 0x05};

            std::cout << "Test " << (test + 1) << "/10: ";
            std::cout << "TX ID=0x" << std::hex << testId << std::dec << " -> ";

            if (!can.sendMessage(testId, txData, 8)) {
                std::cout << "SEND FAILED!" << std::endl;
                failed++;
                continue;
            }

            // Wait for loopback
            std::this_thread::sleep_for(std::chrono::milliseconds(10));

            uint16_t rxId;
            uint8_t rxData[8];
            uint8_t rxLen;

            if (can.receiveMessage(rxId, rxData, rxLen)) {
                if (rxId == testId && rxLen == 8 && memcmp(txData, rxData, 8) == 0) {
                    std::cout << "RX OK (ID=0x" << std::hex << rxId << std::dec
                              << " LEN=" << (int)rxLen << ") PASS" << std::endl;
                    passed++;
                } else {
                    std::cout << "DATA MISMATCH! RX ID=0x" << std::hex << rxId
                              << std::dec << " LEN=" << (int)rxLen << " FAIL" << std::endl;
                    failed++;
                }
            } else {
                std::cout << "NO RESPONSE! FAIL" << std::endl;
                failed++;
            }
        }

        std::cout << "\n=== Loopback Test Results ===" << std::endl;
        std::cout << "Passed: " << passed << "/10" << std::endl;
        std::cout << "Failed: " << failed << "/10" << std::endl;
        can.printStatus();

        if (failed == 0) {
            std::cout << "\nMCP2515 is working correctly!" << std::endl;
        } else {
            std::cout << "\nSome tests failed. Check SPI wiring." << std::endl;
        }
    }

    std::cout << "\nDone." << std::endl;
    return 0;
}
