/*
 * MCP2515 CAN Controller Implementation
 * Refactored from module_test/canbus_moudle/mcp2515_canbus.cpp
 *
 * SPI communication self-contained (open + ioctl(SPI_IOC_MESSAGE))
 * No external bus abstraction — direct SPI register access
 */

#include "mcp2515.h"

#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include <cstdlib>
#include <cstring>
#include <cstdio>
#include <time.h>

/* ---- MCP2515 SPI Instructions ---- */
#define MCP_RESET       0xC0
#define MCP_READ        0x03
#define MCP_WRITE       0x02
#define MCP_BIT_MODIFY  0x05
#define MCP_READ_STATUS 0xA0
#define MCP_RTS_TX0     0x81

/* ---- MCP2515 Registers ---- */
#define CANSTAT     0x0E
#define CANCTRL     0x0F
#define CNF1        0x2A
#define CNF2        0x29
#define CNF3        0x28
#define CANINTE     0x2B
#define CANINTF     0x2C
#define TXB0CTRL    0x30
#define TXB0SIDH    0x31
#define TXB0SIDL    0x32
#define TXB0DLC     0x35
#define TXB0D0      0x36
#define RXB0CTRL    0x60
#define RXB0SIDH    0x61
#define RXB0SIDL    0x62
#define RXB0DLC     0x65
#define RXB0D0      0x66
#define RXB1CTRL    0x70
#define RXM0SIDH    0x20
#define RXM0SIDL    0x21

/* MCP2515 Modes */
#define MODE_NORMAL   0x00
#define MODE_CONFIG   0x80
#define MODE_MASK     0xE0

/* Interrupt flags */
#define RX0IF   0x01
#define RX1IF   0x02
#define TX0IF   0x04

/* Baud rate configs for 8MHz crystal */
struct baud_cfg {
    uint32_t baud;
    uint8_t cnf1, cnf2, cnf3;
};

static const struct baud_cfg BAUD_TABLE[] = {
    {1000000, 0x00, 0x80, 0x00},
    { 500000, 0x00, 0x90, 0x02},
    { 250000, 0x00, 0xB1, 0x05},
    { 125000, 0x01, 0xB1, 0x05},
    { 100000, 0x01, 0xB4, 0x06},
};
#define BAUD_TABLE_SIZE (sizeof(BAUD_TABLE) / sizeof(BAUD_TABLE[0]))

struct mcp2515 {
    int fd;
    uint32_t speed;
};

/* ---- SPI helpers ---- */

static int spi_transfer(mcp2515_t* dev, uint8_t* tx, uint8_t* rx, size_t len) {
    struct spi_ioc_transfer tr;
    memset(&tr, 0, sizeof(tr));
    tr.tx_buf = (unsigned long)tx;
    tr.rx_buf = (unsigned long)rx;
    tr.len = len;
    tr.speed_hz = dev->speed;
    tr.bits_per_word = 8;

    if (ioctl(dev->fd, SPI_IOC_MESSAGE(1), &tr) < 0) {
        return -1;
    }
    return 0;
}

static uint8_t read_reg(mcp2515_t* dev, uint8_t addr) {
    uint8_t tx[3] = {MCP_READ, addr, 0x00};
    uint8_t rx[3] = {0};
    spi_transfer(dev, tx, rx, 3);
    return rx[2];
}

static void write_reg(mcp2515_t* dev, uint8_t addr, uint8_t value) {
    uint8_t tx[3] = {MCP_WRITE, addr, value};
    uint8_t rx[3] = {0};
    spi_transfer(dev, tx, rx, 3);
}

static void bit_modify(mcp2515_t* dev, uint8_t addr, uint8_t mask, uint8_t data) {
    uint8_t tx[4] = {MCP_BIT_MODIFY, addr, mask, data};
    uint8_t rx[4] = {0};
    spi_transfer(dev, tx, rx, 4);
}

static void mcp_reset(mcp2515_t* dev) {
    uint8_t tx[1] = {MCP_RESET};
    uint8_t rx[1] = {0};
    spi_transfer(dev, tx, rx, 1);
    usleep(10000); /* 10ms */
}

static int set_mode(mcp2515_t* dev, uint8_t mode) {
    bit_modify(dev, CANCTRL, MODE_MASK, mode);
    usleep(10000);
    uint8_t actual = read_reg(dev, CANSTAT) & MODE_MASK;
    return (actual == mode) ? 0 : -1;
}

static void sleep_ms(uint32_t ms) {
    struct timespec ts;
    ts.tv_sec = ms / 1000;
    ts.tv_nsec = (ms % 1000) * 1000000L;
    nanosleep(&ts, NULL);
}

/* ---- Public API ---- */

mcp2515_t* mcp2515_create(const char* spi_device, uint32_t spi_speed) {
    int fd = open(spi_device, O_RDWR);
    if (fd < 0) {
        fprintf(stderr, "[mcp2515] Failed to open %s\n", spi_device);
        return NULL;
    }

    uint8_t mode = SPI_MODE_0;
    uint8_t bits = 8;
    ioctl(fd, SPI_IOC_WR_MODE, &mode);
    ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &bits);
    ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &spi_speed);

    mcp2515_t* dev = (mcp2515_t*)malloc(sizeof(mcp2515_t));
    if (!dev) {
        close(fd);
        return NULL;
    }
    dev->fd = fd;
    dev->speed = spi_speed;
    return dev;
}

int mcp2515_init(mcp2515_t* dev, uint32_t baud_rate) {
    if (!dev) return -1;

    /* Reset into config mode */
    mcp_reset(dev);

    uint8_t stat = read_reg(dev, CANSTAT) & MODE_MASK;
    if (stat != MODE_CONFIG) {
        fprintf(stderr, "[mcp2515] Not in config mode after reset (0x%02X)\n", stat);
        return -1;
    }

    /* Find baud rate config */
    const struct baud_cfg* cfg = NULL;
    for (size_t i = 0; i < BAUD_TABLE_SIZE; i++) {
        if (BAUD_TABLE[i].baud == baud_rate) {
            cfg = &BAUD_TABLE[i];
            break;
        }
    }
    if (!cfg) {
        fprintf(stderr, "[mcp2515] Unsupported baud rate: %u\n", baud_rate);
        return -1;
    }

    /* Set CNF registers */
    write_reg(dev, CNF1, cfg->cnf1);
    write_reg(dev, CNF2, cfg->cnf2);
    write_reg(dev, CNF3, cfg->cnf3);

    /* Verify */
    if (read_reg(dev, CNF1) != cfg->cnf1 ||
        read_reg(dev, CNF2) != cfg->cnf2 ||
        read_reg(dev, CNF3) != cfg->cnf3) {
        fprintf(stderr, "[mcp2515] Baud rate config verify failed\n");
        return -1;
    }

    /* Accept all messages (mask = 0x000) */
    write_reg(dev, RXM0SIDH, 0x00);
    write_reg(dev, RXM0SIDL, 0x00);
    write_reg(dev, RXB0CTRL, 0x64); /* Receive any, rollover to RXB1 */
    write_reg(dev, RXB1CTRL, 0x60);

    /* Enable RX interrupts, clear flags */
    write_reg(dev, CANINTE, RX0IF | RX1IF);
    write_reg(dev, CANINTF, 0x00);

    /* Set Normal mode */
    if (set_mode(dev, MODE_NORMAL) < 0) {
        fprintf(stderr, "[mcp2515] Failed to set Normal mode\n");
        return -1;
    }

    return 0;
}

int mcp2515_send(mcp2515_t* dev, uint16_t can_id, const uint8_t* data, uint8_t len) {
    if (!dev || !data) return -1;
    if (len > 8) len = 8;

    /* Wait for TX buffer available */
    for (int retry = 0; retry < 10; retry++) {
        if (!(read_reg(dev, TXB0CTRL) & 0x08)) break;
        sleep_ms(1);
        if (retry == 9) {
            fprintf(stderr, "[mcp2515] TX buffer busy\n");
            return -1;
        }
    }

    /* Set standard 11-bit ID */
    write_reg(dev, TXB0SIDH, (can_id >> 3) & 0xFF);
    write_reg(dev, TXB0SIDL, (can_id << 5) & 0xE0);

    /* Set DLC */
    write_reg(dev, TXB0DLC, len);

    /* Write data */
    for (uint8_t i = 0; i < len; i++) {
        write_reg(dev, TXB0D0 + i, data[i]);
    }

    /* Request to send */
    uint8_t tx[1] = {MCP_RTS_TX0};
    uint8_t rx[1] = {0};
    spi_transfer(dev, tx, rx, 1);

    /* Wait for transmit complete */
    for (int i = 0; i < 10; i++) {
        if (!(read_reg(dev, TXB0CTRL) & 0x08)) {
            bit_modify(dev, CANINTF, TX0IF, 0x00);
            return 0;
        }
        sleep_ms(1);
    }

    fprintf(stderr, "[mcp2515] TX timeout\n");
    return -1;
}

int mcp2515_recv(mcp2515_t* dev, uint16_t* can_id, uint8_t* data, uint8_t* len,
                 uint32_t timeout_ms) {
    if (!dev || !can_id || !data || !len) return -1;

    uint32_t elapsed = 0;
    do {
        uint8_t intf = read_reg(dev, CANINTF);

        /* Check RX buffer 0 */
        if (intf & RX0IF) {
            uint8_t sidh = read_reg(dev, RXB0SIDH);
            uint8_t sidl = read_reg(dev, RXB0SIDL);
            *can_id = (sidh << 3) | (sidl >> 5);

            *len = read_reg(dev, RXB0DLC) & 0x0F;
            if (*len > 8) *len = 8;

            for (uint8_t i = 0; i < *len; i++) {
                data[i] = read_reg(dev, RXB0D0 + i);
            }

            bit_modify(dev, CANINTF, RX0IF, 0x00);
            return 0;
        }

        /* Check RX buffer 1 */
        if (intf & RX1IF) {
            uint8_t sidh = read_reg(dev, 0x71); /* RXB1SIDH */
            uint8_t sidl = read_reg(dev, 0x72); /* RXB1SIDL */
            *can_id = (sidh << 3) | (sidl >> 5);

            *len = read_reg(dev, 0x75) & 0x0F;  /* RXB1DLC */
            if (*len > 8) *len = 8;

            for (uint8_t i = 0; i < *len; i++) {
                data[i] = read_reg(dev, 0x76 + i); /* RXB1D0+ */
            }

            bit_modify(dev, CANINTF, RX1IF, 0x00);
            return 0;
        }

        if (timeout_ms == 0) break;
        sleep_ms(1);
        elapsed++;
    } while (elapsed < timeout_ms);

    return -1; /* No message */
}

void mcp2515_destroy(mcp2515_t* dev) {
    if (!dev) return;
    if (dev->fd >= 0) close(dev->fd);
    free(dev);
}
