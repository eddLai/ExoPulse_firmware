/*
 * MCP2515 CAN Bus Implementation
 * Wraps the existing mcp2515.h C driver with the CanBus C++ interface.
 *
 * Hardware: MCP2515 + TJA1050 CAN Transceiver
 * Platform: Jetson Orin SPI (/dev/spidev0.0)
 */

#ifndef MCP2515_CAN_H
#define MCP2515_CAN_H

#include "can_bus.h"

/* Forward-declare the C handle (defined in motor/mcp2515.h) */
struct mcp2515;
typedef struct mcp2515 mcp2515_t;

class Mcp2515Can : public CanBus {
public:
    /*
     * spi_device: e.g. "/dev/spidev0.0"
     * spi_speed:  SPI clock in Hz (default 1000000)
     */
    Mcp2515Can(const char* spi_device, uint32_t spi_speed = 1000000);
    ~Mcp2515Can() override;

    /* Non-copyable */
    Mcp2515Can(const Mcp2515Can&) = delete;
    Mcp2515Can& operator=(const Mcp2515Can&) = delete;

    int init(uint32_t baud_rate) override;
    int send(uint16_t can_id, const uint8_t* data, uint8_t len) override;
    int recv(uint16_t* can_id, uint8_t* data, uint8_t* len,
             uint32_t timeout_ms) override;

private:
    mcp2515_t* dev_;
};

#endif /* MCP2515_CAN_H */
