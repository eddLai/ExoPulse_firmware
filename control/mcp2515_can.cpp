/*
 * MCP2515 CAN Bus Implementation
 * Thin C++ wrapper over the existing mcp2515 C driver.
 */

#include "mcp2515_can.h"

extern "C" {
#include "motor/mcp2515.h"
}

Mcp2515Can::Mcp2515Can(const char* spi_device, uint32_t spi_speed)
    : dev_(nullptr)
{
    dev_ = mcp2515_create(spi_device, spi_speed);
}

Mcp2515Can::~Mcp2515Can() {
    if (dev_) {
        mcp2515_destroy(dev_);
        dev_ = nullptr;
    }
}

int Mcp2515Can::init(uint32_t baud_rate) {
    if (!dev_) return -1;
    return mcp2515_init(dev_, baud_rate);
}

int Mcp2515Can::send(uint16_t can_id, const uint8_t* data, uint8_t len) {
    if (!dev_) return -1;
    return mcp2515_send(dev_, can_id, data, len);
}

int Mcp2515Can::recv(uint16_t* can_id, uint8_t* data, uint8_t* len,
                     uint32_t timeout_ms) {
    if (!dev_) return -1;
    return mcp2515_recv(dev_, can_id, data, len, timeout_ms);
}
