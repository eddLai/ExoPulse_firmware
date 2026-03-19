/*
 * MCP2515 CAN Controller Module Interface
 * SPI-based CAN bus communication for Jetson Orin
 *
 * Hardware: MCP2515 + TJA1050 CAN Transceiver
 * Platform: Jetson Orin SPI (/dev/spidev0.0)
 *
 * Refactored from module_test/canbus_moudle/mcp2515_canbus.cpp
 */

#ifndef MCP2515_H
#define MCP2515_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Opaque handle */
typedef struct mcp2515 mcp2515_t;

/*
 * Create MCP2515 instance.
 * spi_device: e.g. "/dev/spidev0.0"
 * spi_speed: SPI clock in Hz (default 1000000)
 * Returns NULL on failure.
 */
mcp2515_t* mcp2515_create(const char* spi_device, uint32_t spi_speed);

/*
 * Initialize: open SPI, reset MCP2515, configure baud rate, set Normal mode.
 * baud_rate: CAN baud rate (1000000, 500000, 250000, 125000, 100000)
 * Returns 0 on success, -1 on failure.
 */
int mcp2515_init(mcp2515_t* dev, uint32_t baud_rate);

/*
 * Send a standard CAN frame (11-bit ID).
 * Returns 0 on success, -1 on failure/timeout.
 */
int mcp2515_send(mcp2515_t* dev, uint16_t can_id, const uint8_t* data, uint8_t len);

/*
 * Receive a CAN frame (non-blocking poll).
 * On success: fills can_id, data, len and returns 0.
 * If no message available: returns -1.
 * timeout_ms: max wait time in ms (0 = single poll).
 */
int mcp2515_recv(mcp2515_t* dev, uint16_t* can_id, uint8_t* data, uint8_t* len,
                 uint32_t timeout_ms);

/*
 * Destroy instance, close SPI fd.
 */
void mcp2515_destroy(mcp2515_t* dev);

#ifdef __cplusplus
}
#endif

#endif /* MCP2515_H */
