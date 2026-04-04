/*
 * CAN Bus Abstract Interface
 * ExoPulse Firmware Layer
 *
 * Minimal abstraction: only raw frame send/recv.
 * All protocol logic (frame format, response matching, timeouts)
 * belongs in the motor subclass, NOT here.
 *
 * Implementations: Mcp2515Can (SPI), SocketCan (future)
 */

#ifndef CAN_BUS_H
#define CAN_BUS_H

#include <cstdint>

class CanBus {
public:
    virtual ~CanBus() = default;

    /*
     * Initialize the CAN controller.
     * baud_rate: CAN bus speed in Hz (1000000, 500000, 250000, 125000, 100000)
     * Returns 0 on success, -1 on failure.
     */
    virtual int init(uint32_t baud_rate) = 0;

    /*
     * Send a standard CAN frame (11-bit ID).
     * Returns 0 on success, -1 on failure/timeout.
     */
    virtual int send(uint16_t can_id, const uint8_t* data, uint8_t len) = 0;

    /*
     * Receive a CAN frame (non-blocking poll).
     * On success: fills can_id, data, len and returns 0.
     * If no message available: returns -1.
     * timeout_ms: max wait time in ms (0 = single poll).
     */
    virtual int recv(uint16_t* can_id, uint8_t* data, uint8_t* len,
                     uint32_t timeout_ms) = 0;
};

#endif /* CAN_BUS_H */
