/*
 * Motor Abstract Base Class
 * ExoPulse Firmware Layer
 *
 * Defines the common interface for ALL motor types.
 * Each motor brand implements its own communication protocol internally.
 * Upper layers (protection, serial commands, AI agent) only see this interface.
 *
 * To add a new motor:
 *   1. Create motor/<brand>/<brand>_motor.h inheriting Motor
 *   2. Implement all pure virtual methods with brand-specific CAN protocol
 *   3. Add brand-specific commands as additional public methods
 */

#ifndef MOTOR_BASE_H
#define MOTOR_BASE_H

#include "../can_bus.h"
#include <cstdint>

/* Common motor status — all motor brands fill this same struct.
 * Units are normalized so upper layers don't need brand-specific conversion. */
struct MotorStatus {
    int8_t   temperature    = 0;     /* Celsius */
    int16_t  torque_current = 0;     /* iq raw value (brand-specific scale) */
    int16_t  speed          = 0;     /* deg/s */
    int32_t  acceleration   = 0;     /* deg/s^2 (computed from speed delta) */
    int64_t  angle          = 0;     /* 0.01 deg/LSB, multi-turn */
    uint32_t encoder        = 0;     /* raw encoder value (resolution varies) */
    uint16_t voltage        = 0;     /* 0.1V/LSB */
    uint8_t  error_state    = 0;     /* error flags (interpretation varies) */
};

class Motor {
public:
    Motor(CanBus& can, uint16_t can_id, const char* name)
        : can_(can), can_id_(can_id), name_(name) {}

    virtual ~Motor() = default;

    /* Non-copyable */
    Motor(const Motor&) = delete;
    Motor& operator=(const Motor&) = delete;

    /* ================================================================
     * Common interface — every motor brand MUST implement these.
     * The internal CAN protocol is completely different per brand,
     * but the external API is the same.
     * ================================================================ */

    /*
     * Verify CAN communication with the motor.
     * Returns 0 on success, -1 on failure.
     */
    virtual int init() = 0;

    /*
     * Torque control.
     * iq: raw torque value (scale depends on brand, clamped internally)
     * status_out: optional, filled with motor response.
     * Returns 0 on success, -1 on failure.
     */
    virtual int setTorque(int16_t iq, MotorStatus* status_out = nullptr) = 0;

    /*
     * Position control (multi-turn absolute).
     * angle_001deg: target angle in 0.01 deg units (e.g., 36000 = 360 deg)
     * max_speed: speed limit in deg/s (0 = no limit, brand may interpret differently)
     * Returns 0 on success, -1 on failure.
     */
    virtual int setPosition(int32_t angle_001deg, uint16_t max_speed,
                            MotorStatus* status_out = nullptr) = 0;

    /*
     * Read full motor status.
     * How many CAN commands this takes depends on the brand
     * (e.g., LK-TECH needs 3 commands, CubeMars just recv).
     * Returns 0 on success, -1 on failure.
     */
    virtual int readStatus(MotorStatus* out) = 0;

    /* Stop motor (motor stays enabled but stops moving). */
    virtual int stop() = 0;

    /* Shutdown motor (clears running state, motor disabled). */
    virtual int shutdown() = 0;

    /*
     * Set current position as zero reference.
     * Implementation varies: some write to ROM, some use software offset.
     * Returns 0 on success, -1 on failure.
     */
    virtual int calibrateZero() = 0;

    /* ================================================================
     * Safety hooks — subclass provides brand-specific limits/detection.
     * Called by MotorProtection layer.
     * ================================================================ */

    /* Maximum allowed |iq| value for this motor. */
    virtual int16_t torqueLimit() const = 0;

    /* Check if MotorStatus indicates a hardware error. */
    virtual bool hasError(const MotorStatus& status) const = 0;

    /* ================================================================
     * Common getters (non-virtual)
     * ================================================================ */

    uint16_t    canId() const { return can_id_; }
    const char* name()  const { return name_; }

protected:
    CanBus&     can_;
    uint16_t    can_id_;
    const char* name_;
};

#endif /* MOTOR_BASE_H */
