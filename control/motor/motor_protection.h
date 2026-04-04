/*
 * Motor Protection Module
 * ExoPulse Firmware Layer
 *
 * Wraps ANY Motor subclass (via Motor& reference) with configurable safety:
 *   - Angle limits (default: -45 deg ~ +45 deg)
 *   - Torque limits (default: 150 Nm)
 *   - Emergency stop
 *   - Event callbacks on protection violations
 *
 * Brand-independent: works with LktechMotor, RmdMotor, or any future Motor.
 */

#ifndef MOTOR_PROTECTION_H
#define MOTOR_PROTECTION_H

#include "motor_base.h"
#include <cstdint>

/* Error codes */
#define MOTOR_OK                    0
#define MOTOR_ERR_INVALID_PARAM    -1
#define MOTOR_ERR_ANGLE_LIMIT      -2
#define MOTOR_ERR_TORQUE_LIMIT     -3
#define MOTOR_ERR_HARDWARE         -4
#define MOTOR_ERR_NOT_INITIALIZED  -5
#define MOTOR_ERR_EMERGENCY_STOP   -6

/* Protection mode flags */
enum ProtectMode {
    PROTECT_CLAMP   = 0,   /* Clamp to limit (soft protection) */
    PROTECT_REJECT  = 1,   /* Reject command and return error */
    PROTECT_ESTOP   = 2    /* Emergency stop the motor */
};

/* Protection configuration */
struct ProtectConfig {
    double       angle_min_deg       = -45.0;
    double       angle_max_deg       =  45.0;
    double       torque_max_nm       = 150.0;
    double       torque_constant     =   2.2;        /* Kt: Nm per Amp */
    double       iq_to_amp_scale     =  33.0 / 2048.0;
    ProtectMode  angle_mode          = PROTECT_CLAMP;
    ProtectMode  torque_mode         = PROTECT_CLAMP;
    bool         enable_angle_protect  = true;
    bool         enable_torque_protect = true;
};

/* Extended status with physical-unit conversions */
struct ProtectedStatus {
    MotorStatus  raw;               /* original motor status */
    double       angle_deg;         /* angle in degrees (relative to zero) */
    double       torque_nm;         /* estimated torque in Nm */
};

/* Event callback for protection violations */
typedef void (*ProtectEventCb)(const char* motor_name, int error_code,
                               const char* message, void* user_data);

class MotorProtection {
public:
    MotorProtection(Motor& motor, const ProtectConfig& config = ProtectConfig{});
    ~MotorProtection() = default;

    /* Non-copyable */
    MotorProtection(const MotorProtection&) = delete;
    MotorProtection& operator=(const MotorProtection&) = delete;

    /* ================================================================
     * Lifecycle
     * ================================================================ */

    /* Initialize: verify CAN communication, read initial angle as zero ref. */
    int init();

    /* ================================================================
     * Configuration
     * ================================================================ */

    int  setConfig(const ProtectConfig& config);
    void getConfig(ProtectConfig* out) const;
    void setCallback(ProtectEventCb cb, void* user_data);
    int  setZero();

    /* ================================================================
     * Motor control (with protection)
     * ================================================================ */

    /* Set torque in Nm (clamped/rejected per config). */
    int setTorque(double torque_nm, ProtectedStatus* status_out = nullptr);

    /* Set torque by raw iq (converted to Nm internally for limit check). */
    int setTorqueRaw(int16_t iq, ProtectedStatus* status_out = nullptr);

    /* Read status with angle/torque conversion. */
    int readStatus(ProtectedStatus* status_out);

    int stop();
    int shutdown();
    int emergencyStop();

    /* ================================================================
     * Utility
     * ================================================================ */

    bool    angleInRange() const;
    int16_t nmToIq(double torque_nm) const;
    double  iqToNm(int16_t iq) const;

    /* Access the underlying motor (for brand-specific commands via downcast) */
    Motor&       motor()       { return motor_; }
    const Motor& motor() const { return motor_; }

    static const char* strerror(int error_code);

private:
    void fireEvent(int error_code, const char* msg);
    void fillProtectedStatus(const MotorStatus& raw, ProtectedStatus* out) const;

    Motor&         motor_;
    ProtectConfig  config_;
    ProtectEventCb event_cb_     = nullptr;
    void*          user_data_    = nullptr;
    int64_t        zero_offset_  = 0;
    bool           initialized_  = false;
    bool           emergency_stopped_ = false;
};

#endif /* MOTOR_PROTECTION_H */
