/*
 * Motor Protection Module - Implementation
 * ExoPulse Firmware Layer
 *
 * Brand-independent: operates through Motor& base reference.
 */

#include "motor_protection.h"

#include <cstdlib>
#include <cstring>
#include <cstdio>
#include <cmath>

/* ---- Helpers ---- */

static double angle_raw_to_deg(int64_t raw) {
    return (double)raw * 0.01;
}

/* ---- Lifecycle ---- */

MotorProtection::MotorProtection(Motor& motor, const ProtectConfig& config)
    : motor_(motor)
    , config_(config)
{
}

int MotorProtection::init() {
    int ret = motor_.init();
    if (ret < 0) {
        fireEvent(MOTOR_ERR_HARDWARE, "Init failed: no CAN response");
        return MOTOR_ERR_HARDWARE;
    }

    /* Read initial angle as zero reference */
    MotorStatus raw;
    if (motor_.readStatus(&raw) == 0) {
        zero_offset_ = raw.angle;
    }

    initialized_ = true;
    emergency_stopped_ = false;

    fprintf(stderr, "[MotorProt %s] Init OK, zero=%.2f deg, "
            "angle_limits=[%.1f, %.1f] deg, torque_max=%.1f Nm\n",
            motor_.name(), angle_raw_to_deg(zero_offset_),
            config_.angle_min_deg, config_.angle_max_deg,
            config_.torque_max_nm);

    return MOTOR_OK;
}

/* ---- Configuration ---- */

int MotorProtection::setConfig(const ProtectConfig& config) {
    if (config.angle_min_deg >= config.angle_max_deg) return MOTOR_ERR_INVALID_PARAM;
    if (config.torque_max_nm <= 0.0) return MOTOR_ERR_INVALID_PARAM;
    config_ = config;
    return MOTOR_OK;
}

void MotorProtection::getConfig(ProtectConfig* out) const {
    if (out) *out = config_;
}

void MotorProtection::setCallback(ProtectEventCb cb, void* user_data) {
    event_cb_ = cb;
    user_data_ = user_data;
}

int MotorProtection::setZero() {
    if (!initialized_) return MOTOR_ERR_NOT_INITIALIZED;

    MotorStatus raw;
    if (motor_.readStatus(&raw) < 0)
        return MOTOR_ERR_HARDWARE;

    zero_offset_ = raw.angle;
    fprintf(stderr, "[MotorProt %s] Zero set to %.2f deg\n",
            motor_.name(), angle_raw_to_deg(zero_offset_));
    return MOTOR_OK;
}

/* ---- Control with protection ---- */

int MotorProtection::setTorque(double torque_nm, ProtectedStatus* status_out) {
    if (!initialized_) return MOTOR_ERR_NOT_INITIALIZED;
    if (emergency_stopped_) return MOTOR_ERR_EMERGENCY_STOP;

    /* --- Torque protection --- */
    if (config_.enable_torque_protect) {
        double abs_torque = fabs(torque_nm);
        if (abs_torque > config_.torque_max_nm) {
            switch (config_.torque_mode) {
            case PROTECT_CLAMP:
                torque_nm = (torque_nm > 0)
                    ?  config_.torque_max_nm
                    : -config_.torque_max_nm;
                fireEvent(MOTOR_ERR_TORQUE_LIMIT, "Torque clamped to limit");
                break;
            case PROTECT_REJECT:
                fireEvent(MOTOR_ERR_TORQUE_LIMIT, "Torque command rejected");
                return MOTOR_ERR_TORQUE_LIMIT;
            case PROTECT_ESTOP:
                fireEvent(MOTOR_ERR_TORQUE_LIMIT, "Torque limit -> emergency stop");
                emergencyStop();
                return MOTOR_ERR_EMERGENCY_STOP;
            }
        }
    }

    /* --- Angle protection (pre-check current position) --- */
    if (config_.enable_angle_protect) {
        MotorStatus raw;
        if (motor_.readStatus(&raw) == 0) {
            double current_deg = angle_raw_to_deg(raw.angle - zero_offset_);
            bool at_min = (current_deg <= config_.angle_min_deg);
            bool at_max = (current_deg >= config_.angle_max_deg);

            /* Block torque that would push further out of range */
            if ((at_max && torque_nm > 0) || (at_min && torque_nm < 0)) {
                switch (config_.angle_mode) {
                case PROTECT_CLAMP:
                    torque_nm = 0.0;
                    fireEvent(MOTOR_ERR_ANGLE_LIMIT, "Angle at limit, torque zeroed");
                    break;
                case PROTECT_REJECT:
                    fireEvent(MOTOR_ERR_ANGLE_LIMIT, "Angle at limit, command rejected");
                    return MOTOR_ERR_ANGLE_LIMIT;
                case PROTECT_ESTOP:
                    fireEvent(MOTOR_ERR_ANGLE_LIMIT, "Angle limit -> emergency stop");
                    emergencyStop();
                    return MOTOR_ERR_EMERGENCY_STOP;
                }
            }

            /* Check hardware error flags (brand-specific detection via virtual) */
            if (motor_.hasError(raw)) {
                fireEvent(MOTOR_ERR_HARDWARE, "Hardware error detected, shutting down");
                motor_.shutdown();
                return MOTOR_ERR_HARDWARE;
            }
        }
    }

    /* Convert Nm to raw iq and send */
    int16_t iq = nmToIq(torque_nm);

    MotorStatus raw_out;
    int ret = motor_.setTorque(iq, &raw_out);
    if (ret < 0) return MOTOR_ERR_HARDWARE;

    if (status_out)
        fillProtectedStatus(raw_out, status_out);

    return MOTOR_OK;
}

int MotorProtection::setTorqueRaw(int16_t iq, ProtectedStatus* status_out) {
    double torque_nm = iqToNm(iq);
    return setTorque(torque_nm, status_out);
}

int MotorProtection::readStatus(ProtectedStatus* status_out) {
    if (!status_out) return MOTOR_ERR_INVALID_PARAM;
    if (!initialized_) return MOTOR_ERR_NOT_INITIALIZED;

    MotorStatus raw;
    int ret = motor_.readStatus(&raw);
    if (ret < 0) return MOTOR_ERR_HARDWARE;

    fillProtectedStatus(raw, status_out);
    return MOTOR_OK;
}

int MotorProtection::stop() {
    return (motor_.stop() == 0) ? MOTOR_OK : MOTOR_ERR_HARDWARE;
}

int MotorProtection::shutdown() {
    return (motor_.shutdown() == 0) ? MOTOR_OK : MOTOR_ERR_HARDWARE;
}

int MotorProtection::emergencyStop() {
    emergency_stopped_ = true;
    fireEvent(MOTOR_ERR_EMERGENCY_STOP, "EMERGENCY STOP activated");
    motor_.stop();
    motor_.shutdown();
    return MOTOR_OK;
}

/* ---- Utility ---- */

bool MotorProtection::angleInRange() const {
    if (!initialized_) return false;
    MotorStatus raw;
    if (motor_.readStatus(&raw) < 0) return false;
    double deg = angle_raw_to_deg(raw.angle - zero_offset_);
    return (deg >= config_.angle_min_deg && deg <= config_.angle_max_deg);
}

int16_t MotorProtection::nmToIq(double torque_nm) const {
    if (config_.torque_constant <= 0.0 || config_.iq_to_amp_scale <= 0.0)
        return 0;
    double amps = torque_nm / config_.torque_constant;
    double iq = amps / config_.iq_to_amp_scale;
    if (iq >  32767.0) iq =  32767.0;
    if (iq < -32768.0) iq = -32768.0;
    return (int16_t)iq;
}

double MotorProtection::iqToNm(int16_t iq) const {
    double amps = (double)iq * config_.iq_to_amp_scale;
    return amps * config_.torque_constant;
}

void MotorProtection::fireEvent(int error_code, const char* msg) {
    if (event_cb_) {
        event_cb_(motor_.name(), error_code, msg, user_data_);
    }
    fprintf(stderr, "[MotorProt %s] %s\n", motor_.name(), msg);
}

void MotorProtection::fillProtectedStatus(const MotorStatus& raw,
                                          ProtectedStatus* out) const {
    out->raw = raw;
    out->angle_deg = angle_raw_to_deg(raw.angle - zero_offset_);

    double amps = (double)raw.torque_current * config_.iq_to_amp_scale;
    out->torque_nm = amps * config_.torque_constant;
}

const char* MotorProtection::strerror(int error_code) {
    switch (error_code) {
    case MOTOR_OK:                  return "OK";
    case MOTOR_ERR_INVALID_PARAM:   return "Invalid parameter";
    case MOTOR_ERR_ANGLE_LIMIT:     return "Angle limit exceeded";
    case MOTOR_ERR_TORQUE_LIMIT:    return "Torque limit exceeded";
    case MOTOR_ERR_HARDWARE:        return "Hardware communication error";
    case MOTOR_ERR_NOT_INITIALIZED: return "Motor not initialized";
    case MOTOR_ERR_EMERGENCY_STOP:  return "Emergency stop active";
    default:                        return "Unknown error";
    }
}
