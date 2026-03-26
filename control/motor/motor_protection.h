/*
 * Motor Protection Module
 * ExoPulse Deploy_Orin - Firmware Layer
 *
 * Provides angle and torque protection for LK-TECH MG series motors.
 * Wraps the low-level lktech_motor API with configurable safety limits.
 *
 * Default limits:
 *   - Angle:  -45° ~ +45°
 *   - Torque: 150 Nm
 */

#ifndef MOTOR_PROTECTION_H
#define MOTOR_PROTECTION_H

#include "lktech_motor.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ---- Error codes ---- */
#define MOTOR_OK                    0
#define MOTOR_ERR_INVALID_PARAM    -1
#define MOTOR_ERR_ANGLE_LIMIT      -2
#define MOTOR_ERR_TORQUE_LIMIT     -3
#define MOTOR_ERR_HARDWARE         -4
#define MOTOR_ERR_NOT_INITIALIZED  -5
#define MOTOR_ERR_EMERGENCY_STOP   -6

/* ---- Default protection limits ---- */
#define MOTOR_DEFAULT_ANGLE_MIN_DEG   (-45.0)
#define MOTOR_DEFAULT_ANGLE_MAX_DEG   ( 45.0)
#define MOTOR_DEFAULT_TORQUE_MAX_NM   (150.0)

/* ---- Protection mode flags ---- */
typedef enum {
    PROTECT_CLAMP   = 0,   /* Clamp to limit (soft protection) */
    PROTECT_REJECT  = 1,   /* Reject command and return error */
    PROTECT_ESTOP   = 2    /* Emergency stop the motor */
} motor_protect_mode_t;

/* ---- Protection configuration ---- */
typedef struct {
    double angle_min_deg;          /* Minimum allowed angle in degrees */
    double angle_max_deg;          /* Maximum allowed angle in degrees */
    double torque_max_nm;          /* Maximum allowed torque in Nm */
    double torque_constant;        /* Motor Kt: Nm per Amp (model-specific) */
    double iq_to_amp_scale;        /* iq-to-Amps scale factor (33.0/2048.0 for MG series) */
    motor_protect_mode_t angle_mode;   /* Action on angle violation */
    motor_protect_mode_t torque_mode;  /* Action on torque violation */
    bool   enable_angle_protect;   /* Enable/disable angle protection */
    bool   enable_torque_protect;  /* Enable/disable torque protection */
} motor_protect_config_t;

/* ---- Protected motor status ---- */
typedef struct {
    int8_t   temperature;       /* Celsius */
    int16_t  torque_current;    /* iq raw value */
    int16_t  speed;             /* deg/s */
    int32_t  acceleration;      /* deg/s^2 */
    int64_t  angle;             /* 0.01 deg/LSB, multi-turn */
    uint32_t encoder;           /* raw encoder value */
    uint16_t voltage;           /* 0.1V/LSB */
    uint8_t  error_state;       /* hardware error flags */
    double   angle_deg;         /* angle converted to degrees (relative to zero) */
    double   torque_nm;         /* estimated torque in Nm */
} motor_prot_status_t;

/* ---- Protection event callback ---- */
typedef void (*motor_event_cb_t)(int motor_id, int error_code,
                                  const char* message, void* user_data);

/* Opaque protected motor handle */
typedef struct motor_protected motor_protected_t;

/* ============================================================
 *  API: Lifecycle
 * ============================================================ */

/*
 * Create a protected motor instance.
 *   spi_device : SPI device path for MCP2515 (e.g. "/dev/spidev0.0")
 *   can_id     : Motor CAN ID (0x141, 0x142, ...)
 *   config     : Protection config, or NULL for defaults
 * Returns handle, or NULL on failure.
 */
motor_protected_t* motor_prot_create(const char* spi_device,
                                      uint16_t can_id,
                                      const motor_protect_config_t* config);

/*
 * Initialize motor: verify CAN communication and read initial state.
 * Returns MOTOR_OK on success.
 */
int motor_prot_init(motor_protected_t* mp);

/*
 * Destroy instance and free resources. Sends motor stop first.
 */
void motor_prot_destroy(motor_protected_t* mp);

/* ============================================================
 *  API: Configuration
 * ============================================================ */

/* Get default protection config (angle ±45°, torque 150Nm, clamp mode). */
motor_protect_config_t motor_prot_default_config(void);

/* Update protection config at runtime. */
int motor_prot_set_config(motor_protected_t* mp,
                           const motor_protect_config_t* config);

/* Get current protection config. */
int motor_prot_get_config(const motor_protected_t* mp,
                           motor_protect_config_t* config_out);

/* Register event callback for protection violations. */
int motor_prot_set_callback(motor_protected_t* mp,
                             motor_event_cb_t cb, void* user_data);

/* Set the zero-angle reference to the current position. */
int motor_prot_set_zero(motor_protected_t* mp);

/* ============================================================
 *  API: Motor Control (with protection)
 * ============================================================ */

/*
 * Set motor torque (protected).
 *   torque_nm  : desired torque in Nm (clamped/rejected per config)
 *   status_out : optional, filled with motor response
 * Returns MOTOR_OK, or MOTOR_ERR_TORQUE_LIMIT / MOTOR_ERR_ANGLE_LIMIT.
 */
int motor_prot_set_torque(motor_protected_t* mp, double torque_nm,
                           motor_prot_status_t* status_out);

/*
 * Set motor torque by raw iq value (protected).
 *   iq : raw iq value, converted to Nm internally for limit check
 */
int motor_prot_set_torque_raw(motor_protected_t* mp, int16_t iq,
                               motor_prot_status_t* status_out);

/* Read motor status with angle/torque conversion. */
int motor_prot_read_status(motor_protected_t* mp,
                            motor_prot_status_t* status_out);

/* Stop motor (motor stays enabled but stops moving). */
int motor_prot_stop(motor_protected_t* mp);

/* Shutdown motor (clears running state). */
int motor_prot_shutdown(motor_protected_t* mp);

/* Emergency stop: immediately stop motor regardless of state. */
int motor_prot_emergency_stop(motor_protected_t* mp);

/* ============================================================
 *  API: Utility
 * ============================================================ */

/* Check if current angle is within protection limits. */
bool motor_prot_angle_in_range(const motor_protected_t* mp);

/* Convert torque (Nm) to raw iq value based on motor config. */
int16_t motor_prot_nm_to_iq(const motor_protected_t* mp, double torque_nm);

/* Convert raw iq value to torque (Nm). */
double motor_prot_iq_to_nm(const motor_protected_t* mp, int16_t iq);

/* Get human-readable error string for error code. */
const char* motor_prot_strerror(int error_code);

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_PROTECTION_H */
