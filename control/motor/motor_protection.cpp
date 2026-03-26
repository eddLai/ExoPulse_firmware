/*
 * Motor Protection Module - Implementation
 * ExoPulse Deploy_Orin - Firmware Layer
 *
 * Wraps lktech_motor with angle (-45°~+45°) and torque (150 Nm) protection.
 */

#include "motor_protection.h"

#include <cstdlib>
#include <cstring>
#include <cstdio>
#include <cmath>

/* ---- Internal state ---- */

struct motor_protected {
    lktech_motor_t*        motor;
    mcp2515_t*             can;          /* Owned CAN handle */
    motor_protect_config_t config;
    motor_event_cb_t       event_cb;
    void*                  user_data;
    int64_t                zero_offset;  /* Zero-angle reference (0.01 deg units) */
    uint16_t               can_id;
    bool                   initialized;
    bool                   emergency_stopped;
};

/* ---- Helpers ---- */

static double angle_raw_to_deg(int64_t raw) {
    return (double)raw * 0.01;
}

static void fire_event(motor_protected_t* mp, int error_code, const char* msg) {
    if (mp->event_cb) {
        mp->event_cb(mp->can_id, error_code, msg, mp->user_data);
    }
    fprintf(stderr, "[motor_prot 0x%03X] %s\n", mp->can_id, msg);
}

static void fill_prot_status(const motor_protected_t* mp,
                              const motor_status_t* raw,
                              motor_prot_status_t* out) {
    out->temperature    = raw->temperature;
    out->torque_current = raw->torque_current;
    out->speed          = raw->speed;
    out->acceleration   = raw->acceleration;
    out->angle          = raw->angle;
    out->encoder        = raw->encoder;
    out->voltage        = raw->voltage;
    out->error_state    = raw->error_state;

    /* Angle relative to zero reference */
    out->angle_deg = angle_raw_to_deg(raw->angle - mp->zero_offset);

    /* Estimate torque: iq -> amps -> Nm */
    double amps = (double)raw->torque_current * mp->config.iq_to_amp_scale;
    out->torque_nm = amps * mp->config.torque_constant;
}

static int16_t nm_to_iq(const motor_protect_config_t* cfg, double torque_nm) {
    if (cfg->torque_constant <= 0.0 || cfg->iq_to_amp_scale <= 0.0)
        return 0;
    double amps = torque_nm / cfg->torque_constant;
    double iq = amps / cfg->iq_to_amp_scale;
    if (iq >  32767.0) iq =  32767.0;
    if (iq < -32768.0) iq = -32768.0;
    return (int16_t)iq;
}

static double iq_to_nm(const motor_protect_config_t* cfg, int16_t iq) {
    double amps = (double)iq * cfg->iq_to_amp_scale;
    return amps * cfg->torque_constant;
}

/* ---- Lifecycle ---- */

motor_protect_config_t motor_prot_default_config(void) {
    motor_protect_config_t cfg;
    cfg.angle_min_deg       = MOTOR_DEFAULT_ANGLE_MIN_DEG;   /* -45° */
    cfg.angle_max_deg       = MOTOR_DEFAULT_ANGLE_MAX_DEG;   /* +45° */
    cfg.torque_max_nm       = MOTOR_DEFAULT_TORQUE_MAX_NM;   /* 150 Nm */
    cfg.torque_constant     = 2.2;             /* Kt for MG series (Nm/A) */
    cfg.iq_to_amp_scale     = 33.0 / 2048.0;  /* LK-TECH MG: iq 2048 = 33A */
    cfg.angle_mode          = PROTECT_CLAMP;
    cfg.torque_mode         = PROTECT_CLAMP;
    cfg.enable_angle_protect  = true;
    cfg.enable_torque_protect = true;
    return cfg;
}

motor_protected_t* motor_prot_create(const char* spi_device,
                                      uint16_t can_id,
                                      const motor_protect_config_t* config) {
    if (!spi_device) return NULL;

    motor_protected_t* mp = (motor_protected_t*)calloc(1, sizeof(motor_protected_t));
    if (!mp) return NULL;

    mp->config = config ? *config : motor_prot_default_config();
    mp->can_id = can_id;
    mp->event_cb = NULL;
    mp->user_data = NULL;
    mp->zero_offset = 0;
    mp->initialized = false;
    mp->emergency_stopped = false;

    /* Open CAN bus via MCP2515 */
    mp->can = mcp2515_open(spi_device);
    if (!mp->can) {
        fprintf(stderr, "[motor_prot] Failed to open CAN on %s\n", spi_device);
        free(mp);
        return NULL;
    }

    /* Create low-level motor handle */
    mp->motor = motor_create(mp->can, can_id);
    if (!mp->motor) {
        fprintf(stderr, "[motor_prot] Failed to create motor 0x%03X\n", can_id);
        mcp2515_close(mp->can);
        free(mp);
        return NULL;
    }

    return mp;
}

int motor_prot_init(motor_protected_t* mp) {
    if (!mp) return MOTOR_ERR_INVALID_PARAM;

    int ret = motor_init(mp->motor);
    if (ret < 0) {
        fire_event(mp, MOTOR_ERR_HARDWARE, "Init failed: no CAN response");
        return MOTOR_ERR_HARDWARE;
    }

    /* Read initial angle as zero reference */
    motor_status_t raw;
    if (motor_read_status(mp->motor, &raw) == 0) {
        mp->zero_offset = raw.angle;
    }

    mp->initialized = true;
    mp->emergency_stopped = false;

    fprintf(stderr, "[motor_prot 0x%03X] Init OK, zero=%.2f deg, "
            "angle_limits=[%.1f, %.1f] deg, torque_max=%.1f Nm\n",
            mp->can_id, angle_raw_to_deg(mp->zero_offset),
            mp->config.angle_min_deg, mp->config.angle_max_deg,
            mp->config.torque_max_nm);

    return MOTOR_OK;
}

void motor_prot_destroy(motor_protected_t* mp) {
    if (!mp) return;
    if (mp->motor) {
        motor_stop(mp->motor);
        motor_destroy(mp->motor);
    }
    if (mp->can) {
        mcp2515_close(mp->can);
    }
    free(mp);
}

/* ---- Configuration ---- */

int motor_prot_set_config(motor_protected_t* mp,
                           const motor_protect_config_t* config) {
    if (!mp || !config) return MOTOR_ERR_INVALID_PARAM;
    if (config->angle_min_deg >= config->angle_max_deg) return MOTOR_ERR_INVALID_PARAM;
    if (config->torque_max_nm <= 0.0) return MOTOR_ERR_INVALID_PARAM;
    mp->config = *config;
    return MOTOR_OK;
}

int motor_prot_get_config(const motor_protected_t* mp,
                           motor_protect_config_t* config_out) {
    if (!mp || !config_out) return MOTOR_ERR_INVALID_PARAM;
    *config_out = mp->config;
    return MOTOR_OK;
}

int motor_prot_set_callback(motor_protected_t* mp,
                             motor_event_cb_t cb, void* user_data) {
    if (!mp) return MOTOR_ERR_INVALID_PARAM;
    mp->event_cb = cb;
    mp->user_data = user_data;
    return MOTOR_OK;
}

int motor_prot_set_zero(motor_protected_t* mp) {
    if (!mp || !mp->initialized) return MOTOR_ERR_NOT_INITIALIZED;

    motor_status_t raw;
    if (motor_read_status(mp->motor, &raw) < 0)
        return MOTOR_ERR_HARDWARE;

    mp->zero_offset = raw.angle;
    fprintf(stderr, "[motor_prot 0x%03X] Zero set to %.2f deg\n",
            mp->can_id, angle_raw_to_deg(mp->zero_offset));
    return MOTOR_OK;
}

/* ---- Control (with protection) ---- */

int motor_prot_set_torque(motor_protected_t* mp, double torque_nm,
                           motor_prot_status_t* status_out) {
    if (!mp) return MOTOR_ERR_INVALID_PARAM;
    if (!mp->initialized) return MOTOR_ERR_NOT_INITIALIZED;
    if (mp->emergency_stopped) return MOTOR_ERR_EMERGENCY_STOP;

    /* --- Torque protection --- */
    if (mp->config.enable_torque_protect) {
        double abs_torque = fabs(torque_nm);
        if (abs_torque > mp->config.torque_max_nm) {
            switch (mp->config.torque_mode) {
            case PROTECT_CLAMP:
                torque_nm = (torque_nm > 0)
                    ?  mp->config.torque_max_nm
                    : -mp->config.torque_max_nm;
                fire_event(mp, MOTOR_ERR_TORQUE_LIMIT,
                           "Torque clamped to 150 Nm limit");
                break;
            case PROTECT_REJECT:
                fire_event(mp, MOTOR_ERR_TORQUE_LIMIT,
                           "Torque command rejected: exceeds 150 Nm");
                return MOTOR_ERR_TORQUE_LIMIT;
            case PROTECT_ESTOP:
                fire_event(mp, MOTOR_ERR_TORQUE_LIMIT,
                           "Torque limit -> emergency stop");
                motor_prot_emergency_stop(mp);
                return MOTOR_ERR_EMERGENCY_STOP;
            }
        }
    }

    /* --- Angle protection (pre-check current position) --- */
    if (mp->config.enable_angle_protect) {
        motor_status_t raw;
        if (motor_read_status(mp->motor, &raw) == 0) {
            double current_deg = angle_raw_to_deg(raw.angle - mp->zero_offset);
            bool at_min = (current_deg <= mp->config.angle_min_deg);
            bool at_max = (current_deg >= mp->config.angle_max_deg);

            /* Block torque that would push further out of range */
            if ((at_max && torque_nm > 0) || (at_min && torque_nm < 0)) {
                switch (mp->config.angle_mode) {
                case PROTECT_CLAMP:
                    torque_nm = 0.0;
                    fire_event(mp, MOTOR_ERR_ANGLE_LIMIT,
                               "Angle at limit [-45,+45], torque zeroed");
                    break;
                case PROTECT_REJECT:
                    fire_event(mp, MOTOR_ERR_ANGLE_LIMIT,
                               "Angle at limit, command rejected");
                    return MOTOR_ERR_ANGLE_LIMIT;
                case PROTECT_ESTOP:
                    fire_event(mp, MOTOR_ERR_ANGLE_LIMIT,
                               "Angle limit -> emergency stop");
                    motor_prot_emergency_stop(mp);
                    return MOTOR_ERR_EMERGENCY_STOP;
                }
            }
        }
    }

    /* Convert Nm to raw iq and send */
    int16_t iq = nm_to_iq(&mp->config, torque_nm);

    motor_status_t raw_out;
    int ret = motor_set_torque(mp->motor, iq, &raw_out);
    if (ret < 0) return MOTOR_ERR_HARDWARE;

    if (status_out)
        fill_prot_status(mp, &raw_out, status_out);

    return MOTOR_OK;
}

int motor_prot_set_torque_raw(motor_protected_t* mp, int16_t iq,
                               motor_prot_status_t* status_out) {
    if (!mp) return MOTOR_ERR_INVALID_PARAM;
    double torque_nm = iq_to_nm(&mp->config, iq);
    return motor_prot_set_torque(mp, torque_nm, status_out);
}

int motor_prot_read_status(motor_protected_t* mp,
                            motor_prot_status_t* status_out) {
    if (!mp || !status_out) return MOTOR_ERR_INVALID_PARAM;
    if (!mp->initialized) return MOTOR_ERR_NOT_INITIALIZED;

    motor_status_t raw;
    int ret = motor_read_status(mp->motor, &raw);
    if (ret < 0) return MOTOR_ERR_HARDWARE;

    fill_prot_status(mp, &raw, status_out);
    return MOTOR_OK;
}

int motor_prot_stop(motor_protected_t* mp) {
    if (!mp) return MOTOR_ERR_INVALID_PARAM;
    return (motor_stop(mp->motor) == 0) ? MOTOR_OK : MOTOR_ERR_HARDWARE;
}

int motor_prot_shutdown(motor_protected_t* mp) {
    if (!mp) return MOTOR_ERR_INVALID_PARAM;
    return (motor_shutdown(mp->motor) == 0) ? MOTOR_OK : MOTOR_ERR_HARDWARE;
}

int motor_prot_emergency_stop(motor_protected_t* mp) {
    if (!mp) return MOTOR_ERR_INVALID_PARAM;
    mp->emergency_stopped = true;
    fire_event(mp, MOTOR_ERR_EMERGENCY_STOP, "EMERGENCY STOP activated");
    motor_stop(mp->motor);
    motor_shutdown(mp->motor);
    return MOTOR_OK;
}

/* ---- Utility ---- */

bool motor_prot_angle_in_range(const motor_protected_t* mp) {
    if (!mp || !mp->initialized) return false;
    motor_status_t raw;
    if (motor_read_status(mp->motor, &raw) < 0) return false;
    double deg = angle_raw_to_deg(raw.angle - mp->zero_offset);
    return (deg >= mp->config.angle_min_deg && deg <= mp->config.angle_max_deg);
}

int16_t motor_prot_nm_to_iq(const motor_protected_t* mp, double torque_nm) {
    if (!mp) return 0;
    return nm_to_iq(&mp->config, torque_nm);
}

double motor_prot_iq_to_nm(const motor_protected_t* mp, int16_t iq) {
    if (!mp) return 0.0;
    return iq_to_nm(&mp->config, iq);
}

const char* motor_prot_strerror(int error_code) {
    switch (error_code) {
    case MOTOR_OK:                  return "OK";
    case MOTOR_ERR_INVALID_PARAM:   return "Invalid parameter";
    case MOTOR_ERR_ANGLE_LIMIT:     return "Angle limit exceeded [-45, +45] deg";
    case MOTOR_ERR_TORQUE_LIMIT:    return "Torque limit exceeded (150 Nm max)";
    case MOTOR_ERR_HARDWARE:        return "Hardware communication error";
    case MOTOR_ERR_NOT_INITIALIZED: return "Motor not initialized";
    case MOTOR_ERR_EMERGENCY_STOP:  return "Emergency stop active";
    default:                        return "Unknown error";
    }
}
