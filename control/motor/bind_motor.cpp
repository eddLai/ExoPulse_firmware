/*
 * pybind11 bindings for ExoPulse motor control.
 *
 * Module: exo_motor
 * Exposes: Mcp2515Can, LktechMotor, MotorProtection, MotorStatus, etc.
 *
 * Build: cd control/motor && make
 * Usage: import exo_motor
 */

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "../mcp2515_can.h"
#include "lktech/lktech_motor.h"
#include "motor_protection.h"

namespace py = pybind11;

PYBIND11_MODULE(exo_motor, m) {
    m.doc() = "ExoPulse motor control (pybind11)";

    /* ================================================================
     * MotorStatus struct
     * ================================================================ */
    py::class_<MotorStatus>(m, "MotorStatus")
        .def(py::init<>())
        .def_readwrite("temperature",    &MotorStatus::temperature)
        .def_readwrite("torque_current",  &MotorStatus::torque_current)
        .def_readwrite("speed",           &MotorStatus::speed)
        .def_readwrite("acceleration",    &MotorStatus::acceleration)
        .def_readwrite("angle",           &MotorStatus::angle)
        .def_readwrite("encoder",         &MotorStatus::encoder)
        .def_readwrite("voltage",         &MotorStatus::voltage)
        .def_readwrite("error_state",     &MotorStatus::error_state)
        .def("__repr__", [](const MotorStatus& s) {
            char buf[256];
            snprintf(buf, sizeof(buf),
                "MotorStatus(temp=%d, iq=%d, speed=%d, angle=%ld, "
                "encoder=%u, voltage=%u, error=0x%02X)",
                s.temperature, s.torque_current, s.speed,
                (long)s.angle, s.encoder, s.voltage, s.error_state);
            return std::string(buf);
        });

    /* ================================================================
     * ProtectMode enum & ProtectConfig struct
     * ================================================================ */
    py::enum_<ProtectMode>(m, "ProtectMode")
        .value("CLAMP",  PROTECT_CLAMP)
        .value("REJECT", PROTECT_REJECT)
        .value("ESTOP",  PROTECT_ESTOP);

    py::class_<ProtectConfig>(m, "ProtectConfig")
        .def(py::init<>())
        .def_readwrite("angle_min_deg",        &ProtectConfig::angle_min_deg)
        .def_readwrite("angle_max_deg",        &ProtectConfig::angle_max_deg)
        .def_readwrite("torque_max_nm",        &ProtectConfig::torque_max_nm)
        .def_readwrite("torque_constant",      &ProtectConfig::torque_constant)
        .def_readwrite("iq_to_amp_scale",      &ProtectConfig::iq_to_amp_scale)
        .def_readwrite("angle_mode",           &ProtectConfig::angle_mode)
        .def_readwrite("torque_mode",          &ProtectConfig::torque_mode)
        .def_readwrite("enable_angle_protect", &ProtectConfig::enable_angle_protect)
        .def_readwrite("enable_torque_protect",&ProtectConfig::enable_torque_protect);

    /* ================================================================
     * ProtectedStatus struct
     * ================================================================ */
    py::class_<ProtectedStatus>(m, "ProtectedStatus")
        .def(py::init<>())
        .def_readwrite("raw",        &ProtectedStatus::raw)
        .def_readwrite("angle_deg",  &ProtectedStatus::angle_deg)
        .def_readwrite("torque_nm",  &ProtectedStatus::torque_nm);

    /* ================================================================
     * Error code constants
     * ================================================================ */
    m.attr("MOTOR_OK")                   = MOTOR_OK;
    m.attr("MOTOR_ERR_INVALID_PARAM")    = MOTOR_ERR_INVALID_PARAM;
    m.attr("MOTOR_ERR_ANGLE_LIMIT")      = MOTOR_ERR_ANGLE_LIMIT;
    m.attr("MOTOR_ERR_TORQUE_LIMIT")     = MOTOR_ERR_TORQUE_LIMIT;
    m.attr("MOTOR_ERR_HARDWARE")         = MOTOR_ERR_HARDWARE;
    m.attr("MOTOR_ERR_NOT_INITIALIZED")  = MOTOR_ERR_NOT_INITIALIZED;
    m.attr("MOTOR_ERR_EMERGENCY_STOP")   = MOTOR_ERR_EMERGENCY_STOP;

    /* ================================================================
     * Mcp2515Can — CAN bus via SPI
     * ================================================================ */
    py::class_<Mcp2515Can>(m, "Mcp2515Can")
        .def(py::init([](const std::string& spi_device, uint32_t spi_speed) {
            return new Mcp2515Can(spi_device.c_str(), spi_speed);
        }),
             py::arg("spi_device") = "/dev/spidev0.0",
             py::arg("spi_speed")  = 1000000)
        .def("init", [](Mcp2515Can& self, uint32_t baud_rate) {
            int ret;
            { py::gil_scoped_release release; ret = self.init(baud_rate); }
            return ret;
        }, py::arg("baud_rate") = 1000000);

    /* ================================================================
     * LktechMotor — LK-TECH motor control via CAN
     *
     * keep_alive<1,2>: motor (arg 1 = self) keeps CAN bus (arg 2) alive.
     * GIL released during CAN I/O (blocks up to 100ms per command).
     * ================================================================ */
    py::class_<LktechMotor>(m, "LktechMotor")
        .def(py::init<CanBus&, uint16_t, const char*>(),
             py::arg("can"), py::arg("can_id"), py::arg("name"),
             py::keep_alive<1, 2>())
        .def("init", [](LktechMotor& self) {
            int ret;
            { py::gil_scoped_release release; ret = self.init(); }
            return ret;
        })
        .def("set_torque", [](LktechMotor& self, int16_t iq) {
            MotorStatus s;
            int ret;
            { py::gil_scoped_release release; ret = self.setTorque(iq, &s); }
            return std::make_pair(ret, s);
        }, py::arg("iq"))
        .def("set_position", [](LktechMotor& self, int32_t angle, uint16_t speed) {
            MotorStatus s;
            int ret;
            { py::gil_scoped_release release; ret = self.setPosition(angle, speed, &s); }
            return std::make_pair(ret, s);
        }, py::arg("angle_001deg"), py::arg("max_speed") = 700)
        .def("read_status", [](LktechMotor& self) {
            MotorStatus s;
            int ret;
            { py::gil_scoped_release release; ret = self.readStatus(&s); }
            return std::make_pair(ret, s);
        })
        .def("stop", [](LktechMotor& self) {
            int ret;
            { py::gil_scoped_release release; ret = self.stop(); }
            return ret;
        })
        .def("shutdown", [](LktechMotor& self) {
            int ret;
            { py::gil_scoped_release release; ret = self.shutdown(); }
            return ret;
        })
        .def("calibrate_zero", [](LktechMotor& self) {
            int ret;
            { py::gil_scoped_release release; ret = self.calibrateZero(); }
            return ret;
        })
        .def("torque_limit", &LktechMotor::torqueLimit)
        /* LK-TECH specific commands */
        .def("set_position_single", [](LktechMotor& self, uint16_t angle,
                                       uint8_t dir, uint16_t speed) {
            MotorStatus s;
            int ret;
            { py::gil_scoped_release release;
              ret = self.setPositionSingle(angle, dir, speed, &s); }
            return std::make_pair(ret, s);
        }, py::arg("angle_001deg"), py::arg("direction"), py::arg("max_speed"))
        .def("set_speed", [](LktechMotor& self, int32_t speed) {
            MotorStatus s;
            int ret;
            { py::gil_scoped_release release; ret = self.setSpeed(speed, &s); }
            return std::make_pair(ret, s);
        }, py::arg("speed_001dps"))
        .def("read_pid", [](LktechMotor& self) {
            uint8_t akp, aki, skp, ski, tkp, tki;
            int ret;
            { py::gil_scoped_release release;
              ret = self.readPID(&akp, &aki, &skp, &ski, &tkp, &tki); }
            return py::make_tuple(ret, akp, aki, skp, ski, tkp, tki);
        })
        .def("read_encoder", [](LktechMotor& self) {
            uint32_t enc;
            int ret;
            { py::gil_scoped_release release; ret = self.readEncoder(&enc); }
            return std::make_pair(ret, enc);
        })
        .def_property_readonly("can_id", &LktechMotor::canId)
        .def_property_readonly("name", [](const LktechMotor& m) {
            return std::string(m.name());
        });

    /* ================================================================
     * MotorProtection — safety wrapper around any Motor
     *
     * keep_alive<1,2>: protection (self) keeps motor (arg 2) alive.
     * ================================================================ */
    py::class_<MotorProtection>(m, "MotorProtection")
        .def(py::init<Motor&, const ProtectConfig&>(),
             py::arg("motor"),
             py::arg("config") = ProtectConfig{},
             py::keep_alive<1, 2>())
        .def("init", [](MotorProtection& self) {
            int ret;
            { py::gil_scoped_release release; ret = self.init(); }
            return ret;
        })
        .def("set_config", &MotorProtection::setConfig)
        .def("get_config", [](const MotorProtection& self) {
            ProtectConfig cfg;
            self.getConfig(&cfg);
            return cfg;
        })
        .def("set_zero", [](MotorProtection& self) {
            int ret;
            { py::gil_scoped_release release; ret = self.setZero(); }
            return ret;
        })
        .def("set_torque", [](MotorProtection& self, double torque_nm) {
            ProtectedStatus ps;
            int ret;
            { py::gil_scoped_release release; ret = self.setTorque(torque_nm, &ps); }
            return std::make_pair(ret, ps);
        }, py::arg("torque_nm"))
        .def("set_torque_raw", [](MotorProtection& self, int16_t iq) {
            ProtectedStatus ps;
            int ret;
            { py::gil_scoped_release release; ret = self.setTorqueRaw(iq, &ps); }
            return std::make_pair(ret, ps);
        }, py::arg("iq"))
        .def("read_status", [](MotorProtection& self) {
            ProtectedStatus ps;
            int ret;
            { py::gil_scoped_release release; ret = self.readStatus(&ps); }
            return std::make_pair(ret, ps);
        })
        .def("stop", [](MotorProtection& self) {
            int ret;
            { py::gil_scoped_release release; ret = self.stop(); }
            return ret;
        })
        .def("shutdown", [](MotorProtection& self) {
            int ret;
            { py::gil_scoped_release release; ret = self.shutdown(); }
            return ret;
        })
        .def("emergency_stop", &MotorProtection::emergencyStop)
        .def("angle_in_range", [](MotorProtection& self) {
            bool result;
            { py::gil_scoped_release release; result = self.angleInRange(); }
            return result;
        })
        .def("nm_to_iq",  &MotorProtection::nmToIq)
        .def("iq_to_nm",  &MotorProtection::iqToNm)
        .def_static("strerror", &MotorProtection::strerror);
}
