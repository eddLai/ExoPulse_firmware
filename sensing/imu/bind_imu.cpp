/*
 * pybind11 bindings for ExoPulse IMU sensor.
 *
 * Module: exo_imu
 * Exposes: MPU6050 (RAII class), IMUReading (struct), quat_to_euler()
 *
 * Build: cd sensing/imu && make
 * Usage: import exo_imu
 */

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <stdexcept>
#include <array>
#include <string>

extern "C" {
#include "mpu6050.h"
}

namespace py = pybind11;

/* Python-friendly data container mirroring imu_data_t */
struct IMUReading {
    std::array<float, 3> accel;
    std::array<float, 3> gyro;
    std::array<float, 4> quat;
    float  temperature;
    double timestamp;
};

/* RAII wrapper around mpu6050 opaque C handle */
class MPU6050 {
public:
    MPU6050(const std::string& device, uint8_t address)
        : dev_(nullptr)
    {
        dev_ = mpu6050_create(device.c_str(), address);
        if (!dev_)
            throw std::runtime_error("Failed to create MPU6050 on " + device);
    }

    ~MPU6050() {
        if (dev_) {
            mpu6050_destroy(dev_);
            dev_ = nullptr;
        }
    }

    /* Non-copyable */
    MPU6050(const MPU6050&) = delete;
    MPU6050& operator=(const MPU6050&) = delete;

    int init() {
        return mpu6050_init(dev_);
    }

    int init_config(uint16_t rate_hz, uint16_t cal_samples, float beta) {
        mpu6050_config_t cfg;
        cfg.sample_rate_hz      = rate_hz;
        cfg.calibration_samples = cal_samples;
        cfg.madgwick_beta       = beta;
        return mpu6050_init_config(dev_, &cfg);
    }

    std::pair<int, IMUReading> read() {
        imu_data_t raw;
        int ret = mpu6050_read(dev_, &raw);
        IMUReading r;
        if (ret == 0) {
            r.accel = {raw.accel[0], raw.accel[1], raw.accel[2]};
            r.gyro  = {raw.gyro[0],  raw.gyro[1],  raw.gyro[2]};
            r.quat  = {raw.quat[0],  raw.quat[1],  raw.quat[2], raw.quat[3]};
            r.temperature = raw.temperature;
            r.timestamp   = raw.timestamp;
        }
        return {ret, r};
    }

    int reset_quaternion() {
        return mpu6050_reset_quaternion(dev_);
    }

    std::array<float, 4> get_quaternion() {
        float q[4] = {1, 0, 0, 0};
        mpu6050_get_quaternion(dev_, q);
        return {q[0], q[1], q[2], q[3]};
    }

private:
    mpu6050_t* dev_;
};

PYBIND11_MODULE(exo_imu, m) {
    m.doc() = "ExoPulse IMU sensor (pybind11)";

    /* ================================================================
     * IMUReading struct
     * ================================================================ */
    py::class_<IMUReading>(m, "IMUReading")
        .def(py::init<>())
        .def_readwrite("accel",       &IMUReading::accel)
        .def_readwrite("gyro",        &IMUReading::gyro)
        .def_readwrite("quat",        &IMUReading::quat)
        .def_readwrite("temperature", &IMUReading::temperature)
        .def_readwrite("timestamp",   &IMUReading::timestamp)
        .def("__repr__", [](const IMUReading& r) {
            char buf[256];
            snprintf(buf, sizeof(buf),
                "IMUReading(accel=[%.3f,%.3f,%.3f], gyro=[%.1f,%.1f,%.1f], "
                "quat=[%.3f,%.3f,%.3f,%.3f], temp=%.1f)",
                r.accel[0], r.accel[1], r.accel[2],
                r.gyro[0],  r.gyro[1],  r.gyro[2],
                r.quat[0],  r.quat[1],  r.quat[2],  r.quat[3],
                r.temperature);
            return std::string(buf);
        });

    /* ================================================================
     * MPU6050 class (RAII)
     * GIL released during I2C operations.
     * ================================================================ */
    py::class_<MPU6050>(m, "MPU6050")
        .def(py::init<const std::string&, uint8_t>(),
             py::arg("device") = "/dev/i2c-7",
             py::arg("address") = 0x68)
        .def("init", [](MPU6050& self) {
            int ret;
            { py::gil_scoped_release release; ret = self.init(); }
            return ret;
        })
        .def("init_config", [](MPU6050& self, uint16_t rate, uint16_t cal, float beta) {
            int ret;
            { py::gil_scoped_release release; ret = self.init_config(rate, cal, beta); }
            return ret;
        }, py::arg("rate_hz") = 200,
           py::arg("cal_samples") = 500,
           py::arg("beta") = 0.1f)
        .def("read", [](MPU6050& self) {
            std::pair<int, IMUReading> result;
            { py::gil_scoped_release release; result = self.read(); }
            return result;
        })
        .def("reset_quaternion", &MPU6050::reset_quaternion)
        .def("get_quaternion", &MPU6050::get_quaternion);

    /* ================================================================
     * quat_to_euler free function
     * Input: (w, x, y, z) or list/tuple of 4 floats
     * Output: (roll, pitch, yaw) in degrees
     * ================================================================ */
    m.def("quat_to_euler", [](float w, float x, float y, float z) {
        float q[4] = {w, x, y, z};
        float roll, pitch, yaw;
        quat_to_euler(q, &roll, &pitch, &yaw);
        return py::make_tuple(roll, pitch, yaw);
    }, py::arg("w"), py::arg("x"), py::arg("y"), py::arg("z"),
       "Convert quaternion (w,x,y,z) to Euler angles (roll, pitch, yaw) in degrees");

    /* Overload: accept a list/tuple of 4 floats */
    m.def("quat_to_euler", [](const std::array<float, 4>& q) {
        float roll, pitch, yaw;
        float qf[4] = {q[0], q[1], q[2], q[3]};
        quat_to_euler(qf, &roll, &pitch, &yaw);
        return py::make_tuple(roll, pitch, yaw);
    }, py::arg("quat"),
       "Convert quaternion [w,x,y,z] to Euler angles (roll, pitch, yaw) in degrees");
}
