# Control Modules API Reference

## Motor Control Module (`motor/`)

### Hardware
- **CAN Controller:** MCP2515 + TJA1050 transceiver
- **Interface:** SPI (Jetson Orin, /dev/spidev0.0)
- **Motor:** LK-TECH MG series (MGv2)
- **CAN Bus:** 1Mbps, standard 11-bit ID
- **Motor IDs:** 0x141 (Motor 1 = right hip), 0x142 (Motor 2 = left hip)
- **Encoder:** 18-bit (0~262143)

### MCP2515 CAN API (`mcp2515.h`)

```c
#include "mcp2515.h"

// Lifecycle
mcp2515_t* mcp2515_create(const char* spi_device, uint32_t spi_speed);
int         mcp2515_init(mcp2515_t* dev, uint32_t baud_rate);  // 0=OK
int         mcp2515_send(mcp2515_t* dev, uint16_t can_id, const uint8_t* data, uint8_t len);
int         mcp2515_recv(mcp2515_t* dev, uint16_t* can_id, uint8_t* data, uint8_t* len,
                         uint32_t timeout_ms);
void        mcp2515_destroy(mcp2515_t* dev);
```

Supported baud rates (8MHz crystal): 1000000, 500000, 250000, 125000, 100000

### Motor API (`lktech_motor.h`)

```c
#include "lktech_motor.h"

// Status data
typedef struct {
    int8_t temperature;        // Celsius
    int16_t torque_current;    // iq: -2048~2048 -> -33A~33A
    int16_t speed;             // deg/s
    int32_t acceleration;      // deg/s^2 (computed)
    int64_t angle;             // 0.01 deg/LSB, multi-turn
    uint32_t encoder;          // 18-bit: 0~262143
    uint16_t voltage;          // 0.1V/LSB
    uint8_t error_state;       // bit0: low voltage, bit3: over-temp
} motor_status_t;

// Lifecycle
lktech_motor_t* motor_create(mcp2515_t* can, uint16_t can_id);
int              motor_init(lktech_motor_t* dev);
int              motor_set_torque(lktech_motor_t* dev, int16_t iq, motor_status_t* status_out);
int              motor_read_status(lktech_motor_t* dev, motor_status_t* out);
int              motor_stop(lktech_motor_t* dev);
int              motor_shutdown(lktech_motor_t* dev);
void             motor_destroy(lktech_motor_t* dev);
```

### Torque Control Details
- **Command:** 0xA1 (Torque closed-loop)
- **Range:** iq -2000~2000 (maps to -32A~32A)
- **Safety limit:** Clamped to +/-800 (~12.8A) in firmware
- **Frame format TX:** `[0xA1] [00] [00] [00] [iq_L] [iq_H] [00] [00]`
- **Frame format RX:** `[0xA1] [temp] [iq_L] [iq_H] [speed_L] [speed_H] [enc_L] [enc_H]`

### Status Read Commands
| Command | Description | Response |
|---------|-------------|----------|
| 0x9A | Status 1 | temp, voltage, error flags |
| 0x9C | Status 2 | temp, torque current, speed, encoder |
| 0x92 | Multi-angle | 7-byte int64_t angle (0.01 deg/LSB) |
| 0x81 | Stop | Motor stops, stays enabled |
| 0x80 | Shutdown | Motor off, clears state |

### Build

```bash
cd control/motor
make          # -> libexo_motor.so + test_motor
./test_motor                    # default: /dev/spidev0.0, motor 0x141
./test_motor /dev/spidev0.0 0x142   # test motor 2
```

### Python (ctypes)

```python
import ctypes

lib = ctypes.CDLL("./control/motor/libexo_motor.so")

class MotorStatus(ctypes.Structure):
    _fields_ = [
        ("temperature", ctypes.c_int8),
        ("torque_current", ctypes.c_int16),
        ("speed", ctypes.c_int16),
        ("acceleration", ctypes.c_int32),
        ("angle", ctypes.c_int64),
        ("encoder", ctypes.c_uint32),
        ("voltage", ctypes.c_uint16),
        ("error_state", ctypes.c_uint8),
    ]

# MCP2515
lib.mcp2515_create.restype = ctypes.c_void_p
lib.mcp2515_create.argtypes = [ctypes.c_char_p, ctypes.c_uint32]
lib.mcp2515_init.restype = ctypes.c_int
lib.mcp2515_init.argtypes = [ctypes.c_void_p, ctypes.c_uint32]
lib.mcp2515_destroy.argtypes = [ctypes.c_void_p]

# Motor
lib.motor_create.restype = ctypes.c_void_p
lib.motor_create.argtypes = [ctypes.c_void_p, ctypes.c_uint16]
lib.motor_init.restype = ctypes.c_int
lib.motor_init.argtypes = [ctypes.c_void_p]
lib.motor_set_torque.restype = ctypes.c_int
lib.motor_set_torque.argtypes = [ctypes.c_void_p, ctypes.c_int16, ctypes.POINTER(MotorStatus)]
lib.motor_read_status.restype = ctypes.c_int
lib.motor_read_status.argtypes = [ctypes.c_void_p, ctypes.POINTER(MotorStatus)]
lib.motor_stop.restype = ctypes.c_int
lib.motor_stop.argtypes = [ctypes.c_void_p]
lib.motor_destroy.argtypes = [ctypes.c_void_p]

can = lib.mcp2515_create(b"/dev/spidev0.0", 1000000)
lib.mcp2515_init(can, 1000000)

motor = lib.motor_create(can, 0x141)
lib.motor_init(motor)

status = MotorStatus()
lib.motor_read_status(motor, ctypes.byref(status))
print(f"temp={status.temperature}, speed={status.speed}, encoder={status.encoder}")

lib.motor_stop(motor)
lib.motor_destroy(motor)
lib.mcp2515_destroy(can)
```

### SPI Pin Connections (Jetson Orin 40-pin)
| MCP2515 | Jetson Pin | Function |
|---------|-----------|----------|
| CS | Pin 24 | SPI0_CS0 |
| SCK | Pin 23 | SPI0_SCK |
| MOSI | Pin 19 | SPI0_MOSI |
| MISO | Pin 21 | SPI0_MISO |
| VCC | Pin 2/4 | 5V |
| GND | Pin 25 | GND |
