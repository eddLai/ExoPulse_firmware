#pragma once
#include "motor_protocol.h"
#include "motor_operations.h"
#include "config.h"

/*
 * Motor Control Module
 * Functions for controlling motor position, speed, and torque
 *
 * Command 0xA6: Position closed-loop control (single-turn) mode 2
 * - With spin direction control
 * - Angle range: 0 ~ 35999 (0.01°/LSB, represents 0° ~ 359.99°)
 */

// Control response structure (common for all 0xAx commands)
struct ControlResponse {
    int8_t temperature;      // Motor temperature (°C)
    int16_t torqueCurrent;   // Torque current iq (-2048~2048 -> -33A~33A)
    int16_t speed;           // Motor speed (dps)
    uint32_t encoder;        // Encoder value (0~262143)
};

// ============================================================================
// Position Control Command 0xA6 (Single-turn with direction)
// ============================================================================
// TX: [0xA6] [spinDir] [maxSpeed_L] [maxSpeed_H] [angle_L] [angle_H] [0x00] [0x00]
// RX: [0xA6] [temp] [iq_L] [iq_H] [speed_L] [speed_H] [encoder_L] [encoder_H]
//
// spinDirection: 0x00 = CW, 0x01 = CCW
// maxSpeed: uint16_t, max speed limit (1 dps/LSB), 0 = no limit
// angleControl: uint16_t, 0~35999 (0.01°/LSB, represents 0°~359.99°)

bool sendPositionControl_A6(uint32_t canID, uint8_t spinDirection, uint16_t maxSpeed, uint16_t angleControl, ControlResponse* response = nullptr) {
    // Build command data per protocol
    uint8_t txData[8] = {
        POS_CTRL_SINGLE_2,                      // Byte 0: Command 0xA6
        spinDirection,                           // Byte 1: Spin direction
        (uint8_t)(maxSpeed & 0xFF),             // Byte 2: maxSpeed low byte
        (uint8_t)((maxSpeed >> 8) & 0xFF),      // Byte 3: maxSpeed high byte
        (uint8_t)(angleControl & 0xFF),         // Byte 4: angle low byte
        (uint8_t)((angleControl >> 8) & 0xFF),  // Byte 5: angle high byte
        0x00,                                    // Byte 6: NULL
        0x00                                     // Byte 7: NULL
    };

    Serial.print("[CTRL] TX: ");
    for (int i = 0; i < 8; i++) {
        Serial.print("0x");
        if (txData[i] < 16) Serial.print("0");
        Serial.print(txData[i], HEX);
        Serial.print(" ");
    }
    Serial.println();

    // Suspend canReadTask to prevent it from consuming our response
    vTaskSuspend(canReadTaskHandle);

    // Send command and wait for response atomically
    bool success = false;
    if (xSemaphoreTake(canMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        byte rc = CAN.sendMsgBuf(canID, 0, 8, txData);

        if (rc != CAN_OK) {
            Serial.println("[CTRL] ERROR: CAN send failed");
            xSemaphoreGive(canMutex);
            vTaskResume(canReadTaskHandle);
            return false;
        }

        // Wait for response (while still holding mutex)
        uint8_t rxData[8];
        uint32_t startTime = millis();
        while (millis() - startTime < 100) {
            if (CAN.checkReceive() == CAN_MSGAVAIL) {
                unsigned long rxId;
                byte len;
                CAN.readMsgBuf(&rxId, &len, rxData);

                if (rxId == canID && rxData[0] == POS_CTRL_SINGLE_2) {
                    // Parse response
                    if (response != nullptr) {
                        response->temperature = (int8_t)rxData[1];
                        response->torqueCurrent = (int16_t)(rxData[2] | (rxData[3] << 8));
                        response->speed = (int16_t)(rxData[4] | (rxData[5] << 8));
                        response->encoder = (uint16_t)(rxData[6] | (rxData[7] << 8));
                    }
                    success = true;
                    break;
                }
            }
            vTaskDelay(pdMS_TO_TICKS(1));
        }

        xSemaphoreGive(canMutex);
    } else {
        Serial.println("[CTRL] ERROR: Failed to acquire CAN mutex");
    }

    // Resume canReadTask
    vTaskResume(canReadTaskHandle);

    if (!success) {
        Serial.println("[CTRL] ERROR: No response from motor");
    }
    return success;
}

// Convenience function: Move to angle (degrees) with direction
// angle: 0.0 ~ 359.99 degrees
// maxSpeedDPS: max speed in degrees per second (0 = no limit)
bool moveToAngle(uint32_t canID, float angleDegrees, uint8_t direction, uint16_t maxSpeedDPS = 0, ControlResponse* response = nullptr) {
    // Clamp angle to valid range
    if (angleDegrees < 0) angleDegrees = 0;
    if (angleDegrees > 359.99f) angleDegrees = 359.99f;

    // Convert degrees to 0.01° units (uint16_t, 0~35999)
    uint16_t angleControl = (uint16_t)(angleDegrees * 100.0f);

    Serial.print("[CTRL] Moving to angle: ");
    Serial.print(angleDegrees, 2);
    Serial.print("° (");
    Serial.print(direction == SPIN_CLOCKWISE ? "CW" : "CCW");
    Serial.print("), max speed: ");
    if (maxSpeedDPS == 0) {
        Serial.println("unlimited");
    } else {
        Serial.print(maxSpeedDPS);
        Serial.println(" dps");
    }

    return sendPositionControl_A6(canID, direction, maxSpeedDPS, angleControl, response);
}

// Move Motor 1 to specified angle
bool moveMotor1ToAngle(float angleDegrees, uint8_t direction, uint16_t maxSpeedDPS = 0) {
    ControlResponse response;
    bool success = moveToAngle(CAN_ID_1, angleDegrees, direction, maxSpeedDPS, &response);

    if (success) {
        Serial.print("[CTRL] Motor 1 response - Temp: ");
        Serial.print(response.temperature);
        Serial.print("°C, Current: ");
        Serial.print(response.torqueCurrent);
        Serial.print(", Speed: ");
        Serial.print(response.speed);
        Serial.print(" dps, Encoder: ");
        Serial.println(response.encoder);
    }

    return success;
}

// Move Motor 2 to specified angle
bool moveMotor2ToAngle(float angleDegrees, uint8_t direction, uint16_t maxSpeedDPS = 0) {
    ControlResponse response;
    bool success = moveToAngle(CAN_ID_2, angleDegrees, direction, maxSpeedDPS, &response);

    if (success) {
        Serial.print("[CTRL] Motor 2 response - Temp: ");
        Serial.print(response.temperature);
        Serial.print("°C, Current: ");
        Serial.print(response.torqueCurrent);
        Serial.print(", Speed: ");
        Serial.print(response.speed);
        Serial.print(" dps, Encoder: ");
        Serial.println(response.encoder);
    }

    return success;
}

// ============================================================================
// Torque Closed-Loop Control Command 0xA1
// ============================================================================
// TX: [0xA1] [0x00] [0x00] [0x00] [iq_L] [iq_H] [0x00] [0x00]
// RX: [0xA1] [temp] [iq_L] [iq_H] [speed_L] [speed_H] [encoder_L] [encoder_H]
//
// iqControl: int16_t, -2000~2000 (maps to -32A~32A, 0.01A/LSB)
// Positive = clockwise torque, Negative = counter-clockwise torque

// Maximum torque limit (iq units, ±800 = ±12.8A)
#define TORQUE_IQ_LIMIT 800

bool sendTorqueControl_A1(uint32_t canID, int16_t iqControl, ControlResponse* response = nullptr) {
    // Clamp iqControl to safety limit (±800)
    if (iqControl > TORQUE_IQ_LIMIT) iqControl = TORQUE_IQ_LIMIT;
    if (iqControl < -TORQUE_IQ_LIMIT) iqControl = -TORQUE_IQ_LIMIT;

    // Build command data per protocol
    uint8_t txData[8] = {
        TORQUE_CLOSED_LOOP,                     // Byte 0: Command 0xA1
        0x00,                                    // Byte 1: NULL
        0x00,                                    // Byte 2: NULL
        0x00,                                    // Byte 3: NULL
        (uint8_t)(iqControl & 0xFF),            // Byte 4: iq low byte
        (uint8_t)((iqControl >> 8) & 0xFF),     // Byte 5: iq high byte
        0x00,                                    // Byte 6: NULL
        0x00                                     // Byte 7: NULL
    };

    // Suspend canReadTask to prevent it from consuming our response
    vTaskSuspend(canReadTaskHandle);

    // Send command and wait for response atomically
    bool success = false;
    if (xSemaphoreTake(canMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        byte rc = CAN.sendMsgBuf(canID, 0, 8, txData);

        if (rc != CAN_OK) {
            Serial.println("[TORQUE] ERROR: CAN send failed");
            xSemaphoreGive(canMutex);
            vTaskResume(canReadTaskHandle);
            return false;
        }

        // Wait for response (while still holding mutex)
        uint8_t rxData[8];
        uint32_t startTime = millis();
        while (millis() - startTime < 100) {
            if (CAN.checkReceive() == CAN_MSGAVAIL) {
                unsigned long rxId;
                byte len;
                CAN.readMsgBuf(&rxId, &len, rxData);

                if (rxId == canID && rxData[0] == TORQUE_CLOSED_LOOP) {
                    // Parse response
                    if (response != nullptr) {
                        response->temperature = (int8_t)rxData[1];
                        response->torqueCurrent = (int16_t)(rxData[2] | (rxData[3] << 8));
                        response->speed = (int16_t)(rxData[4] | (rxData[5] << 8));
                        response->encoder = (uint16_t)(rxData[6] | (rxData[7] << 8));
                    }
                    success = true;
                    break;
                }
            }
            vTaskDelay(pdMS_TO_TICKS(1));
        }

        xSemaphoreGive(canMutex);
    } else {
        Serial.println("[TORQUE] ERROR: Failed to acquire CAN mutex");
    }

    // Resume canReadTask
    vTaskResume(canReadTaskHandle);

    if (!success) {
        Serial.println("[TORQUE] ERROR: No response from motor");
    }
    return success;
}

// Convenience function: Set torque by current (Amps)
// currentAmps: -32.0 ~ 32.0 A
bool setTorqueCurrent(uint32_t canID, float currentAmps, ControlResponse* response = nullptr) {
    // Clamp current to valid range
    if (currentAmps > 32.0f) currentAmps = 32.0f;
    if (currentAmps < -32.0f) currentAmps = -32.0f;

    // Convert Amps to iq units (0.01A/LSB, so multiply by 100)
    // But protocol uses -2000~2000 for -32A~32A, so 62.5 units per Amp
    int16_t iqControl = (int16_t)(currentAmps * 62.5f);

    return sendTorqueControl_A1(canID, iqControl, response);
}

// Set Motor 1 torque
// iqValue: -2000~2000 (raw protocol value)
bool setMotor1Torque(int16_t iqValue) {
    ControlResponse response;
    bool success = sendTorqueControl_A1(CAN_ID_1, iqValue, &response);

    if (success) {
        Serial.print("[TORQUE] M1 response - Temp: ");
        Serial.print(response.temperature);
        Serial.print("°C, iq: ");
        Serial.print(response.torqueCurrent);
        Serial.print(", Speed: ");
        Serial.print(response.speed);
        Serial.print(" dps, Encoder: ");
        Serial.println(response.encoder);
    }

    return success;
}

// Set Motor 2 torque
// iqValue: -2000~2000 (raw protocol value)
bool setMotor2Torque(int16_t iqValue) {
    ControlResponse response;
    bool success = sendTorqueControl_A1(CAN_ID_2, iqValue, &response);

    if (success) {
        Serial.print("[TORQUE] M2 response - Temp: ");
        Serial.print(response.temperature);
        Serial.print("°C, iq: ");
        Serial.print(response.torqueCurrent);
        Serial.print(", Speed: ");
        Serial.print(response.speed);
        Serial.print(" dps, Encoder: ");
        Serial.println(response.encoder);
    }

    return success;
}
