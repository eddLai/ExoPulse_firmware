#pragma once
#include "motor_protocol.h"
#include <mcp_can.h>
#include <cstring>  // for memcpy
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

/*
 * Motor CAN Communication Operations
 * Functions for reading motor status via CAN bus
 */

// External references
extern MCP_CAN CAN;
extern SemaphoreHandle_t canMutex;
extern TaskHandle_t canReadTaskHandle;

// Send a read command to specific motor
bool sendReadCommand(uint32_t canID, uint8_t cmd) {
    uint8_t txData[8] = {cmd, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

    if (xSemaphoreTake(canMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        byte rc = CAN.sendMsgBuf(canID, 0, 8, txData);
        xSemaphoreGive(canMutex);
        return (rc == CAN_OK);
    }
    return false;
}

// Read CAN response from motor (non-blocking)
bool readMotorResponse(uint32_t expectedID, uint8_t expectedCmd, uint8_t* rxData, uint32_t timeoutMs = 50) {
    uint32_t startTime = millis();

    while (millis() - startTime < timeoutMs) {
        if (xSemaphoreTake(canMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
            if (CAN.checkReceive() == CAN_MSGAVAIL) {
                unsigned long rxId;
                byte len;

                CAN.readMsgBuf(&rxId, &len, rxData);
                xSemaphoreGive(canMutex);

                // Check if response is from our motor
                if (rxId == expectedID && rxData[0] == expectedCmd) {
                    return true;
                }
            } else {
                xSemaphoreGive(canMutex);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    return false;  // Timeout
}

// Read motor status 2 (temperature, torque current, speed, encoder)
bool readMotorStatus2(uint32_t canID, MotorStatus& status) {
    if (!sendReadCommand(canID, READ_STATUS_2)) {
        return false;
    }

    uint8_t rxData[8];
    if (!readMotorResponse(canID, READ_STATUS_2, rxData, 50)) {
        return false;
    }

    status.temperature = (int8_t)rxData[1];
    status.torqueCurrent = (int16_t)(rxData[2] | (rxData[3] << 8));
    status.speed = (int16_t)(rxData[4] | (rxData[5] << 8));
    status.encoder = (uint16_t)(rxData[6] | (rxData[7] << 8));

    return true;
}

// Read motor status 1 (temperature, voltage, error flags)
bool readMotorStatus1(uint32_t canID, MotorStatus& status) {
    if (!sendReadCommand(canID, READ_STATUS_1_ERROR)) {
        return false;
    }

    uint8_t rxData[8];
    if (!readMotorResponse(canID, READ_STATUS_1_ERROR, rxData, 50)) {
        return false;
    }

    status.temperature = (int8_t)rxData[1];
    status.voltage = (uint16_t)(rxData[3] | (rxData[4] << 8));
    status.errorState = rxData[7];

    return true;
}

// Read motor angle (using multi-turn mode 0x92)
// Returns multi-turn angle (int64_t, supports positive/negative, no wrap)
bool readMotorAngle(uint32_t canID, MotorStatus& status) {
    if (!sendReadCommand(canID, READ_MULTI_ANGLE)) {
        return false;
    }

    uint8_t rxData[8];
    if (!readMotorResponse(canID, READ_MULTI_ANGLE, rxData, 50)) {
        return false;
    }

    // Multi-turn angle is int64_t in rxData[1-7] (7 bytes), unit: 0.01°/LSB
    // Positive = clockwise accumulated, Negative = counter-clockwise accumulated
    int64_t motorAngle = 0;
    // Copy 7 bytes from rxData[1-7] to motorAngle (little-endian)
    memcpy(&motorAngle, &rxData[1], 7);
    // Sign extend if bit 55 is set (negative value in 7-byte representation)
    if (rxData[7] & 0x80) {
        motorAngle |= 0xFF00000000000000LL;  // Set upper byte for sign extension
    }

    status.motorAngle = motorAngle;

    return true;
}

// Read acceleration (DEPRECATED - not used, calculated from speed instead)
// NOTE: Command 0x33 appears to not be implemented in motor firmware
bool readAcceleration(uint32_t canID, MotorStatus& status) {
    if (!sendReadCommand(canID, READ_ACCELERATION)) {
        return false;
    }

    uint8_t rxData[8];
    if (!readMotorResponse(canID, READ_ACCELERATION, rxData, 50)) {
        return false;
    }

    // Acceleration is int32_t, unit: 1dps/s
    status.acceleration = (int32_t)(rxData[4] | (rxData[5] << 8) |
                                    (rxData[6] << 16) | (rxData[7] << 24));

    return true;
}

// Read complete status for one motor
bool readMotorComplete(uint8_t motorID, uint32_t canID, MotorStatus& status) {
    status.motorID = motorID;

    bool success = true;

    if (!readMotorStatus1(canID, status)) {
        success = false;
    }

    if (!readMotorStatus2(canID, status)) {
        success = false;
    }

    if (!readMotorAngle(canID, status)) {
        success = false;
    }

    // NOTE: Command 0x33 (READ_ACCELERATION) is not implemented in motor firmware
    // Acceleration is calculated from speed changes in main.cpp instead
    // if (!readAcceleration(canID, status)) {
    //     success = false;
    // }

    if (success) {
        status.timestamp = millis();
    }

    return success;
}

// ============================================================================
// Motor Control Commands (Write Operations)
// ============================================================================

// Send motor shutdown command (0x80)
// Shuts down motor, clears running state and previous control instructions
bool sendMotorShutdown(uint32_t canID) {
    uint8_t txData[8] = {MOTOR_SHUTDOWN, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

    if (xSemaphoreTake(canMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        byte rc = CAN.sendMsgBuf(canID, 0, 8, txData);
        xSemaphoreGive(canMutex);

        if (rc == CAN_OK) {
            // Wait for response (motor echoes the command back)
            uint8_t rxData[8];
            return readMotorResponse(canID, MOTOR_SHUTDOWN, rxData, 50);
        }
    }
    return false;
}

// Send motor stop command (0x81)
// Stops motor but keeps it enabled
bool sendMotorStop(uint32_t canID) {
    uint8_t txData[8] = {MOTOR_STOP, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

    if (xSemaphoreTake(canMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        byte rc = CAN.sendMsgBuf(canID, 0, 8, txData);
        xSemaphoreGive(canMutex);

        if (rc == CAN_OK) {
            uint8_t rxData[8];
            return readMotorResponse(canID, MOTOR_STOP, rxData, 50);
        }
    }
    return false;
}
