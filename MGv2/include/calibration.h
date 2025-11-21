#pragma once
#include "motor_protocol.h"
#include <mcp_can.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include <Arduino.h>

/*
 * Motor Calibration Functions
 * Software calibration (offset) and hardware zero setting (ROM)
 */

// External references
extern MCP_CAN CAN;
extern SemaphoreHandle_t canMutex;
extern TaskHandle_t canReadTaskHandle;

// Software angle offset (safe, no ROM write)
extern int64_t motor1_angle_offset;
extern int64_t motor2_angle_offset;

// Latest motor status (for calibration commands)
extern volatile int64_t motor1_latest_angle;
extern volatile int64_t motor2_latest_angle;
extern volatile bool motor1_data_valid;
extern volatile bool motor2_data_valid;

// Set motor zero position to ROM - Command 0x19 (PERMANENT - requires reboot)
bool setMotorZeroToROM(uint32_t canID) {
    uint8_t txData[8] = {0x19, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

    Serial.print("[DEBUG] Sending 0x19 to CAN ID: 0x");
    Serial.println(canID, HEX);

    // Suspend CAN read task to avoid interference
    Serial.println("[DEBUG] Suspending CAN read task...");
    vTaskSuspend(canReadTaskHandle);
    vTaskDelay(pdMS_TO_TICKS(50));  // Wait for task to fully suspend

    // Clear CAN receive buffer
    Serial.println("[DEBUG] Clearing CAN buffer...");
    if (xSemaphoreTake(canMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        while (CAN.checkReceive() == CAN_MSGAVAIL) {
            unsigned long rxId;
            byte len;
            uint8_t dummyData[8];
            CAN.readMsgBuf(&rxId, &len, dummyData);
        }
        xSemaphoreGive(canMutex);
        Serial.println("[DEBUG] Buffer cleared.");
    }

    bool success = false;

    if (xSemaphoreTake(canMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        byte rc = CAN.sendMsgBuf(canID, 0, 8, txData);
        xSemaphoreGive(canMutex);

        if (rc == CAN_OK) {
            Serial.println("[DEBUG] Command sent successfully, waiting for response...");

            // Wait for response confirmation
            uint8_t rxData[8] = {0};
            uint32_t startTime = millis();

            while (millis() - startTime < 500) {  // 500ms timeout
                if (xSemaphoreTake(canMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
                    if (CAN.checkReceive() == CAN_MSGAVAIL) {
                        unsigned long rxId;
                        byte len;

                        CAN.readMsgBuf(&rxId, &len, rxData);
                        xSemaphoreGive(canMutex);

                        Serial.print("[DEBUG] Received from 0x");
                        Serial.print(rxId, HEX);
                        Serial.print(", CMD: 0x");
                        Serial.print(rxData[0], HEX);
                        Serial.print(", Data: ");
                        for (int i = 0; i < len; i++) {
                            Serial.print("0x");
                            if (rxData[i] < 0x10) Serial.print("0");
                            Serial.print(rxData[i], HEX);
                            Serial.print(" ");
                        }
                        Serial.println();

                        // Check if response is from our motor with 0x19 command
                        if (rxId == canID && rxData[0] == 0x19) {
                            uint16_t encoderOffset = rxData[6] | (rxData[7] << 8);
                            Serial.print("[DEBUG] Valid response! Encoder offset set to: ");
                            Serial.println(encoderOffset);
                            success = true;
                            break;
                        }
                    } else {
                        xSemaphoreGive(canMutex);
                    }
                }
                vTaskDelay(pdMS_TO_TICKS(1));
            }

            if (!success) {
                Serial.println("[DEBUG] Response timeout!");
            }
        } else {
            Serial.print("[DEBUG] Send failed with error code: ");
            Serial.println(rc);
        }
    } else {
        Serial.println("[DEBUG] Failed to acquire mutex!");
    }

    // Resume CAN read task
    Serial.println("[DEBUG] Resuming CAN read task...");
    vTaskResume(canReadTaskHandle);

    return success;
}

// Write encoder offset to ROM as motor zero point - Command 0x91
// encoderOffset: uint16_t, max 65535 (protocol DATA[6-7] only supports 2 bytes)
// Note: Motor uses 18-bit encoder (0~262143), but this command only sets lower 16 bits
// TX: [0x91] [0x00] [0x00] [0x00] [0x00] [0x00] [offset_L] [offset_H]
// RX: Motor echoes the same frame back
bool writeEncoderOffsetToROM(uint32_t canID, uint16_t encoderOffset) {
    uint8_t txData[8] = {
        WRITE_ENCODER_OFFSET,                   // DATA[0]: Command 0x91
        0x00,                                    // DATA[1]: NULL
        0x00,                                    // DATA[2]: NULL
        0x00,                                    // DATA[3]: NULL
        0x00,                                    // DATA[4]: NULL
        0x00,                                    // DATA[5]: NULL
        (uint8_t)(encoderOffset & 0xFF),        // DATA[6]: offset low byte
        (uint8_t)((encoderOffset >> 8) & 0xFF)  // DATA[7]: offset high byte
    };

    Serial.print("[CAL] Writing encoder offset to ROM: ");
    Serial.print(encoderOffset);
    Serial.print(" (0x");
    Serial.print(encoderOffset, HEX);
    Serial.println(")");

    Serial.print("[CAL] TX: ");
    for (int i = 0; i < 8; i++) {
        Serial.print("0x");
        if (txData[i] < 16) Serial.print("0");
        Serial.print(txData[i], HEX);
        Serial.print(" ");
    }
    Serial.println();

    // Suspend CAN read task to avoid interference
    vTaskSuspend(canReadTaskHandle);
    vTaskDelay(pdMS_TO_TICKS(50));

    // Clear CAN receive buffer
    if (xSemaphoreTake(canMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        while (CAN.checkReceive() == CAN_MSGAVAIL) {
            unsigned long rxId;
            byte len;
            uint8_t dummyData[8];
            CAN.readMsgBuf(&rxId, &len, dummyData);
        }
        xSemaphoreGive(canMutex);
    }

    bool success = false;

    if (xSemaphoreTake(canMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        byte rc = CAN.sendMsgBuf(canID, 0, 8, txData);
        xSemaphoreGive(canMutex);

        if (rc == CAN_OK) {
            Serial.println("[CAL] Command sent, waiting for response...");

            uint8_t rxData[8] = {0};
            uint32_t startTime = millis();

            while (millis() - startTime < 500) {
                if (xSemaphoreTake(canMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
                    if (CAN.checkReceive() == CAN_MSGAVAIL) {
                        unsigned long rxId;
                        byte len;
                        CAN.readMsgBuf(&rxId, &len, rxData);
                        xSemaphoreGive(canMutex);

                        if (rxId == canID && rxData[0] == WRITE_ENCODER_OFFSET) {
                            uint16_t confirmedOffset = rxData[6] | (rxData[7] << 8);
                            Serial.print("[CAL] Confirmed! Encoder offset set to: ");
                            Serial.println(confirmedOffset);
                            success = true;
                            break;
                        }
                    } else {
                        xSemaphoreGive(canMutex);
                    }
                }
                vTaskDelay(pdMS_TO_TICKS(1));
            }

            if (!success) {
                Serial.println("[CAL] ERROR: Response timeout!");
            }
        } else {
            Serial.println("[CAL] ERROR: CAN send failed!");
        }
    } else {
        Serial.println("[CAL] ERROR: Failed to acquire mutex!");
    }

    // Resume CAN read task
    vTaskResume(canReadTaskHandle);

    return success;
}

// Clear motor angle (reset current position to zero) - Command 0x95
// NOTE: This command is NOT IMPLEMENTED in motor firmware (see datasheet)
bool clearMotorAngle(uint32_t canID) {
    uint8_t txData[8] = {0x95, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

    if (xSemaphoreTake(canMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        byte rc = CAN.sendMsgBuf(canID, 0, 8, txData);
        xSemaphoreGive(canMutex);

        if (rc == CAN_OK) {
            // Wait for response confirmation
            uint8_t rxData[8];
            uint32_t startTime = millis();

            while (millis() - startTime < 100) {
                if (xSemaphoreTake(canMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
                    if (CAN.checkReceive() == CAN_MSGAVAIL) {
                        unsigned long rxId;
                        byte len;
                        CAN.readMsgBuf(&rxId, &len, rxData);
                        xSemaphoreGive(canMutex);

                        if (rxId == canID && rxData[0] == 0x95) {
                            return true;
                        }
                    } else {
                        xSemaphoreGive(canMutex);
                    }
                }
                vTaskDelay(pdMS_TO_TICKS(1));
            }
        }
    }
    return false;
}
