#include <Arduino.h>
#include <SPI.h>
#include <mcp_can.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

/*
 * LK-TECH Dual Motor Status Reader - FreeRTOS Optimized
 * Purpose: READ TWO motors (ID 1 and ID 2) status data with high-frequency updates
 *
 * Motor: LK-TECH M Series (MS/MF/MG/MH)
 * CAN Baud Rate: 1 Mbps
 * Motor IDs: 1 and 2
 */

// MCP2515 SPI Configuration
constexpr int CAN_CS   = 5;
constexpr int SPI_SCK  = 18;
constexpr int SPI_MISO = 19;
constexpr int SPI_MOSI = 23;

MCP_CAN CAN(CAN_CS);

constexpr int LED_PIN = 2;
constexpr uint8_t MOTOR_ID_1 = 1;  // First motor ID
constexpr uint8_t MOTOR_ID_2 = 2;  // Second motor ID

// CAN IDs (0x140 + Motor ID)
const uint32_t CAN_ID_1 = 0x140 + MOTOR_ID_1;
const uint32_t CAN_ID_2 = 0x140 + MOTOR_ID_2;

// Update rate configuration
constexpr uint32_t UPDATE_INTERVAL_MS = 100;  // 10Hz update rate

// Read-only commands (no motor control)
enum MotorReadCommand : uint8_t {
    READ_PID_PARAMS         = 0x30,
    READ_ACCELERATION       = 0x33,
    READ_ENCODER            = 0x90,
    READ_MULTI_ANGLE        = 0x92,
    READ_SINGLE_ANGLE       = 0x94,
    READ_STATUS_1_ERROR     = 0x9A,
    READ_STATUS_2           = 0x9C,
    READ_STATUS_3           = 0x9D,
};

// Motor status data structure
struct MotorStatus {
    uint8_t motorID;         // Motor ID (1 or 2)
    int8_t temperature;      // °C
    uint16_t voltage;        // 0.1V/LSB
    uint8_t errorState;      // Error flags
    int16_t torqueCurrent;   // iq: -2048~2048 → -33A~33A
    int16_t speed;           // dps (degrees per second)
    int16_t acceleration;    // dps/s (degrees per second squared)
    uint16_t encoder;        // 0~16383 (14-bit)
    int64_t motorAngle;      // 0.01°/LSB (multi-turn cumulative)
    uint32_t timestamp;      // millis() when read
};

// FreeRTOS objects
QueueHandle_t motorDataQueue;
SemaphoreHandle_t canMutex;
TaskHandle_t canReadTaskHandle;
TaskHandle_t serialOutputTaskHandle;

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

// Read multi-turn angle
bool readMultiTurnAngle(uint32_t canID, MotorStatus& status) {
    if (!sendReadCommand(canID, READ_MULTI_ANGLE)) {
        return false;
    }

    uint8_t rxData[8];
    if (!readMotorResponse(canID, READ_MULTI_ANGLE, rxData, 50)) {
        return false;
    }

    status.motorAngle = (int64_t)rxData[1]
                      | ((int64_t)rxData[2] << 8)
                      | ((int64_t)rxData[3] << 16)
                      | ((int64_t)rxData[4] << 24)
                      | ((int64_t)rxData[5] << 32)
                      | ((int64_t)rxData[6] << 40);

    return true;
}

// Read acceleration
bool readAcceleration(uint32_t canID, MotorStatus& status) {
    if (!sendReadCommand(canID, READ_ACCELERATION)) {
        return false;
    }

    uint8_t rxData[8];
    if (!readMotorResponse(canID, READ_ACCELERATION, rxData, 50)) {
        return false;
    }

    // Acceleration is int32_t, unit: 1dps/s
    int32_t accel = (int32_t)(rxData[4] | (rxData[5] << 8) | (rxData[6] << 16) | (rxData[7] << 24));
    status.acceleration = (int16_t)(accel);  // Truncate to int16_t for compact storage

    return true;
}

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
            return readMotorResponse(canID, 0x95, rxData, 100);
        }
    }
    return false;
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

    if (!readMultiTurnAngle(canID, status)) {
        success = false;
    }

    if (!readAcceleration(canID, status)) {
        success = false;
    }

    if (success) {
        status.timestamp = millis();
    }

    return success;
}

// CAN Reading Task (High Priority) - Runs on Core 1
void canReadTask(void *parameter) {
    MotorStatus status1 = {0};
    MotorStatus status2 = {0};

    while (true) {
        // Read Motor 1
        if (readMotorComplete(MOTOR_ID_1, CAN_ID_1, status1)) {
            xQueueSend(motorDataQueue, &status1, 0);
            digitalWrite(LED_PIN, HIGH);
        }

        // Small delay between motors
        vTaskDelay(pdMS_TO_TICKS(10));

        // Read Motor 2
        if (readMotorComplete(MOTOR_ID_2, CAN_ID_2, status2)) {
            xQueueSend(motorDataQueue, &status2, 0);
            digitalWrite(LED_PIN, LOW);
        }

        // Wait before next read cycle (10Hz = 100ms total)
        vTaskDelay(pdMS_TO_TICKS(90));
    }
}

// Serial Output Task (Lower Priority) - Runs on Core 0
void serialOutputTask(void *parameter) {
    MotorStatus status;
    uint32_t outputCount1 = 0;
    uint32_t outputCount2 = 0;

    while (true) {
        // Check for serial commands (non-blocking)
        if (Serial.available() > 0) {
            String cmd = Serial.readStringUntil('\n');
            cmd.trim();

            if (cmd == "SET_ZERO_M1" || cmd == "ZERO1") {
                Serial.println("[CMD] Setting Motor 1 zero position to ROM (0x19)...");
                Serial.println("[WARNING] Requires MCU reboot to take effect!");
                if (setMotorZeroToROM(CAN_ID_1)) {
                    Serial.println("[OK] Motor 1 zero position set! Please reboot MCU.");
                } else {
                    Serial.println("[ERROR] Motor 1 set zero failed!");
                }
            }
            else if (cmd == "SET_ZERO_M2" || cmd == "ZERO2") {
                Serial.println("[CMD] Setting Motor 2 zero position to ROM (0x19)...");
                Serial.println("[WARNING] Requires MCU reboot to take effect!");
                if (setMotorZeroToROM(CAN_ID_2)) {
                    Serial.println("[OK] Motor 2 zero position set! Please reboot MCU.");
                } else {
                    Serial.println("[ERROR] Motor 2 set zero failed!");
                }
            }
            else if (cmd == "RESET_M1" || cmd == "RESET1") {
                Serial.println("[CMD] Resetting Motor 1 angle to zero (0x95 - NOT IMPLEMENTED)...");
                if (clearMotorAngle(CAN_ID_1)) {
                    Serial.println("[OK] Motor 1 angle reset successful!");
                } else {
                    Serial.println("[ERROR] Motor 1 angle reset failed!");
                }
            }
            else if (cmd == "RESET_M2" || cmd == "RESET2") {
                Serial.println("[CMD] Resetting Motor 2 angle to zero (0x95 - NOT IMPLEMENTED)...");
                if (clearMotorAngle(CAN_ID_2)) {
                    Serial.println("[OK] Motor 2 angle reset successful!");
                } else {
                    Serial.println("[ERROR] Motor 2 angle reset failed!");
                }
            }
            else if (cmd == "RESET_ALL" || cmd == "RESET") {
                Serial.println("[CMD] Resetting ALL motor angles to zero (0x95 - NOT IMPLEMENTED)...");
                bool m1_ok = clearMotorAngle(CAN_ID_1);
                vTaskDelay(pdMS_TO_TICKS(50));
                bool m2_ok = clearMotorAngle(CAN_ID_2);

                if (m1_ok && m2_ok) {
                    Serial.println("[OK] All motor angles reset successful!");
                } else {
                    Serial.println("[ERROR] Some motor resets failed!");
                }
            }
            else if (cmd == "HELP") {
                Serial.println("\n=== Available Commands ===");
                Serial.println("SET_ZERO_M1 or ZERO1  - Set Motor 1 zero to ROM (0x19, permanent)");
                Serial.println("SET_ZERO_M2 or ZERO2  - Set Motor 2 zero to ROM (0x19, permanent)");
                Serial.println("RESET_M1 or RESET1    - Reset Motor 1 angle (0x95, not implemented)");
                Serial.println("RESET_M2 or RESET2    - Reset Motor 2 angle (0x95, not implemented)");
                Serial.println("RESET_ALL or RESET    - Reset both motors (0x95, not implemented)");
                Serial.println("HELP                  - Show this help");
                Serial.println("NOTE: SET_ZERO commands require MCU reboot to take effect");
                Serial.println("========================\n");
            }
        }

        // Wait for data from queue (blocking with timeout)
        if (xQueueReceive(motorDataQueue, &status, pdMS_TO_TICKS(200)) == pdTRUE) {
            // Output in compact format for GUI parsing
            Serial.print("[");
            Serial.print(status.timestamp);
            Serial.print("] M:");
            Serial.print(status.motorID);
            Serial.print(" T:");
            Serial.print(status.temperature);
            Serial.print(" V:");
            Serial.print(status.voltage * 0.1, 1);
            Serial.print(" I:");
            float actualCurrent = (float)status.torqueCurrent * 33.0 / 2048.0;
            Serial.print(actualCurrent, 2);
            Serial.print(" S:");
            Serial.print(status.speed);
            Serial.print(" ACC:");
            Serial.print(status.acceleration);
            Serial.print(" E:");
            Serial.print(status.encoder);
            Serial.print(" A:");
            // Calculate angle in degrees (move outside if block for wider scope)
            float angleDeg = (float)status.motorAngle * 0.01;
            // Check for overflow: valid range is roughly ±2^53 (float precision limit)
            if (status.motorAngle > 9007199254740992LL || status.motorAngle < -9007199254740992LL) {
                Serial.print("ovf");
            } else {
                Serial.print(angleDeg, 2);
            }
            Serial.print(" ERR:0x");
            Serial.print(status.errorState, HEX);
            Serial.println();

            // Track output counts per motor
            if (status.motorID == MOTOR_ID_1) {
                outputCount1++;
            } else if (status.motorID == MOTOR_ID_2) {
                outputCount2++;
            }

            // Print detailed status every 10 reads per motor (1 second at 10Hz)
            if ((status.motorID == MOTOR_ID_1 && outputCount1 % 10 == 0) ||
                (status.motorID == MOTOR_ID_2 && outputCount2 % 10 == 0)) {
                Serial.print("--- Motor ");
                Serial.print(status.motorID);
                Serial.println(" Status (detailed) ---");
                Serial.print("Temperature:     ");
                Serial.print(status.temperature);
                Serial.println(" °C");

                Serial.print("Voltage:         ");
                Serial.print(status.voltage * 0.1, 1);
                Serial.println(" V");

                Serial.print("Torque Current:  ");
                Serial.print(actualCurrent, 2);
                Serial.print(" A  (raw=");
                Serial.print(status.torqueCurrent);
                Serial.println(")");

                Serial.print("Speed:           ");
                Serial.print(status.speed);
                Serial.println(" dps");

                Serial.print("Acceleration:    ");
                Serial.print(status.acceleration);
                Serial.println(" dps/s");

                Serial.print("Encoder:         ");
                Serial.print(status.encoder);
                Serial.println(" (0~16383)");

                Serial.print("Multi-turn Angle:");
                // Check for overflow
                if (status.motorAngle > 9007199254740992LL || status.motorAngle < -9007199254740992LL) {
                    Serial.print("ovf");
                    Serial.print(" °  (ovf turns) [RAW: 0x");
                    Serial.print((uint32_t)(status.motorAngle >> 32), HEX);
                    Serial.print((uint32_t)(status.motorAngle & 0xFFFFFFFF), HEX);
                    Serial.println("]");
                } else {
                    Serial.print(angleDeg, 2);
                    Serial.print(" °  (");
                    Serial.print(angleDeg / 360.0, 2);
                    Serial.println(" turns)");
                }

                Serial.print("Error State:     0x");
                Serial.print(status.errorState, HEX);
                if (status.errorState & 0x01) Serial.print(" [LOW_VOLTAGE]");
                if (status.errorState & 0x08) Serial.print(" [OVER_TEMP]");
                Serial.println();
                Serial.println("--------------------\n");
            }
        }
    }
}

void setup() {
    Serial.begin(115200);
    delay(1000);

    Serial.println("\n========================================");
    Serial.println("   LK-TECH Dual Motor Status Reader");
    Serial.println("   FreeRTOS Optimized - 10Hz");
    Serial.println("========================================\n");

    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);

    // Initialize SPI
    Serial.println("[1] Initializing SPI...");
    SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI);
    Serial.println("    SPI initialized");

    // Initialize MCP2515 at 1Mbps
    Serial.println("\n[2] Initializing MCP2515...");
    Serial.println("    Trying 1MBPS @ 8MHz...");

    byte result = CAN.begin(MCP_ANY, CAN_1000KBPS, MCP_8MHZ);

    if (result != CAN_OK) {
        Serial.println("    Failed! Trying 1MBPS @ 16MHz...");
        result = CAN.begin(MCP_ANY, CAN_1000KBPS, MCP_16MHZ);
    }

    if (result != CAN_OK) {
        Serial.println("\n[ERROR] MCP2515 initialization FAILED!");
        while (1) {
            digitalWrite(LED_PIN, !digitalRead(LED_PIN));
            delay(200);
        }
    }

    Serial.println("    MCP2515 initialized successfully!");

    // Set to NORMAL mode
    Serial.println("\n[3] Setting NORMAL mode...");
    CAN.setMode(MCP_NORMAL);
    Serial.println("    Mode set to NORMAL");

    // Create FreeRTOS objects FIRST (before using clearMotorAngle)
    Serial.println("\n[4] Creating FreeRTOS objects...");

    // Create queue for motor data (capacity: 20 items for both motors)
    motorDataQueue = xQueueCreate(20, sizeof(MotorStatus));
    if (motorDataQueue == NULL) {
        Serial.println("    [ERROR] Failed to create queue!");
        while(1);
    }

    // Create mutex for CAN bus access
    canMutex = xSemaphoreCreateMutex();
    if (canMutex == NULL) {
        Serial.println("    [ERROR] Failed to create mutex!");
        while(1);
    }
    Serial.println("    FreeRTOS objects created!");

    // NOTE: Auto-reset DISABLED because 0x95 command is not implemented in motor firmware
    // Use SET_ZERO_M1/SET_ZERO_M2 commands manually if needed (requires reboot)
    Serial.println("\n[5] Skipping auto-reset (use SET_ZERO commands if needed)...");

    // Create tasks
    Serial.println("\n[6] Creating FreeRTOS tasks...");

    // Create CAN reading task (high priority, core 1)
    xTaskCreatePinnedToCore(
        canReadTask,
        "CAN_Read",
        4096,
        NULL,
        3,
        &canReadTaskHandle,
        1
    );

    // Create Serial output task (lower priority, core 0)
    xTaskCreatePinnedToCore(
        serialOutputTask,
        "Serial_Output",
        4096,
        NULL,
        1,
        &serialOutputTaskHandle,
        0
    );

    Serial.println("    Tasks created successfully!");
    Serial.println("\n[OK] Dual Motor Status Reader ready!");
    Serial.println("========================================");
    Serial.print("Monitoring Motors: ID=");
    Serial.print(MOTOR_ID_1);
    Serial.print(" and ID=");
    Serial.println(MOTOR_ID_2);
    Serial.print("Update rate: ");
    Serial.print(1000 / UPDATE_INTERVAL_MS);
    Serial.println(" Hz per motor\n");
}

void loop() {
    // Main loop is now empty - everything handled by FreeRTOS tasks
    vTaskDelay(pdMS_TO_TICKS(1000));
}
