#include <Arduino.h>
#include <SPI.h>
#include <mcp_can.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

/*
 * LK-TECH Motor Status Reader - FreeRTOS Optimized
 * Purpose: READ motor status data with high-frequency updates using FreeRTOS
 *
 * Performance improvements:
 * - Dedicated CAN reading task (high priority)
 * - Dedicated Serial output task (lower priority)
 * - Queue-based communication between tasks
 * - Non-blocking operations
 * - 10Hz update rate (100ms interval) - much faster than 1Hz
 *
 * Motor: LK-TECH M Series (MS/MF/MG/MH)
 * CAN Baud Rate: 1 Mbps
 * Motor ID: 1 (configurable)
 */

// MCP2515 SPI Configuration
constexpr int CAN_CS   = 5;
constexpr int SPI_SCK  = 18;
constexpr int SPI_MISO = 19;
constexpr int SPI_MOSI = 23;

MCP_CAN CAN(CAN_CS);

constexpr int LED_PIN = 2;
constexpr uint8_t MOTOR_ID = 1;  // Motor ID (1-32)

// CAN IDs (0x140 + Motor ID)
const uint32_t CAN_ID = 0x140 + MOTOR_ID;

// Update rate configuration
constexpr uint32_t UPDATE_INTERVAL_MS = 100;  // 10Hz update rate (was 1000ms = 1Hz)

// Read-only commands (no motor control)
enum MotorReadCommand : uint8_t {
    READ_PID_PARAMS         = 0x30,  // Read PID parameters
    READ_ACCELERATION       = 0x33,  // Read acceleration
    READ_ENCODER            = 0x90,  // Read encoder data
    READ_MULTI_ANGLE        = 0x92,  // Read multi-turn angle
    READ_SINGLE_ANGLE       = 0x94,  // Read single-turn angle
    READ_STATUS_1_ERROR     = 0x9A,  // Read status 1 and error flags
    READ_STATUS_2           = 0x9C,  // Read status 2 (temp, torque, speed, encoder)
    READ_STATUS_3           = 0x9D,  // Read status 3 (temp, phase currents)
};

// Motor status data structure
struct MotorStatus {
    // Status 1
    int8_t temperature;      // °C
    uint16_t voltage;        // 0.1V/LSB
    uint8_t errorState;      // Error flags

    // Status 2
    int16_t torqueCurrent;   // iq: -2048~2048 → -33A~33A
    int16_t speed;           // dps (degrees per second)
    uint16_t encoder;        // 0~16383 (14-bit)

    // Multi-turn angle
    int64_t motorAngle;      // 0.01°/LSB (multi-turn cumulative)

    // Timestamp
    uint32_t timestamp;      // millis() when read
};

// FreeRTOS objects
QueueHandle_t motorDataQueue;
SemaphoreHandle_t canMutex;
TaskHandle_t canReadTaskHandle;
TaskHandle_t serialOutputTaskHandle;

// Buffer for latest motor status
volatile MotorStatus latestStatus = {0};
volatile bool dataReady = false;

// Send a read command to motor
bool sendReadCommand(uint8_t cmd) {
    uint8_t txData[8] = {cmd, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

    if (xSemaphoreTake(canMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        byte rc = CAN.sendMsgBuf(CAN_ID, 0, 8, txData);
        xSemaphoreGive(canMutex);
        return (rc == CAN_OK);
    }
    return false;
}

// Read CAN response from motor (non-blocking)
bool readMotorResponse(uint8_t expectedCmd, uint8_t* rxData, uint32_t timeoutMs = 50) {
    uint32_t startTime = millis();

    while (millis() - startTime < timeoutMs) {
        if (xSemaphoreTake(canMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
            if (CAN.checkReceive() == CAN_MSGAVAIL) {
                unsigned long rxId;
                byte len;

                CAN.readMsgBuf(&rxId, &len, rxData);
                xSemaphoreGive(canMutex);

                // Check if response is from our motor
                if (rxId == CAN_ID && rxData[0] == expectedCmd) {
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
bool readMotorStatus2(MotorStatus& status) {
    if (!sendReadCommand(READ_STATUS_2)) {
        return false;
    }

    uint8_t rxData[8];
    if (!readMotorResponse(READ_STATUS_2, rxData, 50)) {
        return false;
    }

    // Parse response
    status.temperature = (int8_t)rxData[1];
    status.torqueCurrent = (int16_t)(rxData[2] | (rxData[3] << 8));
    status.speed = (int16_t)(rxData[4] | (rxData[5] << 8));
    status.encoder = (uint16_t)(rxData[6] | (rxData[7] << 8));

    return true;
}

// Read motor status 1 (temperature, voltage, error flags)
bool readMotorStatus1(MotorStatus& status) {
    if (!sendReadCommand(READ_STATUS_1_ERROR)) {
        return false;
    }

    uint8_t rxData[8];
    if (!readMotorResponse(READ_STATUS_1_ERROR, rxData, 50)) {
        return false;
    }

    // Parse response
    status.temperature = (int8_t)rxData[1];
    status.voltage = (uint16_t)(rxData[3] | (rxData[4] << 8));
    status.errorState = rxData[7];

    return true;
}

// Read multi-turn angle
bool readMultiTurnAngle(MotorStatus& status) {
    if (!sendReadCommand(READ_MULTI_ANGLE)) {
        return false;
    }

    uint8_t rxData[8];
    if (!readMotorResponse(READ_MULTI_ANGLE, rxData, 50)) {
        return false;
    }

    // Parse response (int64_t, 7 bytes used)
    status.motorAngle = (int64_t)rxData[1]
                      | ((int64_t)rxData[2] << 8)
                      | ((int64_t)rxData[3] << 16)
                      | ((int64_t)rxData[4] << 24)
                      | ((int64_t)rxData[5] << 32)
                      | ((int64_t)rxData[6] << 40);

    return true;
}

// CAN Reading Task (High Priority) - Runs on Core 1
void canReadTask(void *parameter) {
    MotorStatus status = {0};

    while (true) {
        bool success = true;

        // Read Status 1 (voltage, temperature, error)
        if (!readMotorStatus1(status)) {
            success = false;
        }

        // Read Status 2 (torque, speed, encoder)
        if (!readMotorStatus2(status)) {
            success = false;
        }

        // Read multi-turn angle
        if (!readMultiTurnAngle(status)) {
            success = false;
        }

        if (success) {
            status.timestamp = millis();

            // Send to queue (non-blocking)
            xQueueSend(motorDataQueue, &status, 0);

            // Also update latest status directly for fast access
            latestStatus = status;
            dataReady = true;

            // Blink LED on successful read
            digitalWrite(LED_PIN, !digitalRead(LED_PIN));
        }

        // Wait before next read cycle (10Hz = 100ms)
        vTaskDelay(pdMS_TO_TICKS(UPDATE_INTERVAL_MS));
    }
}

// Serial Output Task (Lower Priority) - Runs on Core 0
void serialOutputTask(void *parameter) {
    MotorStatus status;
    uint32_t outputCount = 0;

    while (true) {
        // Wait for data from queue (blocking with timeout)
        if (xQueueReceive(motorDataQueue, &status, pdMS_TO_TICKS(200)) == pdTRUE) {
            // Output in compact format for GUI parsing
            Serial.print("[");
            Serial.print(status.timestamp);
            Serial.print("] ");

            Serial.print("T:");
            Serial.print(status.temperature);
            Serial.print(" V:");
            Serial.print(status.voltage * 0.1, 1);
            Serial.print(" I:");
            float actualCurrent = (float)status.torqueCurrent * 33.0 / 2048.0;
            Serial.print(actualCurrent, 2);
            Serial.print(" S:");
            Serial.print(status.speed);
            Serial.print(" E:");
            Serial.print(status.encoder);
            Serial.print(" A:");
            float angleDeg = (float)status.motorAngle * 0.01;
            Serial.print(angleDeg, 2);
            Serial.print(" ERR:0x");
            Serial.print(status.errorState, HEX);
            Serial.println();

            outputCount++;

            // Print detailed status every 10 reads (1 second at 10Hz)
            if (outputCount % 10 == 0) {
                Serial.println("--- Motor Status (detailed) ---");
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

                Serial.print("Encoder:         ");
                Serial.print(status.encoder);
                Serial.println(" (0~16383)");

                Serial.print("Multi-turn Angle:");
                Serial.print(angleDeg, 2);
                Serial.print(" °  (");
                Serial.print(angleDeg / 360.0, 2);
                Serial.println(" turns)");

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
    Serial.println("   LK-TECH Motor Status Reader");
    Serial.println("   FreeRTOS Optimized - 10Hz");
    Serial.println("========================================\n");

    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);

    // Initialize SPI
    Serial.println("[1] Initializing SPI...");
    SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI);
    Serial.println("    SPI initialized");

    // Initialize MCP2515 at 1Mbps (motor requires 1Mbps!)
    Serial.println("\n[2] Initializing MCP2515...");
    Serial.println("    Trying 1MBPS @ 8MHz...");

    byte result = CAN.begin(MCP_ANY, CAN_1000KBPS, MCP_8MHZ);

    if (result != CAN_OK) {
        Serial.println("    Failed! Trying 1MBPS @ 16MHz...");
        result = CAN.begin(MCP_ANY, CAN_1000KBPS, MCP_16MHZ);
    }

    if (result != CAN_OK) {
        Serial.println("\n[ERROR] MCP2515 initialization FAILED!");
        Serial.println("Check:");
        Serial.println("  1. SPI wiring (CS, SCK, MISO, MOSI)");
        Serial.println("  2. MCP2515 power (VCC=5V, GND)");
        Serial.println("  3. Crystal frequency (8MHz or 16MHz)");
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

    // Create FreeRTOS objects
    Serial.println("\n[4] Creating FreeRTOS tasks...");

    // Create queue for motor data (capacity: 10 items)
    motorDataQueue = xQueueCreate(10, sizeof(MotorStatus));
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

    // Create CAN reading task (high priority, core 1)
    xTaskCreatePinnedToCore(
        canReadTask,           // Task function
        "CAN_Read",            // Task name
        4096,                  // Stack size
        NULL,                  // Parameters
        3,                     // Priority (high)
        &canReadTaskHandle,    // Task handle
        1                      // Core 1
    );

    // Create Serial output task (lower priority, core 0)
    xTaskCreatePinnedToCore(
        serialOutputTask,      // Task function
        "Serial_Output",       // Task name
        4096,                  // Stack size
        NULL,                  // Parameters
        1,                     // Priority (lower)
        &serialOutputTaskHandle, // Task handle
        0                      // Core 0
    );

    Serial.println("    Tasks created successfully!");
    Serial.println("\n[OK] Motor Status Reader ready!");
    Serial.println("========================================");
    Serial.print("Monitoring Motor ID=");
    Serial.println(MOTOR_ID);
    Serial.print("Update rate: ");
    Serial.print(1000 / UPDATE_INTERVAL_MS);
    Serial.println(" Hz (10x faster!)\n");
}

void loop() {
    // Main loop is now empty - everything handled by FreeRTOS tasks
    // This prevents blocking the watchdog
    vTaskDelay(pdMS_TO_TICKS(1000));
}
