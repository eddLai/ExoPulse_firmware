#include <Arduino.h>
#include <WiFi.h>
#include <SPI.h>
#include <mcp_can.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

/*
 * ExoPulse - WiFi Motor Control & Monitoring
 *
 * Features:
 * - Dual motor CAN bus control (Motor ID 1 & 2)
 * - WiFi TCP Server for real-time data streaming
 * - FreeRTOS multi-tasking
 * - Remote calibration commands via WiFi
 *
 * WiFi Configuration:
 * - SSID: ExoPulse
 * - Password: 12345666
 * - Port: 8888
 */

// WiFi Configuration
const char* ssid = "ExoPulse";
const char* password = "12345666";
const uint16_t serverPort = 8888;

// MCP2515 SPI Configuration
constexpr int CAN_CS   = 5;
constexpr int SPI_SCK  = 18;
constexpr int SPI_MISO = 19;
constexpr int SPI_MOSI = 23;

MCP_CAN CAN(CAN_CS);
WiFiServer server(serverPort);
WiFiClient client;

constexpr int LED_PIN = 2;
constexpr uint8_t MOTOR_ID_1 = 1;
constexpr uint8_t MOTOR_ID_2 = 2;

// CAN IDs
const uint32_t CAN_ID_1 = 0x140 + MOTOR_ID_1;
const uint32_t CAN_ID_2 = 0x140 + MOTOR_ID_2;

// Update rate
constexpr uint32_t UPDATE_INTERVAL_MS = 100;  // 10Hz

// Motor commands
enum MotorReadCommand : uint8_t {
    READ_ACCELERATION       = 0x33,
    READ_MULTI_ANGLE        = 0x92,
    READ_STATUS_1_ERROR     = 0x9A,
    READ_STATUS_2           = 0x9C,
};

// Motor status structure
struct MotorStatus {
    uint8_t motorID;
    int8_t temperature;
    uint16_t voltage;
    uint8_t errorState;
    int16_t torqueCurrent;
    int16_t speed;
    int16_t acceleration;
    uint16_t encoder;
    int64_t motorAngle;
    uint32_t timestamp;
};

// FreeRTOS objects
QueueHandle_t motorDataQueue;
SemaphoreHandle_t canMutex;
SemaphoreHandle_t wifiMutex;
TaskHandle_t canReadTaskHandle;
TaskHandle_t wifiOutputTaskHandle;

// Software angle offset
int64_t motor1_angle_offset = 0;
int64_t motor2_angle_offset = 0;

// Latest motor status
volatile int64_t motor1_latest_angle = 0;
volatile int64_t motor2_latest_angle = 0;
volatile bool motor1_data_valid = false;
volatile bool motor2_data_valid = false;

// WiFi status
volatile bool wifi_connected = false;
volatile bool client_connected = false;

// Send CAN command
bool sendReadCommand(uint32_t canID, uint8_t cmd) {
    uint8_t txData[8] = {cmd, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

    if (xSemaphoreTake(canMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        byte rc = CAN.sendMsgBuf(canID, 0, 8, txData);
        xSemaphoreGive(canMutex);
        return (rc == CAN_OK);
    }
    return false;
}

// Read CAN response
bool readMotorResponse(uint32_t expectedID, uint8_t expectedCmd, uint8_t* rxData, uint32_t timeoutMs = 50) {
    uint32_t startTime = millis();

    while (millis() - startTime < timeoutMs) {
        if (xSemaphoreTake(canMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
            if (CAN.checkReceive() == CAN_MSGAVAIL) {
                unsigned long rxId;
                byte len;

                if (CAN.readMsgBuf(&rxId, &len, rxData) == CAN_OK) {
                    xSemaphoreGive(canMutex);
                    if (rxId == expectedID && rxData[0] == expectedCmd) {
                        return true;
                    }
                } else {
                    xSemaphoreGive(canMutex);
                }
            } else {
                xSemaphoreGive(canMutex);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    return false;
}

// Read all motor data
bool readMotorAllData(uint8_t motorID, MotorStatus& status) {
    uint32_t canID = 0x140 + motorID;
    uint8_t rxData[8];

    status.motorID = motorID;
    status.timestamp = millis();

    // Read Status 1 (Temperature, Voltage, Error)
    if (sendReadCommand(canID, READ_STATUS_1_ERROR)) {
        if (readMotorResponse(canID, READ_STATUS_1_ERROR, rxData, 50)) {
            status.temperature = (int8_t)rxData[1];
            status.voltage = (uint16_t)(rxData[3] | (rxData[4] << 8));
            status.errorState = rxData[7];
        }
    }
    vTaskDelay(pdMS_TO_TICKS(5));

    // Read Status 2 (Current, Speed, Encoder)
    if (sendReadCommand(canID, READ_STATUS_2)) {
        if (readMotorResponse(canID, READ_STATUS_2, rxData, 50)) {
            status.torqueCurrent = (int16_t)(rxData[2] | (rxData[3] << 8));
            status.speed = (int16_t)(rxData[4] | (rxData[5] << 8));
            status.encoder = (uint16_t)(rxData[6] | (rxData[7] << 8));
        }
    }
    vTaskDelay(pdMS_TO_TICKS(5));

    // Read Multi-turn Angle
    if (sendReadCommand(canID, READ_MULTI_ANGLE)) {
        if (readMotorResponse(canID, READ_MULTI_ANGLE, rxData, 50)) {
            int64_t rawAngle = (int64_t)rxData[1]
                             | ((int64_t)rxData[2] << 8)
                             | ((int64_t)rxData[3] << 16)
                             | ((int64_t)rxData[4] << 24)
                             | ((int64_t)rxData[5] << 32)
                             | ((int64_t)rxData[6] << 40);

            // Apply software calibration offset
            int64_t offset = (motorID == 1) ? motor1_angle_offset : motor2_angle_offset;
            status.motorAngle = rawAngle - offset;

            // Update global latest angle
            if (motorID == 1) {
                motor1_latest_angle = rawAngle;
                motor1_data_valid = true;
            } else {
                motor2_latest_angle = rawAngle;
                motor2_data_valid = true;
            }
        }
    }
    vTaskDelay(pdMS_TO_TICKS(5));

    // Read Acceleration
    if (sendReadCommand(canID, READ_ACCELERATION)) {
        if (readMotorResponse(canID, READ_ACCELERATION, rxData, 50)) {
            status.acceleration = (int32_t)(rxData[1]
                                          | (rxData[2] << 8)
                                          | (rxData[3] << 16)
                                          | (rxData[4] << 24));
        }
    }

    return true;
}

// CAN Read Task (Core 1)
void canReadTask(void* parameter) {
    MotorStatus motorData;
    TickType_t lastWakeTime = xTaskGetTickCount();

    for (;;) {
        // Read Motor 1
        if (readMotorAllData(MOTOR_ID_1, motorData)) {
            xQueueSend(motorDataQueue, &motorData, 0);
        }

        vTaskDelay(pdMS_TO_TICKS(10));

        // Read Motor 2
        if (readMotorAllData(MOTOR_ID_2, motorData)) {
            xQueueSend(motorDataQueue, &motorData, 0);
        }

        // Maintain update rate
        vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(UPDATE_INTERVAL_MS));
    }
}

// WiFi Output Task (Core 1)
void wifiOutputTask(void* parameter) {
    MotorStatus status;
    char buffer[256];

    for (;;) {
        // Check for client connection
        if (xSemaphoreTake(wifiMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            if (!client || !client.connected()) {
                if (client) client.stop();
                client = server.available();
                if (client) {
                    client_connected = true;
                    Serial.println("[WiFi] Client connected!");
                    client.println("ExoPulse Motor Monitor - Connected");
                } else {
                    client_connected = false;
                }
            }
            xSemaphoreGive(wifiMutex);
        }

        // Process motor data from queue
        if (xQueueReceive(motorDataQueue, &status, pdMS_TO_TICKS(10)) == pdTRUE) {
            // Check for angle overflow
            bool isOverflow = (status.motorAngle > 9007199254740992LL ||
                              status.motorAngle < -9007199254740992LL);

            float voltage = status.voltage * 0.1f;
            float current = (float)status.torqueCurrent * 33.0f / 2048.0f;

            // Format data
            if (isOverflow) {
                snprintf(buffer, sizeof(buffer),
                    "[%lu] M:%d T:%d V:%.1f I:%.2f S:%d ACC:%d E:%u A:ovf ERR:0x%02X\n",
                    status.timestamp,
                    status.motorID,
                    status.temperature,
                    voltage,
                    current,
                    status.speed,
                    status.acceleration,
                    status.encoder,
                    status.errorState
                );
            } else {
                float angleDeg = (float)status.motorAngle * 0.01f;
                snprintf(buffer, sizeof(buffer),
                    "[%lu] M:%d T:%d V:%.1f I:%.2f S:%d ACC:%d E:%u A:%.2f ERR:0x%02X\n",
                    status.timestamp,
                    status.motorID,
                    status.temperature,
                    voltage,
                    current,
                    status.speed,
                    status.acceleration,
                    status.encoder,
                    angleDeg,
                    status.errorState
                );
            }

            // Send to Serial (for debugging)
            Serial.print(buffer);

            // Send to WiFi client if connected
            if (xSemaphoreTake(wifiMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                if (client && client.connected()) {
                    client.print(buffer);
                }
                xSemaphoreGive(wifiMutex);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// Process WiFi commands
void processWiFiCommand(String cmd) {
    cmd.trim();
    cmd.toUpperCase();

    if (cmd == "CAL1" || cmd == "CAL_M1") {
        if (motor1_data_valid) {
            motor1_angle_offset = motor1_latest_angle;
            client.println("[CMD] Motor 1 calibrated!");
            Serial.println("[CMD] Motor 1 calibrated!");
        } else {
            client.println("[ERR] Motor 1 data not available");
        }
    }
    else if (cmd == "CAL2" || cmd == "CAL_M2") {
        if (motor2_data_valid) {
            motor2_angle_offset = motor2_latest_angle;
            client.println("[CMD] Motor 2 calibrated!");
            Serial.println("[CMD] Motor 2 calibrated!");
        } else {
            client.println("[ERR] Motor 2 data not available");
        }
    }
    else if (cmd == "CAL_BOTH") {
        if (motor1_data_valid && motor2_data_valid) {
            motor1_angle_offset = motor1_latest_angle;
            motor2_angle_offset = motor2_latest_angle;
            client.println("[CMD] Both motors calibrated!");
            Serial.println("[CMD] Both motors calibrated!");
        } else {
            client.println("[ERR] Motor data not available");
        }
    }
    else if (cmd == "CLEAR_CAL") {
        motor1_angle_offset = 0;
        motor2_angle_offset = 0;
        client.println("[CMD] Calibration cleared");
        Serial.println("[CMD] Calibration cleared");
    }
    else if (cmd == "STATUS") {
        char buf[200];
        snprintf(buf, sizeof(buf),
            "[STATUS] WiFi: %s | Client: %s | M1: %s | M2: %s\n",
            wifi_connected ? "Connected" : "Disconnected",
            client_connected ? "Connected" : "Disconnected",
            motor1_data_valid ? "OK" : "No Data",
            motor2_data_valid ? "OK" : "No Data"
        );
        client.print(buf);
        Serial.print(buf);
    }
    else if (cmd == "HELP") {
        client.println("Commands: CAL1, CAL2, CAL_BOTH, CLEAR_CAL, STATUS, HELP");
    }
    else {
        client.println("[ERR] Unknown command. Type HELP for commands.");
    }
}

// WiFi command handler task (Core 0)
void wifiCommandTask(void* parameter) {
    String receivedData = "";

    for (;;) {
        if (xSemaphoreTake(wifiMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            if (client && client.connected()) {
                while (client.available()) {
                    char c = client.read();
                    if (c == '\n' || c == '\r') {
                        if (receivedData.length() > 0) {
                            processWiFiCommand(receivedData);
                            receivedData = "";
                        }
                    } else {
                        receivedData += c;
                    }
                }
            }
            xSemaphoreGive(wifiMutex);
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

void setup() {
    Serial.begin(115200);
    delay(1000);

    Serial.println("\n========================================");
    Serial.println("   ExoPulse WiFi Motor Monitor");
    Serial.println("========================================\n");

    pinMode(LED_PIN, OUTPUT);

    // Initialize CAN Bus
    Serial.print("Initializing CAN Bus...");
    SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI, CAN_CS);

    bool canInitialized = false;
    if (CAN.begin(MCP_ANY, CAN_1000KBPS, MCP_8MHZ) == CAN_OK) {
        canInitialized = true;
    } else if (CAN.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) == CAN_OK) {
        canInitialized = true;
        Serial.println(" (Fallback to 500 KBPS)");
    }

    if (canInitialized) {
        CAN.setMode(MCP_NORMAL);
        Serial.println(" OK");
    } else {
        Serial.println(" FAILED");
        Serial.println("[ERROR] CAN initialization failed!");
        while (1) { delay(1000); }
    }

    // Initialize WiFi
    Serial.print("Connecting to WiFi: ");
    Serial.println(ssid);
    WiFi.begin(ssid, password);

    int wifi_attempts = 0;
    while (WiFi.status() != WL_CONNECTED && wifi_attempts < 20) {
        delay(500);
        Serial.print(".");
        wifi_attempts++;
    }

    if (WiFi.status() == WL_CONNECTED) {
        wifi_connected = true;
        Serial.println("\n✓ WiFi Connected!");
        Serial.print("IP Address: ");
        Serial.println(WiFi.localIP());
        Serial.print("Signal Strength (RSSI): ");
        Serial.print(WiFi.RSSI());
        Serial.println(" dBm");
    } else {
        Serial.println("\n✗ WiFi connection failed!");
        Serial.println("Continuing with Serial output only...");
    }

    // Start TCP Server
    if (wifi_connected) {
        server.begin();
        Serial.print("TCP Server started on port: ");
        Serial.println(serverPort);
        Serial.print("PC should connect to ");
        Serial.print(WiFi.localIP());
        Serial.print(":");
        Serial.println(serverPort);
        Serial.println();
    }

    // Create FreeRTOS objects
    motorDataQueue = xQueueCreate(20, sizeof(MotorStatus));
    canMutex = xSemaphoreCreateMutex();
    wifiMutex = xSemaphoreCreateMutex();

    // Create tasks
    xTaskCreatePinnedToCore(
        canReadTask,
        "CAN_Read",
        4096,
        NULL,
        2,
        &canReadTaskHandle,
        1  // Core 1
    );

    xTaskCreatePinnedToCore(
        wifiOutputTask,
        "WiFi_Output",
        4096,
        NULL,
        1,
        &wifiOutputTaskHandle,
        1  // Core 1
    );

    xTaskCreatePinnedToCore(
        wifiCommandTask,
        "WiFi_Command",
        4096,
        NULL,
        1,
        NULL,
        0  // Core 0
    );

    Serial.println("System ready!");
    Serial.println("Waiting for client connection...");
    Serial.println("========================================\n");
}

void loop() {
    // LED heartbeat
    static uint32_t last_led_toggle = 0;
    if (millis() - last_led_toggle > 500) {
        digitalWrite(LED_PIN, !digitalRead(LED_PIN));
        last_led_toggle = millis();
    }

    delay(100);
}
