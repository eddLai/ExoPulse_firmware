#include <Arduino.h>
#include <WiFi.h>
#include <SPI.h>
#include <mcp_can.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

/*
 * ExoPulse - WiFi Motor Monitoring System
 *
 * Features:
 * - Dual motor CAN bus monitoring (Motor ID 1 & 2)
 * - WiFi TCP Server for real-time data streaming
 * - FreeRTOS multi-tasking architecture
 * - Remote calibration commands via WiFi
 * - Compatible with Python WiFi client
 *
 * WiFi Setup:
 * - SSID: ExoPulse
 * - Password: 12345666
 * - TCP Port: 8888
 */

// ========== WiFi Configuration ==========
const char* WIFI_SSID = "ExoPulse";
const char* WIFI_PASSWORD = "12345666";
const uint16_t TCP_PORT = 8888;

// ========== Hardware Configuration ==========
constexpr int CAN_CS   = 5;
constexpr int SPI_SCK  = 18;
constexpr int SPI_MISO = 19;
constexpr int SPI_MOSI = 23;
constexpr int LED_PIN  = 2;

constexpr uint8_t MOTOR_ID_1 = 1;
constexpr uint8_t MOTOR_ID_2 = 2;
constexpr uint32_t CAN_ID_1 = 0x140 + MOTOR_ID_1;
constexpr uint32_t CAN_ID_2 = 0x140 + MOTOR_ID_2;

constexpr uint32_t UPDATE_INTERVAL_MS = 100;  // 10Hz

// ========== CAN Commands ==========
enum MotorReadCommand : uint8_t {
    READ_ACCELERATION       = 0x33,
    READ_MULTI_ANGLE        = 0x92,
    READ_STATUS_1_ERROR     = 0x9A,
    READ_STATUS_2           = 0x9C,
};

// ========== Data Structures ==========
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

// ========== Global Objects ==========
MCP_CAN CAN(CAN_CS);
WiFiServer server(TCP_PORT);
WiFiClient client;

QueueHandle_t motorDataQueue;
SemaphoreHandle_t canMutex;
SemaphoreHandle_t wifiMutex;

// ========== Software Calibration ==========
int64_t motor1_angle_offset = 0;
int64_t motor2_angle_offset = 0;
volatile int64_t motor1_latest_angle = 0;
volatile int64_t motor2_latest_angle = 0;
volatile bool motor1_data_valid = false;
volatile bool motor2_data_valid = false;

// ========== WiFi Status ==========
volatile bool wifi_connected = false;
volatile bool client_connected = false;

// ========== CAN Communication Functions ==========
bool sendReadCommand(uint32_t canID, uint8_t cmd) {
    uint8_t txData[8] = {cmd, 0, 0, 0, 0, 0, 0, 0};

    if (xSemaphoreTake(canMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        byte rc = CAN.sendMsgBuf(canID, 0, 8, txData);
        xSemaphoreGive(canMutex);
        return (rc == CAN_OK);
    }
    return false;
}

bool readMotorResponse(uint32_t expectedID, uint8_t expectedCmd, uint8_t* rxData, uint32_t timeoutMs = 50) {
    uint32_t startTime = millis();

    while (millis() - startTime < timeoutMs) {
        if (xSemaphoreTake(canMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
            if (CAN.checkReceive() == CAN_MSGAVAIL) {
                unsigned long rxId;
                byte len;

                CAN.readMsgBuf(&rxId, &len, rxData);
                xSemaphoreGive(canMutex);

                if (rxId == expectedID && rxData[0] == expectedCmd) {
                    return true;
                }
            } else {
                xSemaphoreGive(canMutex);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    return false;
}

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

            // Apply software calibration
            int64_t offset = (motorID == 1) ? motor1_angle_offset : motor2_angle_offset;
            status.motorAngle = rawAngle - offset;

            // Update global angle
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
            status.acceleration = (int32_t)(rxData[1] | (rxData[2] << 8) |
                                           (rxData[3] << 16) | (rxData[4] << 24));
        }
    }

    return true;
}

// ========== FreeRTOS Tasks ==========
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

        // Process motor data
        if (xQueueReceive(motorDataQueue, &status, pdMS_TO_TICKS(10)) == pdTRUE) {
            // Check overflow
            bool isOverflow = (status.motorAngle > 9007199254740992LL ||
                              status.motorAngle < -9007199254740992LL);

            float voltage = status.voltage * 0.1f;
            float current = (float)status.torqueCurrent * 33.0f / 2048.0f;

            // Format data
            if (isOverflow) {
                snprintf(buffer, sizeof(buffer),
                    "[%lu] M:%d T:%d V:%.1f I:%.2f S:%d ACC:%d E:%u A:ovf ERR:0x%02X\n",
                    status.timestamp, status.motorID, status.temperature,
                    voltage, current, status.speed, status.acceleration,
                    status.encoder, status.errorState);
            } else {
                float angleDeg = (float)status.motorAngle * 0.01f;
                snprintf(buffer, sizeof(buffer),
                    "[%lu] M:%d T:%d V:%.1f I:%.2f S:%d ACC:%d E:%u A:%.2f ERR:0x%02X\n",
                    status.timestamp, status.motorID, status.temperature,
                    voltage, current, status.speed, status.acceleration,
                    status.encoder, angleDeg, status.errorState);
            }

            // Send to Serial
            Serial.print(buffer);

            // Send to WiFi client
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

void processWiFiCommand(String cmd) {
    cmd.trim();
    cmd.toUpperCase();

    if (cmd == "CAL1" || cmd == "CAL_M1") {
        if (motor1_data_valid) {
            motor1_angle_offset = motor1_latest_angle;
            client.println("[OK] Motor 1 calibrated");
            Serial.println("[CMD] Motor 1 calibrated");
        } else {
            client.println("[ERR] Motor 1 data not available");
        }
    }
    else if (cmd == "CAL2" || cmd == "CAL_M2") {
        if (motor2_data_valid) {
            motor2_angle_offset = motor2_latest_angle;
            client.println("[OK] Motor 2 calibrated");
            Serial.println("[CMD] Motor 2 calibrated");
        } else {
            client.println("[ERR] Motor 2 data not available");
        }
    }
    else if (cmd == "CAL_BOTH") {
        if (motor1_data_valid && motor2_data_valid) {
            motor1_angle_offset = motor1_latest_angle;
            motor2_angle_offset = motor2_latest_angle;
            client.println("[OK] Both motors calibrated");
            Serial.println("[CMD] Both motors calibrated");
        } else {
            client.println("[ERR] Motor data not available");
        }
    }
    else if (cmd == "CLEAR_CAL") {
        motor1_angle_offset = 0;
        motor2_angle_offset = 0;
        client.println("[OK] Calibration cleared");
        Serial.println("[CMD] Calibration cleared");
    }
    else if (cmd == "STATUS") {
        char buf[200];
        snprintf(buf, sizeof(buf),
            "[STATUS] WiFi:%s Client:%s M1:%s M2:%s\n",
            wifi_connected ? "OK" : "NO",
            client_connected ? "OK" : "NO",
            motor1_data_valid ? "OK" : "NO",
            motor2_data_valid ? "OK" : "NO");
        client.print(buf);
        Serial.print(buf);
    }
    else if (cmd == "HELP") {
        client.println("Commands: CAL1, CAL2, CAL_BOTH, CLEAR_CAL, STATUS, HELP");
    }
    else {
        client.println("[ERR] Unknown command. Type HELP");
    }
}

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

// ========== Setup ==========
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

    bool canOK = false;
    if (CAN.begin(MCP_ANY, CAN_1000KBPS, MCP_8MHZ) == CAN_OK) {
        canOK = true;
        Serial.println(" OK (1 Mbps)");
    } else if (CAN.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) == CAN_OK) {
        canOK = true;
        Serial.println(" OK (500 Kbps)");
    }

    if (canOK) {
        CAN.setMode(MCP_NORMAL);
    } else {
        Serial.println(" FAILED!");
        while (1) delay(1000);
    }

    // Initialize WiFi
    Serial.println("Starting WiFi initialization...");
    Serial.print("SSID: ");
    Serial.println(WIFI_SSID);
    Serial.print("Password: ");
    Serial.println(WIFI_PASSWORD);

    WiFi.mode(WIFI_STA);
    delay(100);
    Serial.println("WiFi mode set to STA");

    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    Serial.println("WiFi.begin() called");
    delay(100);

    int attempts = 0;
    Serial.print("Connecting");
    while (WiFi.status() != WL_CONNECTED && attempts < 30) {
        delay(500);
        Serial.print(".");
        Serial.flush();  // 強制刷新緩衝區
        attempts++;

        if (attempts % 10 == 0) {
            Serial.print(" [");
            Serial.print(attempts);
            Serial.print("] ");
        }
    }
    Serial.println();

    if (WiFi.status() == WL_CONNECTED) {
        wifi_connected = true;
        Serial.println("✓ WiFi Connected!");
        Serial.print("IP Address: ");
        Serial.println(WiFi.localIP());
        Serial.print("Signal (RSSI): ");
        Serial.print(WiFi.RSSI());
        Serial.println(" dBm");
    } else {
        Serial.println("✗ WiFi Failed!");
        Serial.print("Final WiFi status: ");
        Serial.println(WiFi.status());
        Serial.println("Continuing with Serial only...");
    }

    // Start TCP Server
    if (wifi_connected) {
        server.begin();
        Serial.print("TCP Server on port: ");
        Serial.println(TCP_PORT);
        Serial.print("Connect to: ");
        Serial.print(WiFi.localIP());
        Serial.print(":");
        Serial.println(TCP_PORT);
        Serial.println();
    }

    // Create FreeRTOS objects
    motorDataQueue = xQueueCreate(20, sizeof(MotorStatus));
    canMutex = xSemaphoreCreateMutex();
    wifiMutex = xSemaphoreCreateMutex();

    // Create tasks
    xTaskCreatePinnedToCore(canReadTask, "CAN_Read", 4096, NULL, 2, NULL, 1);
    xTaskCreatePinnedToCore(wifiOutputTask, "WiFi_Output", 4096, NULL, 1, NULL, 1);
    xTaskCreatePinnedToCore(wifiCommandTask, "WiFi_Command", 4096, NULL, 1, NULL, 0);

    Serial.println("System ready!");
    Serial.println("========================================\n");
}

void loop() {
    // LED heartbeat
    static uint32_t lastToggle = 0;
    if (millis() - lastToggle > 500) {
        digitalWrite(LED_PIN, !digitalRead(LED_PIN));
        lastToggle = millis();
    }
    delay(100);
}
