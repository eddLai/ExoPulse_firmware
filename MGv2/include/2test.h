/**
 * ESP32 雙核測試 - FreeRTOS Dual Core Test
 * --------------------------------------------------------------
 * 測試目的：驗證 ESP32 雙核運行
 *
 * Core 0: WiFi STA 測試 (原本的 wifi_sta_test_main.cpp 代碼)
 * Core 1: 空閒測試 (只打印心跳，不做任何工作)
 * --------------------------------------------------------------
 * 使用 FreeRTOS 任務系統明確分配核心
 * --------------------------------------------------------------
 */

#include <Arduino.h>
#include <WiFi.h>

// ---------------- WiFi STA Parameters ----------------
const char* wifi_ssid = "ExoPulse";        // Mobile hotspot name
const char* wifi_password = "12345666";    // Mobile hotspot password

WiFiServer server(8888);                   // TCP Server port
WiFiClient client;

// ---------------- Statistics Variables ----------------
uint32_t packetsSent = 0;                  // Packets sent
uint32_t packetsReceived = 0;              // Packets received
uint32_t bytesReceived = 0;                // Bytes received
uint32_t bytesSent = 0;                    // Bytes sent
uint32_t lastStatTime = 0;                 // Last statistics time
uint32_t lastPacketTime = 0;               // Last packet received time
uint32_t connectionStartTime = 0;          // Connection start time

// ---------------- Test Mode ----------------
bool testMode = false;                     // Test mode active
uint32_t testStartTime = 0;                // Test start time
uint32_t testDataSent = 0;                 // Data sent during test
uint32_t testPacketLoss = 0;               // Packet loss count

// ---------------- Core 1 Statistics ----------------
uint32_t core1HeartbeatCount = 0;          // Core 1 心跳計數

// ---------------- Function Declarations ----------------
void printStatistics();
void sendTestData();
void handleClientCommand(String cmd);

// ═════════════════════════════════════════════════════════
//         Core 1 Task - 空閒測試 (只打印心跳)
// ═════════════════════════════════════════════════════════
void core1IdleTask(void *parameter) {
    Serial.println("[Core 1] Task started - Idle mode (只打印心跳)");

    while (true) {
        core1HeartbeatCount++;

        // 每 5 秒打印一次心跳
        if (core1HeartbeatCount % 50 == 0) {
            Serial.printf("[Core 1] Heartbeat #%lu - Running on Core %d - 目前不執行任何工作\n",
                          core1HeartbeatCount, xPortGetCoreID());
        }

        // 100ms 延遲
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

// ═════════════════════════════════════════════════════════
//         Core 0 Task - WiFi STA 測試 (原本的代碼)
// ═════════════════════════════════════════════════════════
void core0WiFiTask(void *parameter) {
    Serial.println("[Core 0] Task started - WiFi STA Test");
    Serial.printf("[Core 0] Running on Core %d\n", xPortGetCoreID());

    static String clientBuffer = "";
    static uint32_t lastPrint = 0;
    static uint32_t lastHeartbeat = 0;

    while (true) {
        // Check for new client connections
        if (!client || !client.connected()) {
            WiFiClient newClient = server.available();
            if (newClient) {
                client = newClient;
                Serial.println(F("\n>>> Client connected!"));
                Serial.print(F("IP: "));
                Serial.println(client.remoteIP());

                connectionStartTime = millis();
                packetsSent = 0;
                packetsReceived = 0;
                bytesReceived = 0;
                bytesSent = 0;
                clientBuffer = "";
                testMode = false;

                // Turn on LED to indicate connection
                digitalWrite(LED_BUILTIN, HIGH);

                // Send welcome message
                client.println("========================================");
                client.println("   WiFi Dual Core Test System");
                client.println("========================================");
                client.println("Available commands:");
                client.println("  START - Start continuous transmission test");
                client.println("  STOP  - Stop test");
                client.println("  STATS - Show statistics");
                client.println("  PING  - Test connection latency");
                client.println("========================================\n");

                lastStatTime = millis();
            }
        }

        // Handle client data
        if (client && client.connected()) {
            while (client.available()) {
                char c = client.read();
                clientBuffer += c;
                bytesReceived++;
                lastPacketTime = millis();

                if (c == '\n' || c == '\r') {
                    if (clientBuffer.length() > 1) {
                        packetsReceived++;
                        handleClientCommand(clientBuffer);
                    }
                    clientBuffer = "";
                }

                // Prevent buffer overflow
                if (clientBuffer.length() > 256) {
                    clientBuffer = "";
                }
            }

            // Test mode: continuously send data
            if (testMode) {
                sendTestData();
                vTaskDelay(5 / portTICK_PERIOD_MS);  // Control send rate
            }

            // Periodic heartbeat to keep connection alive
            if (millis() - lastHeartbeat >= 5000 && !testMode) {
                lastHeartbeat = millis();
                // Silent heartbeat
            }
        }

        // Display statistics periodically (every 10 seconds)
        if (millis() - lastPrint >= 10000) {
            lastPrint = millis();
            if (client && client.connected()) {
                printStatistics();
            } else {
                // Display WiFi status
                if (WiFi.status() == WL_CONNECTED) {
                    Serial.printf("[Core 0][WiFi] Connected to %s | IP: %s | Signal: %d dBm | Waiting for TCP connection...\n",
                                  wifi_ssid, WiFi.localIP().toString().c_str(), WiFi.RSSI());
                } else {
                    Serial.println("[Core 0][WiFi] Connection lost! Reconnecting...");
                    WiFi.reconnect();
                }
            }
        }

        // Detect client disconnection
        if (client && !client.connected()) {
            Serial.println(F("\n<<< Client disconnected"));
            testMode = false;
            digitalWrite(LED_BUILTIN, LOW);  // Turn off LED
            client.stop();
        }

        // 小延遲避免 watchdog
        vTaskDelay(1 / portTICK_PERIOD_MS);
    }
}

// ─────────── Print Statistics ───────────
void printStatistics() {
    uint32_t now = millis();
    uint32_t elapsed = (now - lastStatTime) / 1000;
    if (elapsed == 0) elapsed = 1; // Avoid division by zero

    // Calculate rates
    float rxRate = (bytesReceived * 8.0) / elapsed / 1000.0; // Kbps
    float txRate = (bytesSent * 8.0) / elapsed / 1000.0;     // Kbps

    Serial.println(F("\n========== Transmission Stats =========="));
    Serial.printf("Connection time: %lu sec\n", (now - connectionStartTime) / 1000);
    Serial.printf("Packets RX: %lu\n", packetsReceived);
    Serial.printf("Packets TX: %lu\n", packetsSent);
    Serial.printf("RX Rate: %.2f Kbps (%lu bytes)\n", rxRate, bytesReceived);
    Serial.printf("TX Rate: %.2f Kbps (%lu bytes)\n", txRate, bytesSent);

    if (testMode) {
        Serial.printf("Packet Loss: %lu\n", testPacketLoss);
    }

    if (lastPacketTime > 0) {
        uint32_t idleTime = (now - lastPacketTime) / 1000;
        Serial.printf("Last packet: %lu sec ago\n", idleTime);
    }

    // Show WiFi connection status and signal strength
    Serial.printf("WiFi Status: %s\n", WiFi.status() == WL_CONNECTED ? "Connected" : "Disconnected");
    if (WiFi.status() == WL_CONNECTED) {
        Serial.printf("Signal Strength: %d dBm\n", WiFi.RSSI());
    }

    // Show Core 1 heartbeat
    Serial.printf("Core 1 Heartbeats: %lu\n", core1HeartbeatCount);

    Serial.println(F("========================================\n"));

    // Reset statistics for next period
    bytesReceived = 0;
    bytesSent = 0;
    lastStatTime = now;
}

// ─────────── Send Test Data ───────────
void sendTestData() {
    if (!client || !client.connected()) return;

    // Generate test packet (1KB)
    char buffer[1024];
    uint32_t now = millis();

    // Add timestamp and sequence number to packet
    int headerLen = snprintf(buffer, sizeof(buffer),
             "[PKT:%lu][TIME:%lu][BYTES:%lu]",
             packetsSent, now, testDataSent);

    // Fill remaining space
    for (size_t i = headerLen; i < sizeof(buffer) - 2; i++) {
        buffer[i] = 'X';
    }
    buffer[sizeof(buffer) - 2] = '\n';
    buffer[sizeof(buffer) - 1] = '\0';

    // Send packet
    size_t written = client.write((uint8_t*)buffer, strlen(buffer));
    if (written > 0) {
        bytesSent += written;
        packetsSent++;
        testDataSent += written;
    } else {
        testPacketLoss++;
    }
}

// ─────────── Handle Client Commands ───────────
void handleClientCommand(String cmd) {
    cmd.trim();

    if (cmd == "START") {
        testMode = true;
        testStartTime = millis();
        testDataSent = 0;
        testPacketLoss = 0;
        Serial.println(F("[TEST] Starting continuous transmission test"));
        client.println("OK: Test started");

    } else if (cmd == "STOP") {
        testMode = false;
        uint32_t duration = (millis() - testStartTime) / 1000;
        if (duration == 0) duration = 1;
        float avgRate = (testDataSent * 8.0) / duration / 1000.0;

        Serial.println(F("[TEST] Test ended"));
        Serial.printf("Test duration: %lu sec\n", duration);
        Serial.printf("Data transferred: %lu bytes\n", testDataSent);
        Serial.printf("Average rate: %.2f Kbps\n", avgRate);
        Serial.printf("Packet loss: %lu\n", testPacketLoss);

        client.printf("OK: Test stopped (%.2f Kbps, Loss:%lu)\n", avgRate, testPacketLoss);

    } else if (cmd == "STATS") {
        printStatistics();
        client.println("OK: Statistics printed to serial");

    } else if (cmd == "PING") {
        client.printf("PONG: %lu ms\n", millis());

    } else {
        Serial.printf("Unknown command: %s\n", cmd.c_str());
        client.println("ERROR: Unknown command");
    }
}

// =================================================
//                         S E T U P
// =================================================
void setup() {
    Serial.begin(115200);
    delay(1000);

    Serial.println(F("\n========================================"));
    Serial.println(F("   ESP32 雙核測試系統"));
    Serial.println(F("   FreeRTOS Dual Core Test"));
    Serial.println(F("========================================"));
    Serial.println(F("Core 0: WiFi STA 測試"));
    Serial.println(F("Core 1: 空閒模式 (只打印心跳)"));
    Serial.println(F("========================================\n"));

    // LED indicator
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, LOW);

    // Set STA mode
    WiFi.mode(WIFI_STA);
    Serial.println(F("Connecting to mobile hotspot..."));
    Serial.print(F("SSID: "));
    Serial.println(wifi_ssid);

    // Connect to WiFi
    WiFi.begin(wifi_ssid, wifi_password);

    // Wait for connection (increased timeout to 40 seconds)
    int attempts = 0;
    Serial.print("Connecting");
    while (WiFi.status() != WL_CONNECTED && attempts < 80) {
        delay(500);
        Serial.print(".");
        attempts++;

        if (attempts % 10 == 0) {
            Serial.printf(" [%d/80] ", attempts);
        }
    }
    Serial.println();

    if (WiFi.status() == WL_CONNECTED) {
        Serial.println(F("\n✓ WiFi Connected!"));
        Serial.print(F("IP Address: "));
        Serial.println(WiFi.localIP());
        Serial.print(F("Signal Strength (RSSI): "));
        Serial.print(WiFi.RSSI());
        Serial.println(F(" dBm"));
    } else {
        Serial.println(F("\n✗ WiFi Connection Failed!"));
        Serial.println(F("Please check:"));
        Serial.println(F("  1. Mobile hotspot is enabled"));
        Serial.println(F("  2. SSID and password are correct"));
        Serial.println(F("Will keep trying to connect in background..."));
    }

    // Always start TCP Server (even if WiFi not connected yet)
    server.begin();
    Serial.printf("TCP Server started on port: 8888\n");
    if (WiFi.status() == WL_CONNECTED) {
        Serial.printf("PC should connect to %s:%d\n", WiFi.localIP().toString().c_str(), 8888);
    }

    Serial.println(F("\n========================================"));
    Serial.println(F("Creating FreeRTOS Tasks..."));
    Serial.println(F("========================================\n"));

    // Create Core 1 Task - 空閒測試 (只打印心跳)
    xTaskCreatePinnedToCore(
        core1IdleTask,         // Task function
        "Core1_Idle",          // Task name
        4096,                  // Stack size
        NULL,                  // Parameters
        1,                     // Priority
        NULL,                  // Task handle
        1                      // Core 1
    );
    Serial.println("✓ Core 1 Task created (Idle mode)");

    // Create Core 0 Task - WiFi STA 測試
    xTaskCreatePinnedToCore(
        core0WiFiTask,         // Task function
        "Core0_WiFi",          // Task name
        8192,                  // Stack size (larger for WiFi)
        NULL,                  // Parameters
        2,                     // Priority (higher than Core 1)
        NULL,                  // Task handle
        0                      // Core 0
    );
    Serial.println("✓ Core 0 Task created (WiFi STA Test)");

    Serial.println(F("\n========================================"));
    Serial.println(F("System Ready! Dual Core Running!"));
    Serial.println(F("========================================\n"));

    lastStatTime = millis();
}

// =================================================
//                       L O O P
// =================================================
void loop() {
    // loop() runs on Core 1 by default, but we've created dedicated tasks
    // Keep this empty to let FreeRTOS tasks handle everything

    // Just a heartbeat to show loop() is alive
    static uint32_t lastLoopPrint = 0;
    if (millis() - lastLoopPrint >= 30000) {
        lastLoopPrint = millis();
        Serial.printf("[loop()] Running on Core %d - FreeRTOS tasks handling work\n", xPortGetCoreID());
    }

    delay(1000);
}
