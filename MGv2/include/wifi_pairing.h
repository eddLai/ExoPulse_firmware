#pragma once
#include <Arduino.h>
#include <WiFi.h>
#include "output_mode.h"

/*
 * WiFi Pairing Module
 * Handles WiFi STA connection and TCP server for motor data transmission
 */

// Forward declaration of command processor (defined in serial_commands.h)
void processSerialCommand(const String& cmd);

namespace WiFiPairing {

// WiFi configuration
constexpr uint16_t TCP_PORT = 8888;
constexpr uint32_t WIFI_CONNECT_TIMEOUT_MS = 20000;  // 20 seconds

// Global objects
WiFiServer server(TCP_PORT);
WiFiClient client;

// WiFi credentials (configured via Serial command)
String wifi_ssid = "";
String wifi_password = "";
bool wifiInitialized = false;
bool wifiConnected = false;
uint32_t connectionStartTime = 0;

// Statistics
uint32_t packetsSent = 0;
uint32_t bytesSent = 0;
uint32_t lastRSSISendTime = 0;

/**
 * Connect to WiFi network
 * @param ssid WiFi network name
 * @param password WiFi password
 * @return true if connected successfully
 */
bool connectToWiFi(const String& ssid, const String& password) {
    wifi_ssid = ssid;
    wifi_password = password;

    Serial.println(F("[WiFi] Setting WiFi to STA mode..."));
    WiFi.mode(WIFI_STA);

    Serial.print(F("[WiFi] Connecting to: "));
    Serial.println(ssid);

    WiFi.begin(ssid.c_str(), password.c_str());

    // Wait for connection with timeout
    uint32_t startTime = millis();
    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && (millis() - startTime) < WIFI_CONNECT_TIMEOUT_MS) {
        delay(500);
        Serial.print(".");
        if (attempts % 10 == 0) Serial.flush();
        attempts++;
    }
    Serial.println();

    if (WiFi.status() == WL_CONNECTED) {
        wifiConnected = true;
        Serial.println(F("[WiFi] ✓ Connected!"));
        Serial.print(F("[WiFi] IP Address: "));
        Serial.println(WiFi.localIP());
        Serial.print(F("[WiFi] MAC Address: "));
        Serial.println(WiFi.macAddress());
        Serial.print(F("[WiFi] Signal Strength (RSSI): "));
        Serial.print(WiFi.RSSI());
        Serial.println(F(" dBm"));
        return true;
    } else {
        wifiConnected = false;
        Serial.println(F("[WiFi] ✗ Connection Failed!"));
        Serial.println(F("[WiFi] Please check:"));
        Serial.println(F("[WiFi]   1. SSID and password are correct"));
        Serial.println(F("[WiFi]   2. WiFi network is available"));
        Serial.println(F("[WiFi]   3. WiFi is on 2.4GHz band (ESP32 doesn't support 5GHz)"));
        return false;
    }
}

/**
 * Start TCP server
 * @return true if server started successfully
 */
bool startTCPServer() {
    if (!wifiConnected) {
        Serial.println(F("[WiFi] Cannot start TCP server: WiFi not connected"));
        return false;
    }

    server.begin();
    wifiInitialized = true;

    Serial.print(F("[WiFi] TCP Server started on port: "));
    Serial.println(TCP_PORT);
    Serial.print(F("[WiFi] PC should connect to: "));
    Serial.print(WiFi.localIP());
    Serial.print(":");
    Serial.println(TCP_PORT);

    return true;
}

/**
 * Disconnect from WiFi
 */
void disconnectWiFi() {
    if (client && client.connected()) {
        client.stop();
        Serial.println(F("[WiFi] Client disconnected"));
    }

    if (wifiInitialized) {
        server.end();
        Serial.println(F("[WiFi] TCP Server stopped"));
    }

    if (wifiConnected) {
        WiFi.disconnect();
        Serial.println(F("[WiFi] WiFi disconnected"));
    }

    wifiConnected = false;
    wifiInitialized = false;
}

/**
 * Check and handle new client connections
 * Called periodically from WiFi management task
 */
void handleClientConnection() {
    if (!wifiInitialized) return;

    // Check for new client
    if (!client || !client.connected()) {
        WiFiClient newClient = server.available();
        if (newClient) {
            client = newClient;
            connectionStartTime = millis();
            packetsSent = 0;
            bytesSent = 0;

            Serial.println(F("\n[WiFi] >>> Client connected!"));
            Serial.print(F("[WiFi] Client IP: "));
            Serial.println(client.remoteIP());

            // Send welcome message
            client.println("========================================");
            client.println("   LK-TECH Dual Motor Status Reader");
            client.println("   WiFi STA Mode - 10Hz");
            client.println("========================================");
            client.print("   SSID: ");
            client.println(wifi_ssid);
            client.print("   IP: ");
            client.println(WiFi.localIP());
            client.print("   RSSI: ");
            client.print(WiFi.RSSI());
            client.println(" dBm");
            client.println("========================================\n");
        }
    }

    // Check for client disconnection
    if (client && !client.connected()) {
        Serial.println(F("\n[WiFi] <<< Client disconnected"));
        client.stop();
    }
}

/**
 * Send data to WiFi client
 * @param data Data string to send
 * @return true if sent successfully
 */
bool sendToWiFi(const String& data) {
    if (!client || !client.connected()) {
        return false;
    }

    size_t written = client.print(data);
    if (written > 0) {
        bytesSent += written;
        packetsSent++;
        return true;
    }

    return false;
}

/**
 * Check if WiFi client is connected
 * @return true if client is connected
 */
bool isClientConnected() {
    return (client && client.connected());
}

/**
 * Get WiFi status information
 */
void printWiFiStatus() {
    Serial.println(F("\n========== WiFi Status =========="));

    if (wifiConnected) {
        Serial.println(F("[WiFi] Status: Connected"));
        Serial.print(F("[WiFi] SSID: "));
        Serial.println(wifi_ssid);
        Serial.print(F("[WiFi] IP Address: "));
        Serial.println(WiFi.localIP());
        Serial.print(F("[WiFi] MAC Address: "));
        Serial.println(WiFi.macAddress());
        Serial.print(F("[WiFi] Signal Strength (RSSI): "));
        Serial.print(WiFi.RSSI());
        Serial.println(F(" dBm"));

        if (wifiInitialized) {
            Serial.print(F("[WiFi] TCP Server: Running on port "));
            Serial.println(TCP_PORT);

            if (isClientConnected()) {
                Serial.print(F("[WiFi] Client: Connected ("));
                Serial.print(client.remoteIP());
                Serial.println(")");

                uint32_t uptime = (millis() - connectionStartTime) / 1000;
                Serial.print(F("[WiFi] Connection time: "));
                Serial.print(uptime);
                Serial.println(F(" seconds"));

                Serial.print(F("[WiFi] Packets sent: "));
                Serial.println(packetsSent);
                Serial.print(F("[WiFi] Bytes sent: "));
                Serial.println(bytesSent);
            } else {
                Serial.println(F("[WiFi] Client: Not connected"));
            }
        } else {
            Serial.println(F("[WiFi] TCP Server: Not started"));
        }
    } else {
        Serial.println(F("[WiFi] Status: Not connected"));
    }

    Serial.println(F("=================================\n"));
}

/**
 * Send periodic WiFi status updates to client
 * Called periodically from WiFi management task
 * NOTE: Only sends status when NOT in MODE_WIFI to avoid interfering with motor data
 */
void sendWiFiStatus() {
    if (!client || !client.connected() || !wifiConnected) {
        return;
    }

    // Don't send status updates when in WiFi output mode (motor data has priority)
    if (currentOutputMode == MODE_WIFI || currentOutputMode == MODE_BOTH) {
        return;
    }

    // Send RSSI update every 1 second (for connection quality monitoring)
    uint32_t now = millis();
    if (now - lastRSSISendTime >= 1000) {
        lastRSSISendTime = now;

        String statusMsg = "[WIFI_STATUS] SSID:";
        statusMsg += wifi_ssid;
        statusMsg += " RSSI:";
        statusMsg += WiFi.RSSI();
        statusMsg += "\n";

        client.print(statusMsg);
    }
}

/**
 * Handle incoming PING requests from PC and respond with PONG
 * Protocol:
 *   PC -> ESP32: [PING] seq:xxx ts:millis
 *   ESP32 -> PC: [PONG] seq:xxx ts_req:millis ts_reply:millis
 * Called periodically from WiFi management task
 */
void handlePingPong() {
    if (!client || !client.connected()) {
        return;
    }

    // Check for incoming data
    while (client.available() > 0) {
        String line = client.readStringUntil('\n');
        line.trim();

        // Check if it's a TEST request
        if (line.startsWith("[TEST]")) {
            // Parse TEST message: [TEST] seq:xxx
            int seqIdx = line.indexOf("seq:");

            if (seqIdx > 0) {
                // Extract sequence number
                String seqStr = line.substring(seqIdx + 4);
                uint32_t seq = seqStr.toInt();

                // Send TEST_REPLY response
                String testReplyMsg = "[TEST_REPLY] seq:";
                testReplyMsg += seq;
                testReplyMsg += " data:OK timestamp:";
                testReplyMsg += millis();
                testReplyMsg += " rssi:";
                testReplyMsg += WiFi.RSSI();
                testReplyMsg += "\n";

                client.print(testReplyMsg);
            }
        }
        // Check if it's a PING request
        else if (line.startsWith("[PING]")) {
            // Parse PING message: [PING] seq:xxx ts:millis
            int seqIdx = line.indexOf("seq:");
            int tsIdx = line.indexOf("ts:");

            if (seqIdx > 0 && tsIdx > 0) {
                // Extract sequence and timestamp
                String seqStr = line.substring(seqIdx + 4, line.indexOf(' ', seqIdx + 4));
                String tsStr = line.substring(tsIdx + 3);

                uint32_t seq = seqStr.toInt();
                uint32_t ts_req = tsStr.toInt();
                uint32_t ts_reply = millis();

                // Send PONG response
                String pongMsg = "[PONG] seq:";
                pongMsg += seq;
                pongMsg += " ts_req:";
                pongMsg += ts_req;
                pongMsg += " ts_reply:";
                pongMsg += ts_reply;
                pongMsg += "\n";

                client.print(pongMsg);
            }
        }
        // Handle other commands (MODE_WIFI, MODE_SERIAL, WIFI_STATUS, etc.)
        else if (line.length() > 0) {
            // Forward the command to serial command processor
            // This allows TCP clients to send commands just like via Serial
            processSerialCommand(line);
        }
    }
}

/**
 * Check WiFi connection and reconnect if needed
 * Called periodically from WiFi management task
 */
void checkConnection() {
    if (wifiConnected && WiFi.status() != WL_CONNECTED) {
        Serial.println(F("[WiFi] Connection lost! Attempting reconnect..."));
        wifiConnected = false;

        // Try to reconnect
        if (!wifi_ssid.isEmpty() && !wifi_password.isEmpty()) {
            connectToWiFi(wifi_ssid, wifi_password);
            if (wifiConnected) {
                startTCPServer();
            }
        }
    }

    // Handle PING/PONG for latency measurement
    handlePingPong();

    // Send periodic WiFi status updates
    sendWiFiStatus();
}

} // namespace WiFiPairing
