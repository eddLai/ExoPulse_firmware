#pragma once
#include <Arduino.h>
#include <WiFi.h>

/*
 * WiFi Pairing Module
 * Handles WiFi STA connection and TCP server for motor data transmission
 */

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
 */
void sendWiFiStatus() {
    if (!client || !client.connected() || !wifiConnected) {
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

    // Send periodic WiFi status updates
    sendWiFiStatus();
}

} // namespace WiFiPairing
