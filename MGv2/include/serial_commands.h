#pragma once
#include "motor_protocol.h"
#include "motor_operations.h"
#include "motor_control.h"
#include "calibration.h"
#include "config.h"
#include "output_mode.h"  // Shared output mode definitions
#include "wifi_pairing.h"
#include <Arduino.h>

/*
 * Serial Command Processing and Output Formatting
 * Handles all serial communication with the user
 * Supports multiple output modes: Serial, WiFi, or Both
 */

// External reference to packet sequence counter (defined in main.cpp)
extern uint32_t packetSequence;

// Format motor data as string
String formatMotorData(const MotorStatus& status, int64_t offset) {
    String data = "[";
    data += status.timestamp;
    data += "] SEQ:";
    data += packetSequence;
    data += " M:";
    data += status.motorID;
    data += " T:";
    data += status.temperature;
    data += " V:";
    data += String(status.voltage * 0.1, 1);
    data += " I:";
    float actualCurrent = (float)status.torqueCurrent * 33.0 / 2048.0;
    data += String(actualCurrent, 2);
    data += " S:";
    // Invert speed sign for Motor 2
    data += (status.motorID == 2) ? -status.speed : status.speed;
    data += " ACC:";
    data += status.acceleration;
    data += " E:";
    data += status.encoder;
    data += " A:";

    // Apply software calibration offset
    int64_t correctedAngle = status.motorAngle - offset;
    // Invert angle sign for Motor 2
    if (status.motorID == 2) {
        correctedAngle = -correctedAngle;
    }
    float angleDeg = (float)correctedAngle * 0.01;

    // Check for overflow (±90 trillion degrees, far beyond practical limits)
    if (correctedAngle > 9007199254740992LL || correctedAngle < -9007199254740992LL) {
        data += "ovf";
    } else {
        data += String(angleDeg, 2);
    }
    data += " ERR:0x";
    data += String(status.errorState, HEX);
    data += "\n";

    return data;
}

// Print compact motor data for GUI parsing (supports multiple output modes)
void printMotorData(const MotorStatus& status, int64_t offset) {
    String data = formatMotorData(status, offset);

    // Output based on current mode
    if (currentOutputMode == MODE_SERIAL || currentOutputMode == MODE_BOTH) {
        Serial.print(data);
    }

    if (currentOutputMode == MODE_WIFI || currentOutputMode == MODE_BOTH) {
        WiFiPairing::sendToWiFi(data);
    }

    // Increment packet sequence counter
    packetSequence++;
}

// Print detailed motor status
void printDetailedStatus(const MotorStatus& status, int64_t offset) {
    float actualCurrent = (float)status.torqueCurrent * 33.0 / 2048.0;

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
    // Invert speed sign for Motor 2
    Serial.print((status.motorID == 2) ? -status.speed : status.speed);
    Serial.println(" dps");

    Serial.print("Acceleration:    ");
    Serial.print(status.acceleration);
    Serial.println(" dps/s");

    Serial.print("Encoder:         ");
    Serial.print(status.encoder);
    Serial.println(" (0~262143)");

    Serial.print("Multi-turn Angle:");
    // Apply calibration offset for detailed output
    int64_t detailedCorrectedAngle = status.motorAngle - offset;
    // Invert angle sign for Motor 2
    if (status.motorID == 2) {
        detailedCorrectedAngle = -detailedCorrectedAngle;
    }
    float detailedAngleDeg = (float)detailedCorrectedAngle * 0.01;

    // Check for overflow (±90 trillion degrees, far beyond practical limits)
    if (detailedCorrectedAngle > 9007199254740992LL || detailedCorrectedAngle < -9007199254740992LL) {
        Serial.print("ovf");
        Serial.print(" °  (ovf turns) [RAW: 0x");
        Serial.print((uint32_t)(status.motorAngle >> 32), HEX);
        Serial.print((uint32_t)(status.motorAngle & 0xFFFFFFFF), HEX);
        Serial.println("]");
    } else {
        Serial.print(detailedAngleDeg, 2);
        Serial.print(" °  (");
        Serial.print(detailedAngleDeg / 360.0, 2);
        Serial.println(" turns)");
    }

    Serial.print("Error State:     0x");
    Serial.print(status.errorState, HEX);
    if (status.errorState & 0x01) Serial.print(" [LOW_VOLTAGE]");
    if (status.errorState & 0x08) Serial.print(" [OVER_TEMP]");
    Serial.println();
    Serial.println("--------------------\n");
}

// Print help message
void printHelp() {
    Serial.println("\n=== Available Commands ===");
    Serial.println("--- Multi-Turn Position Control (0xA4) ---");
    Serial.println("P1:<angle>            - Move Motor 1 to absolute angle (e.g., P1:30, P1:-45.5)");
    Serial.println("P2:<angle>            - Move Motor 2 to absolute angle (e.g., P2:90)");
    Serial.println("P1:<angle>:<speed>    - Move with speed limit (e.g., P1:30:500)");
    Serial.println("                        Supports ±359999.99°, default speed=700 dps");
    Serial.println("");
    Serial.println("--- Single-Turn Position Control (0xA6) ---");
    Serial.println("M1:<angle>            - Move Motor 1 to angle (e.g., M1:90, M1:-45)");
    Serial.println("M2:<angle>            - Move Motor 2 to angle (e.g., M2:180)");
    Serial.println("                        Range: 0~359.99°");
    Serial.println("");
    Serial.println("STOP                  - Stop all motors (0x80)");
    Serial.println("");
    Serial.println("--- Motor Torque Control (0xA1) ---");
    Serial.println("T1:<iq>               - Set Motor 1 torque (-800~800, e.g., T1:500)");
    Serial.println("T2:<iq>               - Set Motor 2 torque (-800~800, e.g., T2:-300)");
    Serial.println("");
    Serial.println("--- ROM Calibration (0x19, writes to motor ROM) ---");
    Serial.println("CAL_M1 or CAL1        - Calibrate Motor 1 (write zero to ROM)");
    Serial.println("CAL_M2 or CAL2        - Calibrate Motor 2 (write zero to ROM)");
    Serial.println("CAL_ALL or CAL        - Calibrate both motors (write zero to ROM)");
    Serial.println("CLEAR_CAL             - Clear software calibration offsets");
    Serial.println("");
    Serial.println("--- Hardware Zero (ROM Write, Permanent) ---");
    Serial.println("SET_ZERO_M1 or ZERO1  - Set Motor 1 zero to ROM (0x19, needs reboot)");
    Serial.println("SET_ZERO_M2 or ZERO2  - Set Motor 2 zero to ROM (0x19, needs reboot)");
    Serial.println("SET_OFFSET_M1:<val>   - Write encoder offset to Motor 1 ROM (0x91, 0~65535)");
    Serial.println("SET_OFFSET_M2:<val>   - Write encoder offset to Motor 2 ROM (0x91, 0~65535)");
    Serial.println("");
    Serial.println("--- WiFi Configuration ---");
    Serial.println("WIFI_CONFIG <SSID> <PASSWORD> - Connect to WiFi and start TCP server");
    Serial.println("WIFI_STATUS           - Show WiFi connection status");
    Serial.println("WIFI_DISCONNECT       - Disconnect from WiFi");
    Serial.println("");
    Serial.println("--- Output Mode Control ---");
    Serial.println("MODE_SERIAL           - Output motor data to Serial (default)");
    Serial.println("MODE_WIFI             - Output motor data to WiFi TCP client");
    Serial.println("MODE_BOTH             - Output motor data to both Serial and WiFi");
    Serial.println("");
    Serial.println("--- Read Commands ---");
    Serial.println("SINGLE1               - Read Motor 1 single-turn angle (0x94)");
    Serial.println("SINGLE2               - Read Motor 2 single-turn angle (0x94)");
    Serial.println("");
    Serial.println("--- Debug Commands ---");
    Serial.println("RESET_M1 or RESET1    - Reset Motor 1 angle (0x95, not implemented)");
    Serial.println("RESET_M2 or RESET2    - Reset Motor 2 angle (0x95, not implemented)");
    Serial.println("RESET_ALL or RESET    - Reset both motors (0x95, not implemented)");
    Serial.println("HELP                  - Show this help");
    Serial.println("========================\n");
}

// Process serial command
void processSerialCommand(const String& cmd) {
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
    else if (cmd.startsWith("SET_OFFSET_M1:")) {
        String valueStr = cmd.substring(14);
        uint16_t offset = (uint16_t)valueStr.toInt();
        Serial.print("[CMD] Writing encoder offset ");
        Serial.print(offset);
        Serial.println(" to Motor 1 ROM (0x91)...");
        Serial.println("[WARNING] This writes to motor ROM permanently!");
        if (writeEncoderOffsetToROM(CAN_ID_1, offset)) {
            Serial.println("[OK] Motor 1 encoder offset written to ROM!");
        } else {
            Serial.println("[ERROR] Motor 1 write encoder offset failed!");
        }
    }
    else if (cmd.startsWith("SET_OFFSET_M2:")) {
        String valueStr = cmd.substring(14);
        uint16_t offset = (uint16_t)valueStr.toInt();
        Serial.print("[CMD] Writing encoder offset ");
        Serial.print(offset);
        Serial.println(" to Motor 2 ROM (0x91)...");
        Serial.println("[WARNING] This writes to motor ROM permanently!");
        if (writeEncoderOffsetToROM(CAN_ID_2, offset)) {
            Serial.println("[OK] Motor 2 encoder offset written to ROM!");
        } else {
            Serial.println("[ERROR] Motor 2 write encoder offset failed!");
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
    else if (cmd == "CAL_M1" || cmd == "CALIBRATE_M1" || cmd == "CAL1") {
        Serial.println("[CMD] Calibrating Motor 1 zero position to ROM (0x19)...");
        Serial.println("[INFO] This writes current position as zero to motor ROM");
        if (setMotorZeroToROM(CAN_ID_1)) {
            // Clear software offset since ROM calibration is used
            motor1_angle_offset = 0;
            Serial.println("[OK] Motor 1 ROM calibration successful!");
            Serial.println("[INFO] Zero position saved to motor ROM");
        } else {
            Serial.println("[ERROR] Motor 1 ROM calibration failed!");
        }
    }
    else if (cmd == "CAL_M2" || cmd == "CALIBRATE_M2" || cmd == "CAL2") {
        Serial.println("[CMD] Calibrating Motor 2 zero position to ROM (0x19)...");
        Serial.println("[INFO] This writes current position as zero to motor ROM");
        if (setMotorZeroToROM(CAN_ID_2)) {
            // Clear software offset since ROM calibration is used
            motor2_angle_offset = 0;
            Serial.println("[OK] Motor 2 ROM calibration successful!");
            Serial.println("[INFO] Zero position saved to motor ROM");
        } else {
            Serial.println("[ERROR] Motor 2 ROM calibration failed!");
        }
    }
    else if (cmd == "CAL_ALL" || cmd == "CALIBRATE" || cmd == "CAL") {
        Serial.println("[CMD] Calibrating ALL motors to ROM (0x19)...");
        Serial.println("[INFO] This writes current positions as zero to motor ROM");

        bool m1_ok = false, m2_ok = false;

        Serial.println("\n--- Motor 1 ---");
        if (setMotorZeroToROM(CAN_ID_1)) {
            motor1_angle_offset = 0;
            Serial.println("[OK] Motor 1 ROM calibration successful!");
            m1_ok = true;
        } else {
            Serial.println("[ERROR] Motor 1 ROM calibration failed!");
        }

        vTaskDelay(pdMS_TO_TICKS(100));  // Small delay between motors

        Serial.println("\n--- Motor 2 ---");
        if (setMotorZeroToROM(CAN_ID_2)) {
            motor2_angle_offset = 0;
            Serial.println("[OK] Motor 2 ROM calibration successful!");
            m2_ok = true;
        } else {
            Serial.println("[ERROR] Motor 2 ROM calibration failed!");
        }

        Serial.println("\n--- Summary ---");
        if (m1_ok && m2_ok) {
            Serial.println("[OK] All motors ROM calibration successful!");
        } else if (m1_ok || m2_ok) {
            Serial.println("[WARNING] Partial calibration - some motors failed");
        } else {
            Serial.println("[ERROR] All motors ROM calibration failed!");
        }
        Serial.println("[INFO] Zero positions saved to motor ROM");
    }
    else if (cmd == "CLEAR_CAL" || cmd == "RESET_CAL") {
        Serial.println("[CMD] Clearing software calibration offsets...");
        motor1_angle_offset = 0;
        motor2_angle_offset = 0;
        Serial.println("[OK] All calibration offsets cleared");
    }
    else if (cmd == "HELP") {
        printHelp();
    }
    // Read Multi-Turn Angle Commands (0x92)
    else if (cmd == "READ_SINGLE_M1" || cmd == "SINGLE1") {
        Serial.println("[CMD] Reading Motor 1 multi-turn angle (0x92)...");
        MotorStatus status;
        if (readMotorAngle(CAN_ID_1, status)) {
            float angleDeg = (float)status.motorAngle * 0.01f;
            Serial.print("[OK] Motor 1 angle: ");
            Serial.print(angleDeg, 2);
            Serial.print("° (raw: ");
            Serial.print((int32_t)(status.motorAngle & 0xFFFFFFFF));  // Show lower 32 bits as signed
            Serial.println(")");
        } else {
            Serial.println("[ERROR] Failed to read Motor 1 angle");
        }
    }
    else if (cmd == "READ_SINGLE_M2" || cmd == "SINGLE2") {
        Serial.println("[CMD] Reading Motor 2 multi-turn angle (0x92)...");
        MotorStatus status;
        if (readMotorAngle(CAN_ID_2, status)) {
            float angleDeg = (float)status.motorAngle * 0.01f;
            Serial.print("[OK] Motor 2 angle: ");
            Serial.print(angleDeg, 2);
            Serial.print("° (raw: ");
            Serial.print((int32_t)(status.motorAngle & 0xFFFFFFFF));  // Show lower 32 bits as signed
            Serial.println(")");
        } else {
            Serial.println("[ERROR] Failed to read Motor 2 angle");
        }
    }
    // WiFi Configuration Commands
    else if (cmd.startsWith("WIFI_CONFIG ")) {
        // Parse SSID and PASSWORD from command
        String params = cmd.substring(12);  // Remove "WIFI_CONFIG "
        params.trim();

        int spaceIndex = params.indexOf(' ');
        if (spaceIndex > 0) {
            String ssid = params.substring(0, spaceIndex);
            String password = params.substring(spaceIndex + 1);
            password.trim();

            Serial.print("[CMD] Configuring WiFi: SSID=");
            Serial.println(ssid);

            if (WiFiPairing::connectToWiFi(ssid, password)) {
                if (WiFiPairing::startTCPServer()) {
                    Serial.println("[OK] WiFi configured successfully!");
                    Serial.println("[INFO] Use MODE_WIFI to switch output to WiFi");
                } else {
                    Serial.println("[ERROR] Failed to start TCP server");
                }
            } else {
                Serial.println("[ERROR] WiFi connection failed");
            }
        } else {
            Serial.println("[ERROR] Usage: WIFI_CONFIG <SSID> <PASSWORD>");
        }
    }
    else if (cmd == "WIFI_STATUS") {
        WiFiPairing::printWiFiStatus();
    }
    else if (cmd == "WIFI_DISCONNECT") {
        Serial.println("[CMD] Disconnecting WiFi...");
        WiFiPairing::disconnectWiFi();
        Serial.println("[OK] WiFi disconnected");

        // Auto-switch to Serial mode if currently in WiFi mode
        if (currentOutputMode == MODE_WIFI) {
            currentOutputMode = MODE_SERIAL;
            Serial.println("[INFO] Output mode switched to SERIAL");
        }
    }
    // Output Mode Control Commands
    else if (cmd == "MODE_SERIAL") {
        currentOutputMode = MODE_SERIAL;
        Serial.println("[OK] Output mode: SERIAL");
        Serial.println("[INFO] Motor data will output to Serial only");
    }
    else if (cmd == "MODE_WIFI") {
        if (WiFiPairing::isClientConnected()) {
            currentOutputMode = MODE_WIFI;
            Serial.println("[OK] Output mode: WIFI");
            Serial.println("[INFO] Motor data will output to WiFi client only");
            Serial.println("[INFO] Serial will still accept commands and show status");
        } else {
            Serial.println("[ERROR] Cannot switch to WiFi mode: No WiFi client connected");
            Serial.println("[INFO] Use WIFI_CONFIG to connect WiFi first");
        }
    }
    else if (cmd == "MODE_BOTH") {
        currentOutputMode = MODE_BOTH;
        Serial.println("[OK] Output mode: BOTH");
        Serial.println("[INFO] Motor data will output to both Serial and WiFi (debug mode)");
    }
    // ============================================================================
    // Motor Position Control Commands (0xA6)
    // ============================================================================
    else if (cmd == "STOP") {
        Serial.println("[CMD] Stopping all motors (0x80)...");
        bool m1_ok = sendMotorShutdown(CAN_ID_1);
        bool m2_ok = sendMotorShutdown(CAN_ID_2);
        if (m1_ok && m2_ok) {
            Serial.println("[OK] All motors stopped");
        } else {
            Serial.print("[WARN] Stop result: M1=");
            Serial.print(m1_ok ? "OK" : "FAIL");
            Serial.print(", M2=");
            Serial.println(m2_ok ? "OK" : "FAIL");
        }
    }
    else if (cmd.startsWith("M1:") || cmd.startsWith("M2:")) {
        // Parse motor position command: M1:90 or M2:-45
        uint8_t motorID = (cmd.charAt(1) == '1') ? MOTOR_ID_1 : MOTOR_ID_2;
        uint32_t canID = (motorID == MOTOR_ID_1) ? CAN_ID_1 : CAN_ID_2;

        String angleStr = cmd.substring(3);
        float angle = angleStr.toFloat();

        // Determine direction based on sign
        uint8_t direction = SPIN_CLOCKWISE;
        if (angle < 0) {
            direction = SPIN_COUNTER_CLOCKWISE;
            angle = -angle;  // Make positive
        }

        // Clamp angle to valid range (0 ~ 359.99)
        if (angle > 359.99) angle = 359.99;

        Serial.print("[CMD] Motor ");
        Serial.print(motorID);
        Serial.print(" -> ");
        Serial.print(angle, 2);
        Serial.print(" deg (");
        Serial.print(direction == SPIN_CLOCKWISE ? "CW" : "CCW");
        Serial.println(")");

        ControlResponse response;
        if (moveToAngle(canID, angle, direction, 100, &response)) {
            Serial.println("[OK] Position command sent");
            Serial.print("  Response: T=");
            Serial.print(response.temperature);
            Serial.print("C, I=");
            Serial.print(response.torqueCurrent);
            Serial.print(", S=");
            Serial.print(response.speed);
            Serial.print("dps, E=");
            Serial.println(response.encoder);
        } else {
            Serial.println("[ERROR] Position command failed");
        }
    }
    // Torque Control Commands (T1:xxx, T2:xxx)
    else if (cmd.startsWith("T1:") || cmd.startsWith("T2:")) {
        int motorID = cmd.charAt(1) - '0';  // 1 or 2
        int16_t iqValue = cmd.substring(3).toInt();
        uint32_t canID = (motorID == 1) ? CAN_ID_1 : CAN_ID_2;

        // Clamp iqValue to safety limit (±800)
        if (iqValue > 800) iqValue = 800;
        if (iqValue < -800) iqValue = -800;

        Serial.print("[CMD] Motor ");
        Serial.print(motorID);
        Serial.print(" torque -> iq=");
        Serial.println(iqValue);

        ControlResponse response;
        if (sendTorqueControl_A1(canID, iqValue, &response)) {
            Serial.println("[OK] Torque command sent");
            Serial.print("  Response: T=");
            Serial.print(response.temperature);
            Serial.print("C, I=");
            Serial.print(response.torqueCurrent);
            Serial.print(", S=");
            Serial.print(response.speed);
            Serial.print("dps, E=");
            Serial.println(response.encoder);
        } else {
            Serial.println("[ERROR] Torque command failed");
        }
    }
    // ============================================================================
    // Multi-Turn Position Control Commands (0xA4): P1:<angle> or P1:<angle>:<speed>
    // ============================================================================
    else if (cmd.startsWith("P1:") || cmd.startsWith("P2:")) {
        uint8_t motorID = (cmd.charAt(1) == '1') ? MOTOR_ID_1 : MOTOR_ID_2;
        uint32_t canID = (motorID == MOTOR_ID_1) ? CAN_ID_1 : CAN_ID_2;

        // Parse angle and optional speed: P1:30 or P1:30:500
        String params = cmd.substring(3);
        int colonIdx = params.indexOf(':');

        float angle = 0.0f;
        uint16_t maxSpeed = DEFAULT_MULTI_TURN_MAX_SPEED;

        if (colonIdx > 0) {
            // Format: P1:<angle>:<speed>
            angle = params.substring(0, colonIdx).toFloat();
            maxSpeed = (uint16_t)params.substring(colonIdx + 1).toInt();
        } else {
            // Format: P1:<angle>
            angle = params.toFloat();
        }

        Serial.print("[CMD] Motor ");
        Serial.print(motorID);
        Serial.print(" multi-turn -> ");
        Serial.print(angle, 2);
        Serial.print(" deg, max speed: ");
        Serial.print(maxSpeed);
        Serial.println(" dps");

        ControlResponse response;
        if (moveToMultiTurnAngle(canID, angle, maxSpeed, &response)) {
            Serial.println("[OK] Multi-turn position command sent");
            Serial.print("  Response: T=");
            Serial.print(response.temperature);
            Serial.print("C, I=");
            Serial.print(response.torqueCurrent);
            Serial.print(", S=");
            Serial.print(response.speed);
            Serial.print("dps, E=");
            Serial.println(response.encoder);
        } else {
            Serial.println("[ERROR] Multi-turn position command failed");
        }
    }
}
