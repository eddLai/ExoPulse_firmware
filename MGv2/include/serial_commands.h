#pragma once
#include "motor_protocol.h"
#include "calibration.h"
#include "config.h"
#include <Arduino.h>

/*
 * Serial Command Processing and Output Formatting
 * Handles all serial communication with the user
 */

// Print compact motor data for GUI parsing
void printMotorData(const MotorStatus& status, int64_t offset) {
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

    // Apply software calibration offset
    int64_t correctedAngle = status.motorAngle - offset;

    // Calculate angle in degrees
    float angleDeg = (float)correctedAngle * 0.01;

    // Check for overflow: valid range is roughly ±2^53 (float precision limit)
    if (correctedAngle > 9007199254740992LL || correctedAngle < -9007199254740992LL) {
        Serial.print("ovf");
    } else {
        Serial.print(angleDeg, 2);
    }
    Serial.print(" ERR:0x");
    Serial.print(status.errorState, HEX);
    Serial.println();
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
    Serial.print(status.speed);
    Serial.println(" dps");

    Serial.print("Acceleration:    ");
    Serial.print(status.acceleration);
    Serial.println(" dps/s");

    Serial.print("Encoder:         ");
    Serial.print(status.encoder);
    Serial.println(" (0~16383)");

    Serial.print("Multi-turn Angle:");
    // Apply calibration offset for detailed output
    int64_t detailedCorrectedAngle = status.motorAngle - offset;
    float detailedAngleDeg = (float)detailedCorrectedAngle * 0.01;

    // Check for overflow
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
    Serial.println("--- Software Calibration (Safe, No ROM Write) ---");
    Serial.println("CAL_M1 or CAL1        - Calibrate Motor 1 (set current position as zero)");
    Serial.println("CAL_M2 or CAL2        - Calibrate Motor 2 (set current position as zero)");
    Serial.println("CAL_ALL or CAL        - Calibrate both motors");
    Serial.println("CLEAR_CAL             - Clear all calibration offsets");
    Serial.println("");
    Serial.println("--- Hardware Zero (ROM Write, Permanent) ---");
    Serial.println("SET_ZERO_M1 or ZERO1  - Set Motor 1 zero to ROM (0x19, needs reboot)");
    Serial.println("SET_ZERO_M2 or ZERO2  - Set Motor 2 zero to ROM (0x19, needs reboot)");
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
        Serial.println("[CMD] Calibrating Motor 1 zero position (software offset)...");
        if (motor1_data_valid) {
            motor1_angle_offset = motor1_latest_angle;
            Serial.print("[OK] Motor 1 calibrated! Current angle set to zero (offset = ");
            Serial.print((float)motor1_angle_offset * 0.01, 2);
            Serial.println("°)");
            Serial.println("[INFO] This is a software offset - no ROM write, resets on reboot");
        } else {
            Serial.println("[ERROR] Motor 1 data not available yet, please wait...");
        }
    }
    else if (cmd == "CAL_M2" || cmd == "CALIBRATE_M2" || cmd == "CAL2") {
        Serial.println("[CMD] Calibrating Motor 2 zero position (software offset)...");
        if (motor2_data_valid) {
            motor2_angle_offset = motor2_latest_angle;
            Serial.print("[OK] Motor 2 calibrated! Current angle set to zero (offset = ");
            Serial.print((float)motor2_angle_offset * 0.01, 2);
            Serial.println("°)");
            Serial.println("[INFO] This is a software offset - no ROM write, resets on reboot");
        } else {
            Serial.println("[ERROR] Motor 2 data not available yet, please wait...");
        }
    }
    else if (cmd == "CAL_ALL" || cmd == "CALIBRATE" || cmd == "CAL") {
        Serial.println("[CMD] Calibrating ALL motors (software offset)...");
        Serial.println("[INFO] Note: CAL_ALL is best implemented in GUI");
        Serial.println("[INFO] Firmware provides: CAL1, CAL2 as atomic operations");

        // Simple implementation: calibrate both if data available
        bool m1_ok = false, m2_ok = false;

        if (motor1_data_valid) {
            motor1_angle_offset = motor1_latest_angle;
            Serial.print("[OK] Motor 1 calibrated (offset = ");
            Serial.print((float)motor1_angle_offset * 0.01, 2);
            Serial.println("°)");
            m1_ok = true;
        }

        if (motor2_data_valid) {
            motor2_angle_offset = motor2_latest_angle;
            Serial.print("[OK] Motor 2 calibrated (offset = ");
            Serial.print((float)motor2_angle_offset * 0.01, 2);
            Serial.println("°)");
            m2_ok = true;
        }

        if (m1_ok && m2_ok) {
            Serial.println("[OK] All motors calibrated!");
        } else {
            Serial.println("[WARNING] Some motors not calibrated (data not available)");
        }
        Serial.println("[INFO] Software offset - no ROM write, resets on reboot");
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
}
