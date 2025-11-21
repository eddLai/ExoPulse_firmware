#pragma once
#include "motor_protocol.h"
#include "motor_operations.h"
#include "config.h"

/*
 * Motor Safety Monitor
 * Uses 0x9A (READ_STATUS_1_ERROR) to check motor error state
 * Automatically sends 0x80 (MOTOR_SHUTDOWN) on error detection
 */

// Safety monitor state
struct SafetyState {
    bool motor1_error_active;
    bool motor2_error_active;
    bool motor1_shutdown_sent;
    bool motor2_shutdown_sent;
    uint8_t motor1_last_error;
    uint8_t motor2_last_error;
    uint32_t motor1_error_time;
    uint32_t motor2_error_time;
};

// Global safety state
static SafetyState safetyState = {false, false, false, false, 0, 0, 0, 0};

// Check if motor has error state (from 0x9A response)
// Returns true if any error flag is set
bool hasMotorError(uint8_t errorState) {
    return (errorState & ERROR_LOW_VOLTAGE) || (errorState & ERROR_OVER_TEMP);
}

// Get error description string
const char* getErrorDescription(uint8_t errorState) {
    if (errorState & ERROR_LOW_VOLTAGE && errorState & ERROR_OVER_TEMP) {
        return "LOW_VOLTAGE + OVER_TEMP";
    } else if (errorState & ERROR_LOW_VOLTAGE) {
        return "LOW_VOLTAGE";
    } else if (errorState & ERROR_OVER_TEMP) {
        return "OVER_TEMP";
    }
    return "NONE";
}

// Check motor safety and shutdown if error detected
// Call this after reading motor status with 0x9A
// Returns: true if motor was shut down due to error, false otherwise
bool checkAndHandleMotorError(uint8_t motorID, uint32_t canID, const MotorStatus& status) {
    bool isMotor1 = (motorID == MOTOR_ID_1);
    bool* error_active = isMotor1 ? &safetyState.motor1_error_active : &safetyState.motor2_error_active;
    bool* shutdown_sent = isMotor1 ? &safetyState.motor1_shutdown_sent : &safetyState.motor2_shutdown_sent;
    uint8_t* last_error = isMotor1 ? &safetyState.motor1_last_error : &safetyState.motor2_last_error;
    uint32_t* error_time = isMotor1 ? &safetyState.motor1_error_time : &safetyState.motor2_error_time;

    // Check for error state
    if (hasMotorError(status.errorState)) {
        *error_active = true;
        *last_error = status.errorState;
        *error_time = millis();

        // Send shutdown command if not already sent
        if (!*shutdown_sent) {
            Serial.print("[SAFETY] Motor ");
            Serial.print(motorID);
            Serial.print(" ERROR detected: ");
            Serial.println(getErrorDescription(status.errorState));
            Serial.print("[SAFETY] Sending SHUTDOWN (0x80) to Motor ");
            Serial.println(motorID);

            if (sendMotorShutdown(canID)) {
                Serial.print("[SAFETY] Motor ");
                Serial.print(motorID);
                Serial.println(" shutdown SUCCESS");
                *shutdown_sent = true;
            } else {
                Serial.print("[SAFETY] Motor ");
                Serial.print(motorID);
                Serial.println(" shutdown FAILED - will retry");
            }
            return true;
        }
    } else {
        // Error cleared - reset state
        if (*error_active) {
            Serial.print("[SAFETY] Motor ");
            Serial.print(motorID);
            Serial.println(" error cleared");
        }
        *error_active = false;
        *shutdown_sent = false;
        *last_error = 0;
    }

    return false;
}

// Force shutdown both motors (emergency stop)
void emergencyShutdownAll() {
    Serial.println("[SAFETY] EMERGENCY SHUTDOWN - All motors");

    if (sendMotorShutdown(CAN_ID_1)) {
        Serial.println("[SAFETY] Motor 1 emergency shutdown SUCCESS");
        safetyState.motor1_shutdown_sent = true;
    } else {
        Serial.println("[SAFETY] Motor 1 emergency shutdown FAILED");
    }

    if (sendMotorShutdown(CAN_ID_2)) {
        Serial.println("[SAFETY] Motor 2 emergency shutdown SUCCESS");
        safetyState.motor2_shutdown_sent = true;
    } else {
        Serial.println("[SAFETY] Motor 2 emergency shutdown FAILED");
    }
}

// Reset safety state (call after manually clearing errors)
void resetSafetyState() {
    safetyState.motor1_error_active = false;
    safetyState.motor2_error_active = false;
    safetyState.motor1_shutdown_sent = false;
    safetyState.motor2_shutdown_sent = false;
    safetyState.motor1_last_error = 0;
    safetyState.motor2_last_error = 0;
    Serial.println("[SAFETY] Safety state reset");
}

// Get current safety status
void printSafetyStatus() {
    Serial.println("\n--- Safety Monitor Status ---");
    Serial.print("Motor 1: ");
    if (safetyState.motor1_error_active) {
        Serial.print("ERROR (");
        Serial.print(getErrorDescription(safetyState.motor1_last_error));
        Serial.print(") Shutdown: ");
        Serial.println(safetyState.motor1_shutdown_sent ? "SENT" : "PENDING");
    } else {
        Serial.println("OK");
    }

    Serial.print("Motor 2: ");
    if (safetyState.motor2_error_active) {
        Serial.print("ERROR (");
        Serial.print(getErrorDescription(safetyState.motor2_last_error));
        Serial.print(") Shutdown: ");
        Serial.println(safetyState.motor2_shutdown_sent ? "SENT" : "PENDING");
    } else {
        Serial.println("OK");
    }
    Serial.println("-----------------------------\n");
}
