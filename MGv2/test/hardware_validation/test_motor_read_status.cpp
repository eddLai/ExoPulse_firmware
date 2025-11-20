#include <Arduino.h>
#include <SPI.h>
#include <mcp_can.h>

/*
 * LK-TECH Motor Status Reader Test
 * Purpose: READ motor status data without controlling the motor
 *
 * This program demonstrates how to read various motor parameters
 * without sending any control commands.
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

    // Single-turn angle
    uint32_t circleAngle;    // 0.01°/LSB (0~36000*ratio-1)
};

MotorStatus motorStatus;

void setup() {
    Serial.begin(115200);
    delay(1000);

    Serial.println("\n========================================");
    Serial.println("   LK-TECH Motor Status Reader");
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

    Serial.println("\n[OK] Motor Status Reader ready!");
    Serial.println("========================================\n");
    Serial.print("Monitoring Motor ID=");
    Serial.println(MOTOR_ID);
    Serial.println("Reading motor status every 1 second...\n");

    // Blink LED to indicate ready
    for (int i = 0; i < 3; i++) {
        digitalWrite(LED_PIN, HIGH);
        delay(100);
        digitalWrite(LED_PIN, LOW);
        delay(100);
    }
}

// Send a read command to motor
bool sendReadCommand(uint8_t cmd) {
    uint8_t txData[8] = {cmd, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

    byte rc = CAN.sendMsgBuf(CAN_ID, 0, 8, txData);

    if (rc == CAN_OK) {
        return true;
    } else {
        Serial.print("[ERROR] Send failed: rc=");
        Serial.println(rc);
        return false;
    }
}

// Read CAN response from motor
bool readMotorResponse(uint8_t expectedCmd, uint8_t* rxData, uint32_t timeoutMs = 100) {
    unsigned long startTime = millis();

    while (millis() - startTime < timeoutMs) {
        if (CAN.checkReceive() == CAN_MSGAVAIL) {
            unsigned long rxId;
            byte len;

            CAN.readMsgBuf(&rxId, &len, rxData);

            // Check if response is from our motor
            if (rxId == CAN_ID && rxData[0] == expectedCmd) {
                return true;
            }
        }
        delay(1);
    }

    return false;  // Timeout
}

// Read motor status 2 (temperature, torque current, speed, encoder)
bool readMotorStatus2() {
    if (!sendReadCommand(READ_STATUS_2)) {
        return false;
    }

    uint8_t rxData[8];
    if (!readMotorResponse(READ_STATUS_2, rxData, 100)) {
        Serial.println("[WARN] No response from motor (Status 2)");
        return false;
    }

    // Parse response
    motorStatus.temperature = (int8_t)rxData[1];
    motorStatus.torqueCurrent = (int16_t)(rxData[2] | (rxData[3] << 8));
    motorStatus.speed = (int16_t)(rxData[4] | (rxData[5] << 8));
    motorStatus.encoder = (uint16_t)(rxData[6] | (rxData[7] << 8));

    return true;
}

// Read motor status 1 (temperature, voltage, error flags)
bool readMotorStatus1() {
    if (!sendReadCommand(READ_STATUS_1_ERROR)) {
        return false;
    }

    uint8_t rxData[8];
    if (!readMotorResponse(READ_STATUS_1_ERROR, rxData, 100)) {
        Serial.println("[WARN] No response from motor (Status 1)");
        return false;
    }

    // Parse response
    motorStatus.temperature = (int8_t)rxData[1];
    motorStatus.voltage = (uint16_t)(rxData[3] | (rxData[4] << 8));
    motorStatus.errorState = rxData[7];

    return true;
}

// Read multi-turn angle
bool readMultiTurnAngle() {
    if (!sendReadCommand(READ_MULTI_ANGLE)) {
        return false;
    }

    uint8_t rxData[8];
    if (!readMotorResponse(READ_MULTI_ANGLE, rxData, 100)) {
        Serial.println("[WARN] No response from motor (Multi-turn angle)");
        return false;
    }

    // Parse response (int64_t, 7 bytes used)
    motorStatus.motorAngle = (int64_t)rxData[1]
                           | ((int64_t)rxData[2] << 8)
                           | ((int64_t)rxData[3] << 16)
                           | ((int64_t)rxData[4] << 24)
                           | ((int64_t)rxData[5] << 32)
                           | ((int64_t)rxData[6] << 40);

    return true;
}

// Print motor status
void printMotorStatus() {
    Serial.println("--- Motor Status ---");

    Serial.print("Temperature:     ");
    Serial.print(motorStatus.temperature);
    Serial.println(" °C");

    Serial.print("Voltage:         ");
    Serial.print(motorStatus.voltage * 0.1, 1);
    Serial.println(" V");

    Serial.print("Torque Current:  ");
    float actualCurrent = (float)motorStatus.torqueCurrent * 33.0 / 2048.0;
    Serial.print(actualCurrent, 2);
    Serial.print(" A  (raw=");
    Serial.print(motorStatus.torqueCurrent);
    Serial.println(")");

    Serial.print("Speed:           ");
    Serial.print(motorStatus.speed);
    Serial.println(" dps");

    Serial.print("Encoder:         ");
    Serial.print(motorStatus.encoder);
    Serial.println(" (0~16383)");

    Serial.print("Multi-turn Angle:");
    float angleDeg = (float)motorStatus.motorAngle * 0.01;
    Serial.print(angleDeg, 2);
    Serial.print(" °  (");
    Serial.print(angleDeg / 360.0, 2);
    Serial.println(" turns)");

    // Error state
    Serial.print("Error State:     0x");
    Serial.print(motorStatus.errorState, HEX);
    if (motorStatus.errorState & 0x01) Serial.print(" [LOW_VOLTAGE]");
    if (motorStatus.errorState & 0x08) Serial.print(" [OVER_TEMP]");
    Serial.println();

    Serial.println("--------------------\n");
}

void loop() {
    static uint32_t lastReadTime = 0;
    uint32_t now = millis();

    // Read motor status every 1 second
    if (now - lastReadTime >= 1000) {
        lastReadTime = now;

        digitalWrite(LED_PIN, HIGH);

        Serial.print("[");
        Serial.print(now / 1000);
        Serial.println("s] Reading motor status...");

        bool success = true;

        // Read Status 1 (voltage, temperature, error)
        if (!readMotorStatus1()) {
            success = false;
        }

        delay(10);

        // Read Status 2 (torque, speed, encoder)
        if (!readMotorStatus2()) {
            success = false;
        }

        delay(10);

        // Read multi-turn angle
        if (!readMultiTurnAngle()) {
            success = false;
        }

        if (success) {
            printMotorStatus();
        } else {
            Serial.println("[ERROR] Failed to read motor status");
            Serial.println("Check:");
            Serial.println("  1. Motor is powered ON");
            Serial.println("  2. CAN bus wiring (CAN_H, CAN_L, GND)");
            Serial.println("  3. 120Ω termination resistors on both ends");
            Serial.println("  4. Motor ID is correct (currently ID=1)");
            Serial.println();
        }

        digitalWrite(LED_PIN, LOW);
    }

    delay(10);
}
