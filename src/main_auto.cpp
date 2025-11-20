#include <Arduino.h>
#include <SPI.h>
#include <mcp_can.h>

/*
 * Auto CAN Test - Continuous Data Collection
 * Purpose: Auto test READ_ACCELERATION and READ_STATUS_2 every 100ms
 * Output format optimized for Python plotting
 * Motor: LK-TECH M Series
 * CAN Baud Rate: 1 Mbps
 */

// MCP2515 SPI Configuration
constexpr int CAN_CS   = 5;
constexpr int SPI_SCK  = 18;
constexpr int SPI_MISO = 19;
constexpr int SPI_MOSI = 23;

MCP_CAN CAN(CAN_CS);

constexpr int LED_PIN = 2;
constexpr uint8_t MOTOR_ID = 1;  // Test Motor ID 1
const uint32_t CAN_ID = 0x140 + MOTOR_ID;

// Commands to test
constexpr uint8_t READ_ACCELERATION = 0x33;
constexpr uint8_t READ_STATUS_2     = 0x9C;

// Timing
unsigned long lastTestTime = 0;
constexpr unsigned long TEST_INTERVAL = 100;  // 100ms = 0.1 second
bool dataCollectionActive = true;

// Data storage for current cycle
struct MotorData {
    unsigned long timestamp;
    int8_t temperature;
    int16_t current;
    int16_t speed;
    uint16_t encoder;
    int32_t acceleration;
    bool statusValid;
    bool accelValid;
} currentData;

void setup() {
    Serial.begin(115200);
    delay(1000);
    
    Serial.println("\n=======================================");
    Serial.println("   Auto CAN Data Collection Tool");
    Serial.println("   Testing Motor ID: " + String(MOTOR_ID));
    Serial.println("   Interval: 100ms (10Hz)");
    Serial.println("=======================================\n");
    
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);
    
    // Initialize data structure
    currentData.statusValid = false;
    currentData.accelValid = false;
    
    yield();
    
    // Initialize SPI
    Serial.println("[1] Initializing SPI...");
    SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI);
    delay(100);
    Serial.println("    SPI OK");
    
    yield();
    
    // Initialize MCP2515
    Serial.println("\n[2] Initializing MCP2515...");
    
    byte result = CAN.begin(MCP_ANY, CAN_1000KBPS, MCP_8MHZ);
    if (result != CAN_OK) {
        Serial.println("    8MHz failed, trying 16MHz...");
        delay(100);
        result = CAN.begin(MCP_ANY, CAN_1000KBPS, MCP_16MHZ);
    }
    
    if (result != CAN_OK) {
        Serial.println("    [WARNING] MCP2515 initialization failed!");
        Serial.println("    Data will show as invalid...");
    } else {
        CAN.setMode(MCP_NORMAL);
        Serial.println("    MCP2515 OK");
    }
    
    yield();
    
    Serial.println("\n[3] Starting auto data collection...");
    Serial.println("Data format: TIMESTAMP,TEMP,CURRENT,SPEED,ENCODER,ACCELERATION,STATUS_OK,ACCEL_OK");
    Serial.println("Commands: H=Help, P=Pause/Resume, S=Stop");
    Serial.println("=======================================\n");
    
    delay(1000);
    Serial.println("DATA_START");  // Marker for Python script
}

// Send CAN command (optimized for speed)
bool sendCommand(uint8_t command) {
    uint8_t txData[8] = {command, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    byte result = CAN.sendMsgBuf(CAN_ID, 0, 8, txData);
    return (result == CAN_OK);
}

// Read CAN response (optimized with shorter timeout)
bool readResponse(uint8_t expectedCmd, uint8_t* rxData, uint32_t timeoutMs = 30) {
    uint32_t startTime = millis();
    
    while (millis() - startTime < timeoutMs) {
        if (CAN.checkReceive() == CAN_MSGAVAIL) {
            unsigned long rxId;
            byte len;
            
            CAN.readMsgBuf(&rxId, &len, rxData);
            
            if (rxId == CAN_ID && rxData[0] == expectedCmd) {
                return true;
            }
        }
        yield();  // 餵看門狗
    }
    return false;
}

// Test status command (0x9C)
bool testStatus() {
    if (!sendCommand(READ_STATUS_2)) {
        return false;
    }
    
    uint8_t rxData[8];
    if (!readResponse(READ_STATUS_2, rxData, 50)) {
        return false;
    }
    
    currentData.temperature = (int8_t)rxData[1];
    currentData.current = (int16_t)(rxData[2] | (rxData[3] << 8));
    currentData.speed = (int16_t)(rxData[4] | (rxData[5] << 8));
    currentData.encoder = (uint16_t)(rxData[6] | (rxData[7] << 8));
    
    return true;
}

// Test acceleration command (0x33)
bool testAcceleration() {
    if (!sendCommand(READ_ACCELERATION)) {
        return false;
    }
    
    uint8_t rxData[8];
    if (!readResponse(READ_ACCELERATION, rxData, 50)) {
        return false;
    }
    
    currentData.acceleration = (int32_t)(rxData[4] | (rxData[5] << 8) | (rxData[6] << 16) | (rxData[7] << 24));
    
    return true;
}

// Perform one complete test cycle
void performTestCycle() {
    currentData.timestamp = millis();
    
    // Test status command
    currentData.statusValid = testStatus();
    
    // Small delay between commands
    delay(10);
    
    // Test acceleration command
    currentData.accelValid = testAcceleration();
    
    // Output data in CSV format for easy parsing
    Serial.print(currentData.timestamp);
    Serial.print(",");
    Serial.print(currentData.temperature);
    Serial.print(",");
    Serial.print(currentData.current);
    Serial.print(",");
    Serial.print(currentData.speed);
    Serial.print(",");
    Serial.print(currentData.encoder);
    Serial.print(",");
    Serial.print(currentData.acceleration);
    Serial.print(",");
    Serial.print(currentData.statusValid ? 1 : 0);
    Serial.print(",");
    Serial.print(currentData.accelValid ? 1 : 0);
    Serial.println();
    
    // LED feedback
    digitalWrite(LED_PIN, (currentData.statusValid || currentData.accelValid) ? HIGH : LOW);
}

void loop() {
    yield();  // 餵看門狗
    
    // Handle serial commands
    if (Serial.available() > 0) {
        char cmd = Serial.read();
        
        // Clear buffer
        while (Serial.available()) {
            Serial.read();
        }
        
        switch (cmd) {
            case 'H':
            case 'h':
                Serial.println("\n=== Commands ===");
                Serial.println("H - Show this help");
                Serial.println("P - Pause/Resume data collection");
                Serial.println("S - Stop and show summary");
                Serial.println("================\n");
                break;
                
            case 'P':
            case 'p':
                dataCollectionActive = !dataCollectionActive;
                Serial.print("Data collection: ");
                Serial.println(dataCollectionActive ? "RESUMED" : "PAUSED");
                break;
                
            case 'S':
            case 's':
                Serial.println("DATA_END");
                Serial.println("Data collection stopped.");
                dataCollectionActive = false;
                break;
                
            default:
                Serial.println("Unknown command. Press H for help.");
                break;
        }
    }
    
    // Perform test cycle at regular intervals
    if (dataCollectionActive && (millis() - lastTestTime >= TEST_INTERVAL)) {
        performTestCycle();
        lastTestTime = millis();
    }
    
    yield();
}