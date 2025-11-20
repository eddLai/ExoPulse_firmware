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
constexpr uint8_t READ_STATUS_2     = 0x9C;  // Working command for comparison

// Timing
unsigned long lastTestTime = 0;
constexpr unsigned long TEST_INTERVAL = 100;  // 100ms = 0.1 second

// Data storage for current cycle
struct MotorData {
    unsigned long timestamp;
    int8_t temperature;
    int16_t current;
    int16_t speed;
    uint16_t encoder;
    int32_t acceleration;
    bool dataValid;
} currentData;

void setup() {
    Serial.begin(115200);
    delay(1000);  // 減少延遲時間
    
    Serial.println("\n=======================================");
    Serial.println("   CAN Acceleration Debug Tool");
    Serial.println("   Testing Motor ID: " + String(MOTOR_ID));
    Serial.println("=======================================\n");
    
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);
    
    // 餵看門狗
    yield();
    
    // Initialize SPI
    Serial.println("[1] Initializing SPI...");
    SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI);
    delay(100);  // 給 SPI 一些時間初始化
    Serial.println("    SPI OK");
    
    yield();
    
    // Initialize MCP2515
    Serial.println("\n[2] Initializing MCP2515...");
    
    // 嘗試不同的初始化配置
    byte result = CAN.begin(MCP_ANY, CAN_1000KBPS, MCP_8MHZ);
    if (result != CAN_OK) {
        Serial.println("    8MHz failed, trying 16MHz...");
        delay(100);
        result = CAN.begin(MCP_ANY, CAN_1000KBPS, MCP_16MHZ);
    }
    
    if (result != CAN_OK) {
        Serial.println("    [WARNING] MCP2515 initialization failed!");
        Serial.println("    Continuing without CAN (for debugging)...");
        // 不要進入無限循環，繼續執行
    } else {
        CAN.setMode(MCP_NORMAL);
        Serial.println("    MCP2515 OK");
    }
    
    yield();
    
    Serial.println("\n[3] Ready to test!");
    Serial.println("Commands:");
    Serial.println("  A - Test READ_ACCELERATION (0x33)");
    Serial.println("  S - Test READ_STATUS_2 (0x9C)"); 
    Serial.println("  R - Read all pending messages");
    Serial.println("  H - Show this help");
    Serial.println("=======================================\n");
    
    Serial.println("System ready. Please enter a command...");
}

// Send CAN command and show what we send
bool sendCommand(uint8_t command) {
    uint8_t txData[8] = {command, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    
    Serial.print(">>> SENDING to CAN ID 0x");
    Serial.print(CAN_ID, HEX);
    Serial.print(": [");
    for (int i = 0; i < 8; i++) {
        if (i > 0) Serial.print(" ");
        if (txData[i] < 16) Serial.print("0");
        Serial.print(txData[i], HEX);
    }
    Serial.println("]");
    
    // 餵看門狗
    yield();
    
    byte result = CAN.sendMsgBuf(CAN_ID, 0, 8, txData);
    
    if (result == CAN_OK) {
        Serial.println("    Command sent successfully");
        digitalWrite(LED_PIN, HIGH);
        return true;
    } else {
        Serial.print("    [ERROR] Failed to send command, result code: ");
        Serial.println(result);
        return false;
    }
}

// Read and display all CAN responses
void readAllResponses(uint32_t timeoutMs = 200) {
    Serial.println("--- Listening for responses ---");
    
    uint32_t startTime = millis();
    int messageCount = 0;
    
    while (millis() - startTime < timeoutMs) {
        if (CAN.checkReceive() == CAN_MSGAVAIL) {
            unsigned long rxId;
            byte len;
            uint8_t rxData[8];
            
            CAN.readMsgBuf(&rxId, &len, rxData);
            messageCount++;
            
            Serial.print("<<< RECEIVED from CAN ID 0x");
            Serial.print(rxId, HEX);
            Serial.print(" (len=");
            Serial.print(len);
            Serial.print("): [");
            
            for (int i = 0; i < len; i++) {
                if (i > 0) Serial.print(" ");
                if (rxData[i] < 16) Serial.print("0");
                Serial.print(rxData[i], HEX);
            }
            Serial.print("]");
            
            // Decode if it matches our motor
            if (rxId == CAN_ID) {
                Serial.print("  ✓ FROM OUR MOTOR");
                Serial.print("  CMD=0x");
                Serial.print(rxData[0], HEX);
                
                if (rxData[0] == READ_ACCELERATION) {
                    // Parse acceleration data
                    int32_t accel = (int32_t)(rxData[4] | (rxData[5] << 8) | (rxData[6] << 16) | (rxData[7] << 24));
                    Serial.print("  ACCELERATION=");
                    Serial.print(accel);
                    Serial.print(" dps/s");
                } else if (rxData[0] == READ_STATUS_2) {
                    // Parse status data
                    int8_t temp = (int8_t)rxData[1];
                    int16_t current = (int16_t)(rxData[2] | (rxData[3] << 8));
                    int16_t speed = (int16_t)(rxData[4] | (rxData[5] << 8));
                    uint16_t encoder = (uint16_t)(rxData[6] | (rxData[7] << 8));
                    
                    Serial.print("  TEMP=");
                    Serial.print(temp);
                    Serial.print("°C CURRENT=");
                    Serial.print(current);
                    Serial.print(" SPEED=");
                    Serial.print(speed);
                    Serial.print(" ENCODER=");
                    Serial.print(encoder);
                }
            } else {
                Serial.print("  (from other device)");
            }
            Serial.println();
            
            digitalWrite(LED_PIN, LOW);
        }
        delay(1);
    }
    
    if (messageCount == 0) {
        Serial.println("    [TIMEOUT] No responses received");
    } else {
        Serial.print("    Total messages received: ");
        Serial.println(messageCount);
    }
    Serial.println("--- End of responses ---\n");
}

void loop() {
    // 餵看門狗
    yield();
    
    if (Serial.available() > 0) {
        char cmd = Serial.read();
        
        // Clear remaining characters
        while (Serial.available()) {
            Serial.read();
        }
        
        Serial.print("Received command: ");
        Serial.println(cmd);
        
        switch (cmd) {
            case 'A':
            case 'a':
                Serial.println("\n=== Testing READ_ACCELERATION (0x33) ===");
                if (sendCommand(READ_ACCELERATION)) {
                    delay(50);  // Small delay before reading
                    readAllResponses(300);  // Longer timeout for acceleration
                }
                break;
                
            case 'S':
            case 's':
                Serial.println("\n=== Testing READ_STATUS_2 (0x9C) ===");
                if (sendCommand(READ_STATUS_2)) {
                    delay(50);
                    readAllResponses(200);
                }
                break;
                
            case 'R':
            case 'r':
                Serial.println("\n=== Reading all pending messages ===");
                readAllResponses(500);
                break;
                
            case 'H':
            case 'h':
                Serial.println("\nCommands:");
                Serial.println("  A - Test READ_ACCELERATION (0x33)");
                Serial.println("  S - Test READ_STATUS_2 (0x9C)");
                Serial.println("  R - Read all pending messages");
                Serial.println("  H - Show this help");
                Serial.println();
                break;
                
            default:
                Serial.println("Unknown command. Press H for help.");
                break;
        }
    }
    
    // 餵看門狗並避免阻塞
    yield();
    delay(10);
}