#include <Arduino.h>
#include <SPI.h>
#include <mcp_can.h>

// MCP2515 configuration
constexpr int CAN_CS   = 5;
constexpr int SPI_SCK  = 18;
constexpr int SPI_MISO = 19;
constexpr int SPI_MOSI = 23;

MCP_CAN CAN(CAN_CS);

constexpr int LED_PIN = 2;

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("\n========================================");
  Serial.println("   CAN Bus Receiver Test (FIXED)");
  Serial.println("========================================\n");

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  // Initialize SPI with correct pin order
  Serial.println("[1] Initializing SPI...");
  SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI);
  Serial.println("    SPI initialized");

  // Initialize MCP2515
  Serial.println("\n[2] Initializing MCP2515...");
  Serial.println("    Trying 500KBPS @ 8MHz...");

  byte result = CAN.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ);

  if (result != CAN_OK) {
    Serial.println("    Failed! Trying 500KBPS @ 16MHz...");
    result = CAN.begin(MCP_ANY, CAN_500KBPS, MCP_16MHZ);
  }

  if (result != CAN_OK) {
    Serial.println("    Failed! Trying 1000KBPS @ 8MHz...");
    result = CAN.begin(MCP_ANY, CAN_1000KBPS, MCP_8MHZ);
  }

  if (result != CAN_OK) {
    Serial.println("    Failed! Trying 1000KBPS @ 16MHz...");
    result = CAN.begin(MCP_ANY, CAN_1000KBPS, MCP_16MHZ);
  }

  if (result != CAN_OK) {
    Serial.println("\n[ERROR] MCP2515 initialization FAILED!");
    Serial.println("Check:");
    Serial.println("  1. SPI wiring (CS, SCK, MISO, MOSI)");
    Serial.println("  2. MCP2515 power (VCC, GND)");
    Serial.println("  3. Crystal frequency (8MHz or 16MHz)");
    while (1) {
      digitalWrite(LED_PIN, !digitalRead(LED_PIN));
      delay(200);
    }
  }

  Serial.println("    MCP2515 initialized successfully!");

  // Set to NORMAL mode explicitly
  Serial.println("\n[3] Setting NORMAL mode...");
  CAN.setMode(MCP_NORMAL);
  Serial.println("    Mode set to NORMAL");

  Serial.println("\n[OK] CAN Bus Receiver ready!");
  Serial.println("========================================\n");
  Serial.println("Waiting for messages...\n");

  for (int i = 0; i < 3; i++) {
    digitalWrite(LED_PIN, HIGH);
    delay(100);
    digitalWrite(LED_PIN, LOW);
    delay(100);
  }
}

void loop() {
  static uint32_t rxCount = 0;
  static uint32_t lastRxTime = 0;
  static uint32_t lastCounter = 0;
  static uint32_t missedCount = 0;
  static uint32_t lastStatsTime = 0;

  // Check for received messages
  if (CAN.checkReceive() == CAN_MSGAVAIL) {
    unsigned long rxId;
    byte len;
    uint8_t rxData[8];

    // Read message
    CAN.readMsgBuf(&rxId, &len, rxData);

    digitalWrite(LED_PIN, HIGH);

    // Display received message
    Serial.print("[RX] #");
    Serial.print(rxCount);
    Serial.print(" ID=0x");
    Serial.print(rxId, HEX);
    Serial.print(" Len=");
    Serial.print(len);
    Serial.print(" Data: ");

    for (int i = 0; i < len; i++) {
      if (rxData[i] < 0x10) Serial.print("0");
      Serial.print(rxData[i], HEX);
      Serial.print(" ");
    }

    // Check if from sender (ID=0x100, marker 0xAA55)
    if (rxId == 0x100 && len == 8 && rxData[0] == 0xAA && rxData[1] == 0x55) {
      uint32_t counter = ((uint32_t)rxData[2] << 24) |
                         ((uint32_t)rxData[3] << 16) |
                         ((uint32_t)rxData[4] << 8) |
                         ((uint32_t)rxData[5]);

      Serial.print("| Counter=");
      Serial.print(counter);

      // Check for missed messages
      if (rxCount > 0) {
        uint32_t expected = lastCounter + 1;
        if (counter != expected) {
          uint32_t missed = counter - expected;
          missedCount += missed;
          Serial.print(" ⚠️ MISSED ");
          Serial.print(missed);
          Serial.print(" msg(s)");
        }
      }
      lastCounter = counter;
    }

    Serial.println();

    rxCount++;
    lastRxTime = millis();
    digitalWrite(LED_PIN, LOW);
  }

  // Display statistics every 5 seconds
  uint32_t now = millis();
  if (now - lastStatsTime >= 5000) {
    lastStatsTime = now;

    Serial.println("\n--- Statistics ---");
    Serial.print("Received: ");
    Serial.print(rxCount);
    Serial.print(" messages");

    if (missedCount > 0) {
      Serial.print(" | Missed: ");
      Serial.print(missedCount);
    }

    if (rxCount > 0) {
      uint32_t timeSinceLastRx = now - lastRxTime;
      Serial.print(" | Last RX: ");
      Serial.print(timeSinceLastRx / 1000.0, 1);
      Serial.print("s ago");
    }

    Serial.println();

    if (rxCount > 0 && (now - lastRxTime) > 3000) {
      Serial.println("⚠️  No messages received recently!");
      Serial.println("   Check sender is running");
    } else if (rxCount == 0 && now > 5000) {
      Serial.println("⚠️  No messages received yet!");
      Serial.println("   Check:");
      Serial.println("   - Sender is running");
      Serial.println("   - CAN_H/CAN_L wiring");
      Serial.println("   - 120Ω termination");
      Serial.println("   - Both use same baudrate");
    }

    Serial.println("------------------\n");
  }

  delay(10);
}
