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
  Serial.println("   CAN Bus Sender Test (FIXED)");
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

  Serial.println("\n[OK] CAN Bus Sender ready!");
  Serial.println("========================================\n");
  Serial.println("Sending messages every 500ms...\n");

  for (int i = 0; i < 3; i++) {
    digitalWrite(LED_PIN, HIGH);
    delay(100);
    digitalWrite(LED_PIN, LOW);
    delay(100);
  }
}

void loop() {
  static uint32_t counter = 0;
  static uint32_t successCount = 0;
  static uint32_t failCount = 0;

  // Prepare data
  uint8_t txData[8];
  txData[0] = 0xAA;
  txData[1] = 0x55;
  txData[2] = (counter >> 24) & 0xFF;
  txData[3] = (counter >> 16) & 0xFF;
  txData[4] = (counter >> 8) & 0xFF;
  txData[5] = counter & 0xFF;
  txData[6] = 0x00;
  txData[7] = 0x00;

  // Send CAN message with standard ID
  byte rc = CAN.sendMsgBuf(0x100, 0, 8, txData);

  digitalWrite(LED_PIN, HIGH);

  if (rc == CAN_OK) {
    successCount++;
    Serial.print("[TX] #");
    Serial.print(counter);
    Serial.print(" ID=0x100 Data: ");
    for (int i = 0; i < 8; i++) {
      if (txData[i] < 0x10) Serial.print("0");
      Serial.print(txData[i], HEX);
      Serial.print(" ");
    }
    Serial.println("✓ OK");
  } else {
    failCount++;
    Serial.print("[TX] #");
    Serial.print(counter);
    Serial.print(" FAILED! rc=");
    Serial.print(rc);

    byte txErr = CAN.errorCountTX();
    byte rxErr = CAN.errorCountRX();
    Serial.print(" TXErr=");
    Serial.print(txErr);
    Serial.print(" RXErr=");
    Serial.println(rxErr);

    if (txErr > 100) {
      Serial.println("  ⚠️  High error count! Check:");
      Serial.println("      - CAN_H/CAN_L wiring");
      Serial.println("      - 120Ω termination resistors");
      Serial.println("      - Receiver is running");
    }
  }

  digitalWrite(LED_PIN, LOW);

  if (counter > 0 && counter % 10 == 0) {
    float successRate = (float)successCount / (float)(successCount + failCount) * 100.0;
    Serial.println("\n--- Statistics ---");
    Serial.print("Total: ");
    Serial.print(counter);
    Serial.print(" | Success: ");
    Serial.print(successCount);
    Serial.print(" | Failed: ");
    Serial.print(failCount);
    Serial.print(" | Rate: ");
    Serial.print(successRate, 1);
    Serial.println("%");
    Serial.println("------------------\n");
  }

  counter++;
  delay(500);
}
