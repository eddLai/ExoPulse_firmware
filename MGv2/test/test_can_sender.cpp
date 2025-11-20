#include <Arduino.h>
#include <SPI.h>
#include <mcp_can.h>

/*
 * CAN Bus Sender 測試程式
 * 用途：持續發送 CAN 訊息，測試硬體連接
 *
 * 接線：
 *   MCP2515 → ESP32
 *   CS   → GPIO 5
 *   SCK  → GPIO 18
 *   MISO → GPIO 19
 *   MOSI → GPIO 23
 *   VCC  → 3.3V/5V
 *   GND  → GND
 *
 *   MCP2515 → CAN Bus
 *   CANH → 另一塊 ESP32 的 CANH
 *   CANL → 另一塊 ESP32 的 CANL
 *
 * 注意：兩端都要有 120Ω 終端電阻！
 */

// MCP2515 配置
constexpr int CAN_CS   = 5;
constexpr int SPI_SCK  = 18;
constexpr int SPI_MISO = 19;
constexpr int SPI_MOSI = 23;

MCP_CAN CAN(CAN_CS);

// LED 指示（可選）
#if defined(CONFIG_IDF_TARGET_ESP32S3)
  constexpr int LED_PIN = 48;  // ESP32-S3
#else
  constexpr int LED_PIN = 2;   // ESP32
#endif

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("\n========================================");
  Serial.println("   CAN Bus Sender Test");
  Serial.println("========================================\n");

  // 初始化 LED
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  // 初始化 SPI
  Serial.println("[1] Initializing SPI...");
  SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI, CAN_CS);
  Serial.println("    SPI initialized");

  // 初始化 MCP2515
  Serial.println("\n[2] Initializing MCP2515...");
  Serial.println("    Trying 500KBPS @ 8MHz...");

  // 嘗試多種配置，直到成功（8MHz 晶振）
  byte result = CAN.begin(MCP_STDEXT, CAN_500KBPS, MCP_8MHZ);

  if (result != CAN_OK) {
    Serial.println("    Failed! Trying 500KBPS @ 16MHz...");
    result = CAN.begin(MCP_STDEXT, CAN_500KBPS, MCP_16MHZ);
  }

  if (result != CAN_OK) {
    Serial.println("    Failed! Trying 1000KBPS @ 8MHz...");
    result = CAN.begin(MCP_STDEXT, CAN_1000KBPS, MCP_8MHZ);
  }

  if (result != CAN_OK) {
    Serial.println("    Failed! Trying 1000KBPS @ 16MHz...");
    result = CAN.begin(MCP_STDEXT, CAN_1000KBPS, MCP_16MHZ);
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

  // 設置為 Normal 模式
  Serial.println("\n[3] Setting NORMAL mode...");
  CAN.setMode(MCP_NORMAL);
  Serial.println("    Mode set to NORMAL");

  Serial.println("\n[OK] CAN Bus Sender ready!");
  Serial.println("========================================\n");
  Serial.println("Sending messages every 500ms...\n");

  // LED 閃爍表示初始化成功
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
  static uint32_t lastPrint = 0;

  // 準備發送數據
  uint8_t txData[8];
  txData[0] = 0xAA;  // 標識字節
  txData[1] = 0x55;  // 標識字節
  txData[2] = (counter >> 24) & 0xFF;  // 計數器高字節
  txData[3] = (counter >> 16) & 0xFF;
  txData[4] = (counter >> 8) & 0xFF;
  txData[5] = counter & 0xFF;          // 計數器低字節
  txData[6] = 0x00;
  txData[7] = 0x00;

  // 發送 CAN 訊息
  // ID = 0x100 (標準幀)
  uint32_t txId = 0x100;
  byte rc = CAN.sendMsgBuf(txId, 0, 8, txData);

  // LED 閃爍表示發送
  digitalWrite(LED_PIN, HIGH);

  if (rc == CAN_OK) {
    successCount++;
    Serial.print("[TX] #");
    Serial.print(counter);
    Serial.print(" ID=0x");
    Serial.print(txId, HEX);
    Serial.print(" Data: ");
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

    // 顯示錯誤計數器
    byte txErr = CAN.errorCountTX();
    byte rxErr = CAN.errorCountRX();
    Serial.print(" TXErr=");
    Serial.print(txErr);
    Serial.print(" RXErr=");
    Serial.println(rxErr);

    // 如果錯誤計數器過高，給出提示
    if (txErr > 100) {
      Serial.println("  ⚠️  High error count! Check:");
      Serial.println("      - CAN_H/CAN_L wiring");
      Serial.println("      - 120Ω termination resistors");
      Serial.println("      - Receiver is running");
    }
  }

  digitalWrite(LED_PIN, LOW);

  // 每 10 次顯示統計
  if (counter > 0 && counter % 10 == 0) {
    uint32_t now = millis();
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
  delay(500);  // 每 500ms 發送一次
}
