#include <Arduino.h>
#include <SPI.h>
#include <mcp_can.h>

/*
 * CAN Bus Receiver 測試程式
 * 用途：接收並顯示 CAN 訊息，測試硬體連接
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
 *   CANH → Sender ESP32 的 CANH
 *   CANL → Sender ESP32 的 CANL
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
  Serial.println("   CAN Bus Receiver Test");
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

  // 嘗試多種配置，直到成功（8MHz 晶振，與 Sender 相同）
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

  Serial.println("\n[OK] CAN Bus Receiver ready!");
  Serial.println("========================================\n");
  Serial.println("Waiting for messages...\n");

  // LED 閃爍表示初始化成功
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

  // 檢查是否有接收到訊息
  if (CAN.checkReceive() == CAN_MSGAVAIL) {
    unsigned long rxId;
    byte len;
    uint8_t rxData[8];

    // 讀取訊息
    CAN.readMsgBuf(&rxId, &len, rxData);

    // LED 閃爍表示接收
    digitalWrite(LED_PIN, HIGH);

    // 顯示接收到的訊息
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

    // 如果是來自 Sender 的訊息（ID=0x100，且前兩字節是 0xAA55）
    if (rxId == 0x100 && len == 8 && rxData[0] == 0xAA && rxData[1] == 0x55) {
      // 解析計數器
      uint32_t counter = ((uint32_t)rxData[2] << 24) |
                         ((uint32_t)rxData[3] << 16) |
                         ((uint32_t)rxData[4] << 8) |
                         ((uint32_t)rxData[5]);

      Serial.print("| Counter=");
      Serial.print(counter);

      // 檢查是否有遺失的訊息
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

  // 每 5 秒顯示統計
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

    // 如果超過 3 秒沒收到訊息，顯示提示
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

  delay(10);  // 小延遲，避免 CPU 滿載
}
