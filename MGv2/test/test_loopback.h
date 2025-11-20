#include <Arduino.h>
#include <SPI.h>
#include <mcp_can.h>

// MCP2515 測試程式 - Loopback 模式
// 此模式不需要外部 CAN 設備，MCP2515 會自己接收自己發送的消息

constexpr int CAN_CS = 5;
constexpr int SPI_SCK = 18;
constexpr int SPI_MISO = 19;
constexpr int SPI_MOSI = 23;

MCP_CAN CAN(CAN_CS);

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n=== MCP2515 Loopback Test ===");

  SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI, CAN_CS);

  // 嘗試初始化 MCP2515
  Serial.println("Initializing MCP2515...");

  byte result = CAN.begin(MCP_STDEXT, CAN_1000KBPS, MCP_8MHZ);
  if (result != CAN_OK) {
    result = CAN.begin(MCP_STDEXT, CAN_1000KBPS, MCP_16MHZ);
  }

  if (result != CAN_OK) {
    Serial.println("[FAIL] MCP2515 init failed!");
    Serial.println("Check: 1) SPI wiring, 2) CS pin, 3) MCP2515 power");
    while (1) delay(1000);
  }

  Serial.println("[OK] MCP2515 initialized");

  // 設置為 Loopback 模式（自發自收）
  Serial.println("Setting LOOPBACK mode...");
  CAN.setMode(MCP_LOOPBACK);
  Serial.println("[OK] Loopback mode set");
  Serial.println("\nStarting self-test...\n");
}

void loop() {
  static uint32_t testCount = 0;
  static uint32_t successCount = 0;
  static uint32_t failCount = 0;

  // 發送測試消息
  uint8_t txData[8] = {0x9C, 0x00, 0x00, 0x00,
                       (uint8_t)(testCount & 0xFF),
                       (uint8_t)((testCount >> 8) & 0xFF),
                       0x00, 0x00};

  uint32_t txId = 0x141;

  byte rc = CAN.sendMsgBuf(txId, 0, 8, txData);

  if (rc == CAN_OK) {
    Serial.print("[TX] Sent frame #");
    Serial.print(testCount);
    Serial.print(" ID=0x");
    Serial.print(txId, HEX);

    // 嘗試接收（在 loopback 模式下應該能收到自己發的）
    delay(10);

    if (CAN.checkReceive() == CAN_MSGAVAIL) {
      unsigned long rxId;
      byte len;
      uint8_t rxData[8];

      CAN.readMsgBuf(&rxId, &len, rxData);

      Serial.print(" -> [RX] ID=0x");
      Serial.print(rxId, HEX);
      Serial.print(" len=");
      Serial.print(len);
      Serial.print(" data=");
      for (int i = 0; i < len; i++) {
        Serial.print(rxData[i], HEX);
        Serial.print(" ");
      }
      Serial.println(" ✓ PASS");
      successCount++;
    } else {
      Serial.println(" -> [RX] No response ✗ FAIL");
      failCount++;
    }
  } else {
    Serial.print("[TX] Send failed, rc=");
    Serial.println(rc);
    failCount++;
  }

  testCount++;

  if (testCount % 10 == 0) {
    Serial.print("\n--- Stats: Total=");
    Serial.print(testCount);
    Serial.print(", Success=");
    Serial.print(successCount);
    Serial.print(", Fail=");
    Serial.print(failCount);
    Serial.println(" ---\n");
  }

  delay(500);
}
