
#include <Arduino.h>
#include "ExoBus.h"
#include <SPI.h>
#include "SerialConsole.h"

// 固定 CAN(MCP2515) 使用之 SPI 腳位 (ESP32 VSPI)
static constexpr int CAN_SCK  = 18; // iSCK
static constexpr int CAN_MISO = 19; // MISO(SO)
static constexpr int CAN_MOSI = 23; // MOSI(SI)
static constexpr int CAN_CS   = 5;  // SS / CS

static ExoBus exo;

// 軟體看門狗（逾時自動 STOP）
class SoftWatchdog {
public:
  void begin(uint32_t timeoutMs) { timeout_ = timeoutMs; feed(); tripped_ = false; }
  void feed() { lastFeed_ = millis(); }
  void poll() {
    if (!tripped_ && (millis() - lastFeed_ > timeout_)) {
      tripped_ = true;
      Serial.println(F("! WDT timeout -> STOP"));
      exo.stop();
    }
  }
private:
  uint32_t timeout_ = 200;
  uint32_t lastFeed_ = 0;
  bool tripped_ = false;
};

static SoftWatchdog wdt;
static void feedWdt() { wdt.feed(); }

// 使用 SerialConsole 處理序列指令
static SerialConsole console(exo);

void setup() {
  Serial.begin(115200);
  while (!Serial) { delay(10); }
  
  Serial.println(F("\n\n========================================"));
  Serial.println(F("[DEBUG] setup() - Start"));
  Serial.println(F("========================================"));

  Serial.println(F("[DEBUG] Step 1: Initializing GPIO"));
  pinMode(CAN_CS, OUTPUT);
  digitalWrite(CAN_CS, HIGH);
  Serial.println(F("[DEBUG] CS pin configured (HIGH)"));
  
  // 測試 CS 腳位
  Serial.println(F("[DEBUG] Testing CS pin..."));
  digitalWrite(CAN_CS, LOW);
  delay(10);
  digitalWrite(CAN_CS, HIGH);
  Serial.println(F("[DEBUG] CS pin test complete"));

  Serial.println(F("[DEBUG] Step 2: Initializing SPI"));
  SPI.begin(CAN_SCK, CAN_MISO, CAN_MOSI, CAN_CS);
  Serial.println(F("[DEBUG] SPI.begin() completed"));
  
  // 顯示 SPI 設定
  Serial.print(F("[DEBUG] SPI Configuration:"));
  Serial.print(F(" SCK="));  Serial.print(CAN_SCK);
  Serial.print(F(" MISO=")); Serial.print(CAN_MISO);
  Serial.print(F(" MOSI=")); Serial.print(CAN_MOSI);
  Serial.print(F(" CS="));   Serial.println(CAN_CS);

  Serial.println(F("# Exo Serial Control ready @115200"));

  Serial.println(F("[DEBUG] Step 3: Initializing ExoBus"));
  Serial.println(F("[DEBUG] This may take a few seconds..."));
  
  if (!exo.begin()) {
    Serial.println(F(""));
    Serial.println(F("========================================"));
    Serial.println(F("[ERROR] ExoBus initialization FAILED"));
    Serial.println(F("========================================"));
    Serial.println(F(""));
    Serial.println(F("Common issues:"));
    Serial.println(F("  1. Check MCP2515 module is powered (3.3V or 5V)"));
    Serial.println(F("  2. Verify SPI wiring:"));
    Serial.println(F("     ESP32 SCK(18)  -> MCP2515 SCK"));
    Serial.println(F("     ESP32 MISO(19) -> MCP2515 SO"));
    Serial.println(F("     ESP32 MOSI(23) -> MCP2515 SI"));
    Serial.println(F("     ESP32 CS(5)    -> MCP2515 CS"));
    Serial.println(F("  3. Check MCP2515 crystal frequency (8MHz or 16MHz)"));
    Serial.println(F("  4. Ensure CAN_H and CAN_L are connected properly"));
    Serial.println(F("  5. Check 120Ω termination resistor"));
    Serial.println(F(""));
    Serial.println(F("========================================"));
    
    while (true) { 
      Serial.println(F("[ERROR] System halted. Reset required."));
      delay(3000); 
    }
  }
  Serial.println(F("[DEBUG] ExoBus initialization successful"));

  Serial.println(F("[DEBUG] Step 4: Initializing Watchdog"));
  wdt.begin(2000);
  Serial.println(F("[DEBUG] Watchdog started (2000ms timeout)"));

  Serial.println(F("[DEBUG] Step 5: Initializing Console"));
  console.begin();
  Serial.println(F("[DEBUG] Console initialization successful"));
  
  Serial.println(F("========================================"));
  Serial.println(F("[DEBUG] setup() - Complete"));
  Serial.println(F("========================================\n"));
}

void loop() {
  static uint32_t loopCount = 0;
  static uint32_t lastDebugMs = 0;
  static bool firstLoop = true;
  
  loopCount++;
  
  // 首次進入 loop 時顯示提示
  if (firstLoop) {
    Serial.println(F("[INFO] Loop started, waiting for serial commands..."));
    Serial.println(F("[INFO] Try: HELP, STATUS 0, STOP, etc."));
    firstLoop = false;
  }
  
  // 檢查序列埠接收 - 即時顯示
  if (Serial.available() > 0) {
    int availableBytes = Serial.available();
    Serial.print(F("\n[RX] *** Received "));
    Serial.print(availableBytes);
    Serial.print(F(" byte(s) from serial ***\n"));
    Serial.print(F("[RX] First byte: 0x"));
    Serial.print(Serial.peek(), HEX);
    Serial.print(F(" ('"));
    Serial.print((char)Serial.peek());
    Serial.println(F("')"));
    
    // 顯示所有接收的字元（不消耗）
    Serial.print(F("[RX] Preview: \""));
    for (int i = 0; i < min(availableBytes, 20); i++) {
      char c = Serial.read();
      if (c == '\n') Serial.print(F("\\n"));
      else if (c == '\r') Serial.print(F("\\r"));
      else Serial.print(c);
    }
    Serial.println(F("\""));
    
    // 清空剩餘資料以防止干擾
    while (Serial.available() > 0) Serial.read();
  }
  
  // 每5秒印出一次 loop 統計（減少輸出）
  if (millis() - lastDebugMs >= 5000) {
    Serial.print(F("[DEBUG] loop() running @ "));
    Serial.print(loopCount / 5.0, 0);
    Serial.println(F(" Hz"));
    loopCount = 0;
    lastDebugMs = millis();
  }
  
  wdt.poll();
  console.poll();
  exo.poll();
}