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
  while (!Serial) { delay(10); } // for native USB boards

  // 先初始化 SPI 與 CS 腳位（之後 ExoBus/CAN 會使用到）
  pinMode(CAN_CS, OUTPUT);
  digitalWrite(CAN_CS, HIGH);              // 預設不選取
  SPI.begin(CAN_SCK, CAN_MISO, CAN_MOSI, CAN_CS);

  Serial.println(F("# Exo Serial Control ready @115200"));
  Serial.print(F("# CAN SPI pins -> SCK:")); Serial.print(CAN_SCK);
  Serial.print(F(" MISO:")); Serial.print(CAN_MISO);
  Serial.print(F(" MOSI:")); Serial.print(CAN_MOSI);
  Serial.print(F(" CS:"));   Serial.println(CAN_CS);

  if (!exo.begin()) {
    Serial.println(F("! ExoBus begin failed -> halt"));
    while (true) { delay(500); }
  }

  // 看門狗與序列控制台
  wdt.begin(2000);                 // 2 秒逾時
  console.begin();                 // 印出指令說明
}

void loop() {
  console.poll(); // 2. serial 接受、命令解析
  exo.poll();     // 3. canbus 轉換背景工作（如需要）
  wdt.poll();     // 1. 主要程式碼，看門狗
}