
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

// 軟體看門狗（同時監控 2 個馬達 ID=1, ID=2）
class SoftWatchdog {
public:
  void begin(uint32_t timeoutMs) { 
    timeout_ = timeoutMs; 
    tripped_ = false;
    enabled_ = false;  // 初始禁用，首次命令後才啟用
  }
  
  void feed(int motorId = -1) { 
    uint32_t now = millis();
    if (motorId == 1 || motorId == -1) lastFeed_[0] = now;
    if (motorId == 2 || motorId == -1) lastFeed_[1] = now;
    tripped_ = false;
    enabled_ = true;  // 首次餵養後啟用看門狗
  }
  
  void poll() {
    if (!enabled_) return;  // 未啟用時不檢查
    
    uint32_t now = millis();
    bool motor1_timeout = (now - lastFeed_[0] > timeout_);
    bool motor2_timeout = (now - lastFeed_[1] > timeout_);
    
    if (!tripped_ && (motor1_timeout || motor2_timeout)) {
      tripped_ = true;
      Serial.print(F("! WDT timeout: "));
      if (motor1_timeout) Serial.print(F("Motor1 "));
      if (motor2_timeout) Serial.print(F("Motor2 "));
      Serial.println(F("-> STOP"));
      exo.stop();
    }
  }
  
private:
  uint32_t timeout_ = 200;
  uint32_t lastFeed_[2] = {0, 0};  // [0]=Motor1, [1]=Motor2
  bool tripped_ = false;
  bool enabled_ = false;
};

static SoftWatchdog wdt;
static void feedWdt(int motorId = -1) { wdt.feed(motorId); }

// 使用 SerialConsole 處理序列指令
static SerialConsole console(exo);

void setup() {
  Serial.begin(115200);
  while (!Serial) { delay(10); }
  
  Serial.println(F("\n# Exo Serial Control ready @115200"));

  pinMode(CAN_CS, OUTPUT);
  digitalWrite(CAN_CS, HIGH);
  
  SPI.begin(CAN_SCK, CAN_MISO, CAN_MOSI, CAN_CS);
  
  if (!exo.begin()) {
    Serial.println(F("[ERROR] ExoBus initialization FAILED"));
    Serial.println(F("Possible causes:"));
    Serial.println(F("  1. Check MCP2515 module is powered"));
    Serial.println(F("  2. Verify SPI wiring"));
    Serial.println(F("  3. Check CAN_H and CAN_L connections"));
    Serial.println(F("  4. Verify 120Ω termination resistor"));
    while (true) { delay(3000); }
  }
  Serial.println(F("[OK] ExoBus initialized"));

  wdt.begin(2000);  // 2 秒超時，但初始禁用

  console.begin();
}

void loop() {
  wdt.poll();
  console.poll();
  exo.poll();
}