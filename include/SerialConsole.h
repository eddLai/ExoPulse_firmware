#pragma once
#include <Arduino.h>
#include "ExoBus.h"

class SerialConsole {
public:
  explicit SerialConsole(ExoBus& exo) : exo_(exo) {}
  void begin();                 // 只列印說明，不呼叫 Serial.begin()
  void poll();                  // 非阻塞讀取並解析一行命令

  // 新增：靜態回呼函式（供 ExoBus 呼叫）
  static void serialFeedback(const char* msg);
  static void setInstance(SerialConsole* inst) { instance_ = inst; }

private:
  void printHelp_();
  void handleLine_(String l);
  void feedWdt_() { lastWdtMs_ = millis(); }

  // 新增：內部回饋處理
  void handleFeedback_(const char* msg);

  ExoBus& exo_;
  String line_;
  uint32_t lastWdtMs_ = 0;
  static constexpr uint32_t kWdtMs_ = 3000;

  static SerialConsole* instance_;
};
