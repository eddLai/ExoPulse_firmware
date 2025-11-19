#pragma once
#include <Arduino.h>
#include "ExoBus.h"

class SerialConsole {
public:
  explicit SerialConsole(ExoBus& exo) : exo_(exo) {}
  
  void begin() {
    instance_ = this;
    exo_.setCallback(SerialConsole::serialFeedback);
    
    Serial.println(F("# ========================================"));
    Serial.println(F("# ExoBus Serial Console"));
    Serial.println(F("# ========================================"));
    printHelp_();
  }

  void poll() {
    while (Serial.available() > 0) {
      char c = (char)Serial.read();
      if (c == '\r') continue;
      if (c == '\n') {
        Serial.print(F("[RX] "));
        Serial.println(line_);
        handleLine_(line_);
        line_ = "";
      } else {
        if (line_.length() < 200) line_ += c;
      }
    }
  }

  static void serialFeedback(const char* msg) {
    if (instance_) {
      instance_->handleFeedback_(msg);
    } else {
      Serial.println(msg);
    }
  }

  static void setInstance(SerialConsole* inst) { instance_ = inst; }

private:
  void printHelp_() {
    Serial.println(F("# Commands:"));
    Serial.println(F("#   TORQ <id> <Nm>      - Set torque (Nm)"));
    Serial.println(F("#   IQ <id> <iq>        - Set torque (Iq raw)"));
    Serial.println(F("#   SPEED <id> <dps>    - Set speed (deg/s)"));
    Serial.println(F("#   POS <id> <deg>      - Set position (deg)"));
    Serial.println(F("#   ZERO <id|ALL>       - Zero encoder"));
    Serial.println(F("#   STATUS <id>         - Query motor state"));
    Serial.println(F("#   DEBUG [<id>]        - Dump recent CAN frames (no arg = all)."));
    Serial.println(F("#                         id accepts 0-3 or 1-4 (1-based auto-converted)."));
    Serial.println(F("#   STOP                - Emergency stop"));
    Serial.println(F("#   HELP                - Show this help"));
    Serial.println(F("# ========================================"));
  }

  void handleFeedback_(const char* msg) {
    Serial.println(msg);
    feedWdt_();
  }

  void handleLine_(String l) {
    l.trim();
    if (l.length() == 0) return;

    String cmd, a1, a2;
    int sp1 = l.indexOf(' ');
    if (sp1 < 0) {
      cmd = l;
    } else {
      cmd = l.substring(0, sp1);
      String rest = l.substring(sp1 + 1);
      rest.trim();
      int sp2 = rest.indexOf(' ');
      if (sp2 < 0) {
        a1 = rest;
      } else {
        a1 = rest.substring(0, sp2);
        a2 = rest.substring(sp2 + 1);
        a2.trim();
      }
    }
    cmd.toUpperCase();

    if (cmd == "HELP") {
      printHelp_();
      Serial.println(F("OK"));
      feedWdt_();
      return;
    }

    if (cmd == "STOP") {
      if (exo_.stop()) { Serial.println(F("OK")); feedWdt_(); }
      else Serial.println(F("ERR: stop failed"));
      return;
    }

    if (cmd == "STATUS") {
      if (a1.length() == 0) {
        Serial.println(F("ERR: usage STATUS <id>"));
        return;
      }
      int id = a1.toInt();
      auto state = exo_.getLastState(id);
      if (state.valid) {
        Serial.print(F("# Motor "));
        Serial.print(id);
        Serial.print(F(": ang="));
        Serial.print(state.angleDeg, 2);
        Serial.print(F(" deg, sp="));
        Serial.print(state.speedDps, 1);
        Serial.print(F(" dps, iq="));
        Serial.print(state.iqA, 3);
        Serial.print(F(" A, T="));
        Serial.println(state.temperature);
        Serial.println(F("OK"));
      } else {
        Serial.println(F("ERR: no valid state"));
      }
      feedWdt_();
      return;
    }

    if (cmd == "TORQ" || cmd == "T") {
      if (a1.length() == 0 || a2.length() == 0) {
        Serial.println(F("ERR: usage TORQ <id> <Nm>"));
        return;
      }
      int id = a1.toInt();
      double nm = a2.toFloat();
      if (exo_.setTorque(id, nm)) { Serial.println(F("OK")); feedWdt_(id); }
      else Serial.println(F("ERR: setTorque failed"));
      return;
    }

    if (cmd == "IQ") {
      if (a1.length() == 0 || a2.length() == 0) {
        Serial.println(F("ERR: usage IQ <id> <iq>"));
        return;
      }
      int id = a1.toInt();
      int16_t iq = (int16_t)a2.toInt();
      if (exo_.setTorqueIq(id, iq)) { Serial.println(F("OK")); feedWdt_(id); }
      else Serial.println(F("ERR: setTorqueIq failed"));
      return;
    }

    if (cmd == "SPEED" || cmd == "S") {
      if (a1.length() == 0 || a2.length() == 0) {
        Serial.println(F("ERR: usage SPEED <id> <dps>"));
        return;
      }
      int id = a1.toInt();
      float dps = a2.toFloat();
      if (exo_.setSpeed(id, dps)) { Serial.println(F("OK")); feedWdt_(id); }
      else Serial.println(F("ERR: setSpeed failed"));
      return;
    }

    if (cmd == "POS" || cmd == "P") {
      if (a1.length() == 0 || a2.length() == 0) {
        Serial.println(F("ERR: usage POS <id> <deg>"));
        return;
      }
      int id = a1.toInt();
      float deg = a2.toFloat();
      if (exo_.setPosition(id, deg)) { Serial.println(F("OK")); feedWdt_(id); }
      else Serial.println(F("ERR: setPosition failed"));
      return;
    }

    if (cmd == "ZERO") {
      if (a1.length() == 0) {
        Serial.println(F("ERR: usage ZERO <id|ALL>"));
        return;
      }
      a1.toUpperCase();
      bool ok = false;
      if (a1 == "ALL") {
        ok = exo_.zeroAll();
      } else {
        int id = a1.toInt();
        ok = exo_.zeroAngle(id);
      }
      if (ok) { Serial.println(F("OK")); feedWdt_(); }
      else Serial.println(F("ERR: zero failed"));
      return;
    }

    if (cmd == "DEBUG") {
      if (a1.length() == 0) {
        exo_.dumpRecentFrames(-1);
        Serial.println(F("OK"));
        return;
      }
      int jid = a1.toInt();
      if (jid >= 1 && jid <= 4) jid = jid - 1;
      if (jid < 0 || jid > 3) {
        Serial.println(F("ERR: DEBUG requires motor id (0-3 or 1-4) or no arg"));
        return;
      }
      exo_.dumpRecentFrames(jid);
      Serial.println(F("OK"));
      return;
    }

    Serial.print(F("[WARN] Unknown command: "));
    Serial.println(cmd);
    Serial.println(F("ERR: unknown command"));
    printHelp_();
  }

  void feedWdt_(int motorId = -1) { lastWdtMs_ = millis(); }

  ExoBus& exo_;
  String line_;
  uint32_t lastWdtMs_ = 0;
  static constexpr uint32_t kWdtMs_ = 3000;

  static SerialConsole* instance_;
};

// 靜態成員初始化（放在 .h 檔尾，需要 inline 避免重複定義）
inline SerialConsole* SerialConsole::instance_ = nullptr;
