#pragma once
#include <Arduino.h>
#include <SPI.h>

// Driver selection
#ifdef EXOBUS_MCP2515_DEDALQQ
  #include <mcp2515.h>
  using CanDriver = MCP2515;
#else
  #include "mcp_can.h"
  using CanDriver = MCP_CAN;
#endif

// 回呼函式型別：用於通知狀態變化
using ExoBusCallback = void (*)(const char* msg);

class ExoBus {
public:
  ExoBus();
  ~ExoBus();
  bool begin();
  void poll();

  bool setTorque(int jointId, double torqueNm);
  // 新增：直接下 Iq、速度、位置的指令（對齊 test0）
  bool setTorqueIq(int jointId, int16_t iq);
  bool setSpeed(int jointId, float speedDps);
  bool setPosition(int jointId, float angleDeg);

  bool stop();
  bool callExo();
  bool zeroAngle(int jointId);
  bool zeroAll();

  // 設定回呼函式
  void setCallback(ExoBusCallback cb) { callback_ = cb; }

  // 取得最新狀態
  struct MotorState {
    int jointId;
    float angleDeg;
    float speedDps;
    float iqA;
    int8_t temperature;
    bool valid;
  };
  MotorState getLastState(int jointId) const;

private:
  void logTx(const char* what, int id, double v);
  void safeBlink(int times, int onMs = 120, int offMs = 120);
  // CAN
  bool canInit();
  bool canSend(uint32_t id, const uint8_t* data, size_t len);
  void requestState_();
  void processRx_();
  void parseStateFrame_(uint32_t rxId, uint8_t len, const uint8_t* buf);
  // 狀態
  bool canReady_ = false;
  uint32_t lastReqMs_ = 0;
  static constexpr uint32_t kStatePeriodMs_ = 50;
  static constexpr int kCanCsPin_ = 5;
  static constexpr float kNmToIq_ = 20.0f; // 暫定換算係數: Nm -> Iq，可調整
  // 新增：ESP32 VSPI 腳位（對齊 test0）
  static constexpr int kSpiSck_  = 18;
  static constexpr int kSpiMiso_ = 19;
  static constexpr int kSpiMosi_ = 23;

#ifdef EXOBUS_MCP2515_DEDALQQ
  spi_device_handle_t spiHandle_ = nullptr;
  CanDriver* can_ = nullptr; // 改用指標
#else
  CanDriver* can_ = nullptr; // 改用指標
#endif

  uint32_t motorBaseId_(int jointId) { return 0x140 + jointId; }

  ExoBusCallback callback_ = nullptr;
  MotorState lastState_[4] = {}; // 支援最多 4 個關節

  void notify_(const char* msg);
};
