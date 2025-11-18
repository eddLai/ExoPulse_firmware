#pragma once
#include <Arduino.h>
#include <SPI.h>

// Driver selection
#ifdef EXOBUS_MCP2515_DEDALQQ
  #include <mcp_can.h> 
  using CanDriver = MCP_CAN;  // ✅ 正確
#else
  #include <mcp_can.h>
  using CanDriver = MCP_CAN;  // coryjfowler driver
#endif

#ifndef LED_BUILTIN
#define LED_BUILTIN 2
#endif

// 回呼函式型別：用於通知狀態變化
using ExoBusCallback = void (*)(const char* msg);

class ExoBus {
public:
  ExoBus() {
    // 建構函式留空，實際初始化在 begin()
  }

  ~ExoBus() {
    if (can_) {
      delete can_;
      can_ = nullptr;
    }
  }

  bool begin() {
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, LOW);

    notify_("[DEBUG] ExoBus::begin() - Start");
    notify_("[DEBUG] Checking if CAN driver is already initialized...");
    
    if (can_ != nullptr) {
      notify_("[WARN] CAN driver already exists, deleting...");
      delete can_;
      can_ = nullptr;
    }

#ifdef EXOBUS_MCP2515_DEDALQQ
    notify_("[DEBUG] Using dedalqq/esp32-mcp2515 driver");
    notify_("[DEBUG] Initializing SPI bus configuration");
    
    spi_bus_config_t busConfig = {};
    busConfig.miso_io_num = kSpiMiso_;
    busConfig.mosi_io_num = kSpiMosi_;
    busConfig.sclk_io_num = kSpiSck_;
    busConfig.quadwp_io_num = -1;
    busConfig.quadhd_io_num = -1;
    
    notify_("[DEBUG] Calling spi_bus_initialize()");
    esp_err_t ret = spi_bus_initialize(HSPI_HOST, &busConfig, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK) {
      char buf[64];
      snprintf(buf, sizeof(buf), "[ERROR] SPI bus init failed: %d", ret);
      notify_(buf);
      return false;
    }
    notify_("[DEBUG] SPI bus initialized successfully");

    notify_("[DEBUG] Configuring SPI device");
    spi_device_interface_config_t devConfig = {};
    devConfig.spics_io_num = kCanCsPin_;
    devConfig.clock_speed_hz = 1 * 1000 * 1000;
    devConfig.mode = 0;
    devConfig.queue_size = 1;
    
    notify_("[DEBUG] Calling spi_bus_add_device()");
    ret = spi_bus_add_device(HSPI_HOST, &devConfig, &spiHandle_);
    if (ret != ESP_OK) {
      char buf[64];
      snprintf(buf, sizeof(buf), "[ERROR] SPI device add failed: %d", ret);
      notify_(buf);
      return false;
    }
    notify_("[DEBUG] SPI device added successfully");

    notify_("[DEBUG] Creating MCP2515 object");
    can_ = new CanDriver(&spiHandle_);
    notify_("[DEBUG] MCP2515 object created");
#else
    notify_("[DEBUG] Using coryjfowler/mcp_can driver");
    
    char buf[64];
    snprintf(buf, sizeof(buf), "[DEBUG] CS Pin: %d", kCanCsPin_);
    notify_(buf);
    
    notify_("[DEBUG] Creating MCP_CAN object");
    try {
      can_ = new CanDriver(kCanCsPin_);
      if (can_ == nullptr) {
        notify_("[ERROR] Failed to allocate MCP_CAN object (nullptr)");
        return false;
      }
      notify_("[DEBUG] MCP_CAN object created successfully");
    } catch (...) {
      notify_("[ERROR] Exception during MCP_CAN object creation");
      return false;
    }
#endif

    notify_("[DEBUG] Calling canInit()");
    if (!canInit()) {
      notify_("[ERROR] CAN init failed");
      notify_("[ERROR] Possible causes:");
      notify_("[ERROR]   1. MCP2515 not connected");
      notify_("[ERROR]   2. Wrong SPI pins");
      notify_("[ERROR]   3. Wrong CS pin");
      notify_("[ERROR]   4. MCP2515 crystal frequency mismatch");
      notify_("[ERROR]   5. Power supply issue");
      return false;
    }
    notify_("[DEBUG] ExoBus initialization complete");
    return true;
  }

  void poll() {
    if (!canReady_) {
      notify_("[WARN] poll() called but CAN not ready");
      return;
    }
    uint32_t now = millis();
    if (now - lastReqMs_ >= kStatePeriodMs_) {
      notify_("[DEBUG] poll() - Requesting state");
      requestState_();
      lastReqMs_ = now;
    }
    processRx_();
  }

  bool setTorque(int jointId, double torqueNm) {
    notify_("[DEBUG] setTorque() called");
    char buf[64];
    snprintf(buf, sizeof(buf), "[DEBUG] jointId=%d, torqueNm=%.3f", jointId, torqueNm);
    notify_(buf);
    
    const double kNmMax = 30.0;
    if (!isfinite(torqueNm)) {
      notify_("[ERROR] torqueNm is not finite");
      return false;
    }
    if (torqueNm > kNmMax) {
      snprintf(buf, sizeof(buf), "[WARN] Torque clamped: %.3f -> %.3f", torqueNm, kNmMax);
      notify_(buf);
      torqueNm = kNmMax;
    }
    if (torqueNm < -kNmMax) {
      snprintf(buf, sizeof(buf), "[WARN] Torque clamped: %.3f -> %.3f", torqueNm, -kNmMax);
      notify_(buf);
      torqueNm = -kNmMax;
    }
    
    int16_t iq = (int16_t)round(torqueNm * kNmToIq_);
    snprintf(buf, sizeof(buf), "[DEBUG] Converted to iq=%d", iq);
    notify_(buf);
    
    uint8_t d[8] = {0xA1,0,0,0,0,0,0,0};
    d[4] = iq & 0xFF;
    d[5] = (iq >> 8) & 0xFF;
    
    notify_("[DEBUG] Calling canSend()");
    bool ok = canSend(motorBaseId_(jointId), d, 8);
    if (ok) {
      notify_("[DEBUG] setTorque() successful");
      logTx("TORQ", jointId, torqueNm);
    } else {
      notify_("[ERROR] setTorque() failed");
    }
    return ok;
  }

  bool setTorqueIq(int jointId, int16_t iq) {
    notify_("[DEBUG] setTorqueIq() called");
    char buf[64];
    snprintf(buf, sizeof(buf), "[DEBUG] jointId=%d, iq=%d", jointId, iq);
    notify_(buf);
    
    const int16_t kMaxAbsIq = 300;
    int16_t orig = iq;
    if (iq >  kMaxAbsIq) iq =  kMaxAbsIq;
    if (iq < -kMaxAbsIq) iq = -kMaxAbsIq;
    
    if (orig != iq) {
      snprintf(buf, sizeof(buf), "[WARN] Iq clamped: %d -> %d", orig, iq);
      notify_(buf);
    }
    
    uint8_t d[8] = {0xA1,0,0,0,0,0,0,0};
    d[4] = (uint8_t)(iq & 0xFF);
    d[5] = (uint8_t)((iq >> 8) & 0xFF);
    
    notify_("[DEBUG] Calling canSend()");
    bool ok = canSend(motorBaseId_(jointId), d, 8);
    if (ok) {
      notify_("[DEBUG] setTorqueIq() successful");
      logTx("TORQ_IQ", jointId, (double)iq);
    } else {
      notify_("[ERROR] setTorqueIq() failed");
    }
    return ok;
  }

  bool setSpeed(int jointId, float speedDps) {
    notify_("[DEBUG] setSpeed() called");
    char buf[64];
    snprintf(buf, sizeof(buf), "[DEBUG] jointId=%d, speedDps=%.2f", jointId, speedDps);
    notify_(buf);
    
    const float kMaxAbsSpeed = 50.0f;
    if (!isfinite(speedDps)) {
      notify_("[ERROR] speedDps is not finite");
      return false;
    }
    float orig = speedDps;
    if (speedDps >  kMaxAbsSpeed) speedDps =  kMaxAbsSpeed;
    if (speedDps < -kMaxAbsSpeed) speedDps = -kMaxAbsSpeed;
    
    if (orig != speedDps) {
      snprintf(buf, sizeof(buf), "[WARN] Speed clamped: %.2f -> %.2f dps", orig, speedDps);
      notify_(buf);
    }
    
    int32_t sp = (int32_t)(speedDps * 100.0f);
    snprintf(buf, sizeof(buf), "[DEBUG] Converted to raw speed=%ld", sp);
    notify_(buf);
    
    uint8_t d[8] = {0xA2,0,0,0,0,0,0,0};
    d[4] = (uint8_t)(sp & 0xFF);
    d[5] = (uint8_t)((sp >> 8) & 0xFF);
    d[6] = (uint8_t)((sp >> 16) & 0xFF);
    d[7] = (uint8_t)((sp >> 24) & 0xFF);
    
    notify_("[DEBUG] Calling canSend()");
    bool ok = canSend(motorBaseId_(jointId), d, 8);
    if (ok) {
      notify_("[DEBUG] setSpeed() successful");
      logTx("SPEED", jointId, speedDps);
    } else {
      notify_("[ERROR] setSpeed() failed");
    }
    return ok;
  }

  bool setPosition(int jointId, float angleDeg) {
    notify_("[DEBUG] setPosition() called");
    char buf[64];
    snprintf(buf, sizeof(buf), "[DEBUG] jointId=%d, angleDeg=%.2f", jointId, angleDeg);
    notify_(buf);
    
    if (!isfinite(angleDeg)) {
      notify_("[ERROR] angleDeg is not finite");
      return false;
    }
    
    int32_t p = (int32_t)(angleDeg * 100.0f);
    snprintf(buf, sizeof(buf), "[DEBUG] Converted to raw position=%ld", p);
    notify_(buf);
    
    uint8_t d[8] = {0xA3,0,0,0,0,0,0,0};
    d[4] = (uint8_t)(p & 0xFF);
    d[5] = (uint8_t)((p >> 8) & 0xFF);
    d[6] = (uint8_t)((p >> 16) & 0xFF);
    d[7] = (uint8_t)((p >> 24) & 0xFF);
    
    notify_("[DEBUG] Calling canSend()");
    bool ok = canSend(motorBaseId_(jointId), d, 8);
    if (ok) {
      notify_("[DEBUG] setPosition() successful");
      logTx("POS", jointId, (double)angleDeg);
    } else {
      notify_("[ERROR] setPosition() failed");
    }
    return ok;
  }

  bool stop() {
    notify_("[DEBUG] stop() called");
    int jointId = 0;
    uint8_t d[8] = {0xA1,0,0,0,0,0,0,0};
    bool ok = canReady_ ? canSend(motorBaseId_(jointId), d, 8) : true;
    if (ok) {
      notify_("[DEBUG] stop() successful");
    } else {
      notify_("[ERROR] stop() failed");
    }
    logTx("STOP", -1, 0);
    safeBlink(2, 60, 60);
    return ok;
  }

  bool callExo() {
    logTx("CALL", -1, 0);
    safeBlink(3);
    return true;
  }

  bool zeroAngle(int jointId) {
    notify_("[DEBUG] zeroAngle() called");
    char buf[64];
    snprintf(buf, sizeof(buf), "[DEBUG] jointId=%d", jointId);
    notify_(buf);
    
    uint8_t d[8] = {0x19,0,0,0,0,0,0,0};
    bool ok = canReady_ ? canSend(motorBaseId_(jointId), d, 8) : true;
    if (ok) {
      notify_("[DEBUG] zeroAngle() successful");
    } else {
      notify_("[ERROR] zeroAngle() failed");
    }
    logTx("ZERO", jointId, 0);
    safeBlink(1);
    return ok;
  }

  bool zeroAll() {
    notify_("[DEBUG] zeroAll() called");
    bool ok = zeroAngle(0);
    logTx("ZERO_ALL", -1, 0);
    safeBlink(2);
    return ok;
  }

  void setCallback(ExoBusCallback cb) { callback_ = cb; }

  struct MotorState {
    int jointId;
    float angleDeg;
    float speedDps;
    float iqA;
    int8_t temperature;
    bool valid;
  };

  MotorState getLastState(int jointId) const {
    if (jointId >= 0 && jointId < 4) {
      return lastState_[jointId];
    }
    return MotorState{};
  }

  // Dump recent received CAN frames. If motorId >= 0, filter by motor base id (0x140 + motorId).
  void dumpRecentFrames(int motorId = -1) {
    char buf[160];
    notify_("[DEBUG] dumpRecentFrames() - start");
    for (int i = 0; i < kRecentCount_; ++i) {
      int idx = (recentIdx_ + i) % kRecentCount_;
      const RecentFrame& f = recent_[idx];
      if (!f.valid) continue;
      if (motorId >= 0) {
        uint32_t want = motorBaseId_(motorId);
        if (f.id != want) continue;
      }
      int pos = snprintf(buf, sizeof(buf), "[RXDBG] ts=%lu id=0x%03lX len=%d data=", (unsigned long)f.ts, (unsigned long)f.id, f.len);
      for (int b = 0; b < f.len && b < 8; ++b) pos += snprintf(buf + pos, sizeof(buf) - pos, " %02X", f.data[b]);
      notify_(buf);
    }
    notify_("[DEBUG] dumpRecentFrames() - end");
  }

private:
  struct RecentFrame {
    uint32_t id = 0;
    uint8_t len = 0;
    uint8_t data[8] = {};
    uint32_t ts = 0;
    bool valid = false;
  };

  static constexpr int kRecentCount_ = 32;
  RecentFrame recent_[kRecentCount_] = {};
  int recentIdx_ = 0;

  void logRecentFrame_(uint32_t id, uint8_t len, const uint8_t* d) {
    RecentFrame& r = recent_[recentIdx_];
    r.id = id;
    r.len = len;
    r.ts = millis();
    r.valid = true;
    for (int i = 0; i < 8; ++i) r.data[i] = (i < len && d != nullptr) ? d[i] : 0;
    recentIdx_ = (recentIdx_ + 1) % kRecentCount_;
  }
  void logTx(const char* what, int id, double v) {
    Serial.print(F("[TX] "));
    Serial.print(what);
    if (id >= 0) {
      Serial.print(F(" id="));
      Serial.print(id);
    }
    if (strcmp(what, "TORQ") == 0) {
      Serial.print(F(" Nm="));
      Serial.print(v, 3);
    }
    Serial.println();
  }

  void safeBlink(int times, int onMs = 120, int offMs = 120) {
    for (int i = 0; i < times; ++i) {
      digitalWrite(LED_BUILTIN, HIGH);
      delay(onMs);
      digitalWrite(LED_BUILTIN, LOW);
      delay(offMs);
    }
  }

  bool canInit() {
    notify_("[DEBUG] canInit() - Start");
    
    if (can_ == nullptr) {
      notify_("[ERROR] CAN driver is nullptr!");
      return false;
    }
    
#ifdef EXOBUS_MCP2515_DEDALQQ
    notify_("[DEBUG] Resetting MCP2515...");
    can_->reset();
    notify_("[DEBUG] Reset complete");
    
    notify_("[DEBUG] Setting bitrate 1Mbps@8MHz...");
    auto err = can_->setBitrate(CAN_1000KBPS, MCP_8MHZ);
    if (err != MCP2515::ERROR_OK) {
      char buf[64];
      snprintf(buf, sizeof(buf), "[WARN] 1Mbps failed (err=%d), trying 500kbps...", (int)err);
      notify_(buf);
      err = can_->setBitrate(CAN_500KBPS, MCP_8MHZ);
      if (err != MCP2515::ERROR_OK) {
        snprintf(buf, sizeof(buf), "[ERROR] Bitrate config failed (err=%d)", (int)err);
        notify_(buf);
        return false;
      }
      notify_("[DEBUG] Bitrate set to 500kbps");
    } else {
      notify_("[DEBUG] Bitrate set to 1Mbps");
    }
    
    notify_("[DEBUG] Setting NORMAL mode...");
    can_->setNormalMode();
    notify_("[DEBUG] MCP2515 in NORMAL mode");
#else
    notify_("[DEBUG] Initializing MCP_CAN...");
    notify_("[DEBUG] Attempting: MCP_STDEXT, CAN_1000KBPS, MCP_8MHZ");
    
    byte result = can_->begin(MCP_STDEXT, CAN_1000KBPS, MCP_8MHZ);
    
    char buf[64];
    snprintf(buf, sizeof(buf), "[DEBUG] MCP_CAN::begin() returned: %d", result);
    notify_(buf);
    
    if (result != CAN_OK) {
      notify_("[ERROR] MCP_CAN begin failed");
      notify_("[DEBUG] Trying alternative configurations...");
      
      // 嘗試 16MHz 晶振
      notify_("[DEBUG] Trying: CAN_1000KBPS @ MCP_16MHZ");
      result = can_->begin(MCP_STDEXT, CAN_1000KBPS, MCP_16MHZ);
      snprintf(buf, sizeof(buf), "[DEBUG] Result: %d", result);
      notify_(buf);
      
      if (result != CAN_OK) {
        // 嘗試 500kbps @ 8MHz
        notify_("[DEBUG] Trying: CAN_500KBPS @ MCP_8MHZ");
        result = can_->begin(MCP_STDEXT, CAN_500KBPS, MCP_8MHZ);
        snprintf(buf, sizeof(buf), "[DEBUG] Result: %d", result);
        notify_(buf);
        
        if (result != CAN_OK) {
          // 嘗試 500kbps @ 16MHz
          notify_("[DEBUG] Trying: CAN_500KBPS @ MCP_16MHZ");
          result = can_->begin(MCP_STDEXT, CAN_500KBPS, MCP_16MHZ);
          snprintf(buf, sizeof(buf), "[DEBUG] Result: %d", result);
          notify_(buf);
          
          if (result != CAN_OK) {
            notify_("[ERROR] All MCP_CAN configuration attempts failed");
            notify_("[ERROR] Please check hardware connections");
            return false;
          }
          notify_("[DEBUG] Success with CAN_500KBPS @ MCP_16MHZ");
        } else {
          notify_("[DEBUG] Success with CAN_500KBPS @ MCP_8MHZ");
        }
      } else {
        notify_("[DEBUG] Success with CAN_1000KBPS @ MCP_16MHZ");
      }
    } else {
      notify_("[DEBUG] MCP_CAN begin successful with initial config");
    }
    
    notify_("[DEBUG] Setting NORMAL mode...");
    can_->setMode(MCP_NORMAL);
    notify_("[DEBUG] MCP_CAN in NORMAL mode");
#endif
    canReady_ = true;
    notify_("[DEBUG] canInit() - Complete");
    return true;
  }

  bool canSend(uint32_t id, const uint8_t* data, size_t len) {
    char buf[128];
    snprintf(buf, sizeof(buf), "[DEBUG] canSend() - ID=0x%03lX, len=%d", id, len);
    notify_(buf);
    
    if (!canReady_ || !can_) {
      notify_("[ERROR] CAN not ready for send");
      return false;
    }
    if (len > 8) {
      notify_("[ERROR] CAN frame too long");
      return false;
    }
    
    // 顯示要發送的資料
    snprintf(buf, sizeof(buf), "[DEBUG] Data: %02X %02X %02X %02X %02X %02X %02X %02X",
             data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7]);
    notify_(buf);
    
#ifdef EXOBUS_MCP2515_DEDALQQ
    struct can_frame f;
    f.can_id  = id;
    f.can_dlc = (uint8_t)len;
    for (uint8_t i = 0; i < len; ++i) f.data[i] = data[i];
    
    notify_("[DEBUG] Calling MCP2515::sendMessage()");
    auto err = can_->sendMessage(&f);
    if (err != MCP2515::ERROR_OK) {
      snprintf(buf, sizeof(buf), "[ERROR] CAN send error: %d", (int)err);
      notify_(buf);
      return false;
    }
    notify_("[DEBUG] CAN send successful");
    return true;
#else
    notify_("[DEBUG] Calling MCP_CAN::sendMsgBuf()");
    byte rc = can_->sendMsgBuf(id, 0, (byte)len, (uint8_t*)data);
    if (rc != CAN_OK) {
      snprintf(buf, sizeof(buf), "[ERROR] CAN send error: %d", (int)rc);
      notify_(buf);
      return false;
    }
    notify_("[DEBUG] CAN send successful");
    return true;
#endif
  }

  void requestState_() {
    notify_("[DEBUG] requestState_() - Sending state request");
    uint8_t d[8] = {0x9C,0,0,0,0,0,0,0};
    canSend(motorBaseId_(0), d, 8);
  }

  void processRx_() {
    if (!canReady_ || !can_) return;
#ifdef EXOBUS_MCP2515_DEDALQQ
    struct can_frame f;
    while (can_->readMessage(&f) == MCP2515::ERROR_OK) {
      notify_("[DEBUG] processRx_() - Message received");
      // store recent frame for debug
      logRecentFrame_(f.can_id, f.can_dlc, f.data);
      parseStateFrame_(f.can_id, f.can_dlc, f.data);
    }
#else
    while (can_->checkReceive() == CAN_MSGAVAIL) {
      notify_("[DEBUG] processRx_() - Message available");
      unsigned long rxId;
      byte len;
      uint8_t buf[8];
      can_->readMsgBuf(&rxId, &len, buf);
      // store recent frame for debug
      logRecentFrame_(rxId, len, buf);
      parseStateFrame_(rxId, len, buf);
    }
#endif
  }

  void parseStateFrame_(uint32_t rxId, uint8_t len, const uint8_t* buf) {
    char dbg[128];
    snprintf(dbg, sizeof(dbg), "[DEBUG] parseStateFrame_() - ID=0x%03lX, len=%d, cmd=0x%02X", 
             rxId, len, buf[0]);
    notify_(dbg);
    
    if (len == 8 && buf[0] == 0x9C && rxId >= 0x140 && rxId < 0x160) {
      int jointId = (int)(rxId - 0x140);
      snprintf(dbg, sizeof(dbg), "[DEBUG] Valid state frame for joint %d", jointId);
      notify_(dbg);
      
      if (jointId < 0 || jointId >= 4) {
        notify_("[WARN] Joint ID out of range");
        return;
      }
      
      int8_t temperature = (int8_t)buf[1];
      int16_t iq_raw = (int16_t)(buf[2] | (buf[3] << 8));
      int16_t speed_raw = (int16_t)(buf[4] | (buf[5] << 8));
      uint16_t encoder = (uint16_t)(buf[6] | (buf[7] << 8));
      uint16_t encoder14 = encoder & 0x3FFF;
      
      float iq_A = (float)iq_raw * 33.0f / 2048.0f;
      float speed_dps = (float)speed_raw;
      float angle_deg = (float)encoder14 * 360.0f / 16384.0f;
      
      lastState_[jointId].jointId = jointId;
      lastState_[jointId].angleDeg = angle_deg;
      lastState_[jointId].speedDps = speed_dps;
      lastState_[jointId].iqA = iq_A;
      lastState_[jointId].temperature = temperature;
      lastState_[jointId].valid = true;
      
      char buf2[128];
      snprintf(buf2, sizeof(buf2), 
               "[RX] J=%d ang=%.2f sp=%.1f iq=%.3f T=%d",
               jointId, angle_deg, speed_dps, iq_A, temperature);
      notify_(buf2);
    } else {
      notify_("[DEBUG] Frame does not match state frame criteria");
    }
  }

  void notify_(const char* msg) {
    if (callback_) {
      callback_(msg);
    } else {
      Serial.println(msg);
    }
  }

  uint32_t motorBaseId_(int jointId) { return 0x140 + jointId; }

  bool canReady_ = false;
  uint32_t lastReqMs_ = 0;
  static constexpr uint32_t kStatePeriodMs_ = 1000;
  static constexpr int kCanCsPin_ = 5;
  static constexpr float kNmToIq_ = 20.0f;
  static constexpr int kSpiSck_  = 18;
  static constexpr int kSpiMiso_ = 19;
  static constexpr int kSpiMosi_ = 23;

#ifdef EXOBUS_MCP2515_DEDALQQ
  spi_device_handle_t spiHandle_ = nullptr;
  CanDriver* can_ = nullptr;
#else
  CanDriver* can_ = nullptr;
#endif

  ExoBusCallback callback_ = nullptr;
  MotorState lastState_[4] = {};
};
