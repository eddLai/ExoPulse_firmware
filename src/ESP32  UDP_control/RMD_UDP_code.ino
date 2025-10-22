#include <SPI.h>
#include "mcp_can.h"
#include <WiFi.h>
#include <WiFiUdp.h>
#include <ArduinoJson.h>

// ======================= WiFi / UDP =======================
const char* WIFI_SSID = "CaduHammer";
const char* WIFI_PASS = "ntklab6302";
WiFiUDP udp;
constexpr uint16_t UDP_PORT = 4210;
char incomingPacket[512];  // single-packet buffer (payload only)

// ======================= CAN (MCP2515) =======================
constexpr uint8_t CAN_CS_PIN  = 5;
constexpr uint8_t CAN_INT_PIN = 4;   // not strictly required here
MCP_CAN CAN(CAN_CS_PIN);

static inline uint16_t tx_id(uint8_t id){ return 0x140u + id; }
// static inline uint16_t rx_id(uint8_t id){ return 0x240u + id; } // not used in non-blocking path

// --- minimal non-blocking sender (8 bytes) ---
inline bool canSend8(uint16_t id, const uint8_t d[8]) {
  return CAN.sendMsgBuf(id, 0, 8, (byte*)d) == CAN_OK;
}

// --- pack 16-bit int (little-endian) into d[4..5] per RMD spec ---
inline void pack_le16(uint8_t d[8], int16_t v) {
  d[4] = (uint8_t)(v & 0xFF);
  d[5] = (uint8_t)((v >> 8) & 0xFF);
  d[6] = 0; d[7] = 0;
}

// ======================= RMD Commands (non-blocking) =======================
// Enter closed loop (0x80)
bool rmd_enterClosedLoop(uint8_t id) {
  uint8_t d[8] = {0x80,0,0,0,0,0,0,0};
  return canSend8(tx_id(id), d);
}

// Stop motor (0x81)
bool rmd_stop(uint8_t id) {
  uint8_t d[8] = {0x81,0,0,0,0,0,0,0};
  return canSend8(tx_id(id), d);
}

// Torque closed-loop (0xA1) — iq in Amps, 0.01 A/LSB
bool rmd_setTorque_A(uint8_t id, float iq_A) {
  // Clamp & quantize here
  // (RMD small hubs often allow |iq| ~ 6–12 A in torque loop; adjust below)
  static constexpr float TORQUE_MAX_A = 6.0f;     // <-- safety clamp, EDIT per motor
  if (iq_A >  TORQUE_MAX_A) iq_A =  TORQUE_MAX_A;
  if (iq_A < -TORQUE_MAX_A) iq_A = -TORQUE_MAX_A;

  int16_t iq_cnt = (int16_t)lrintf(iq_A * 100.0f); // 0.01 A/LSB
  uint8_t d[8] = {0xA1,0,0,0,0,0,0,0};
  pack_le16(d, iq_cnt);
  return canSend8(tx_id(id), d);
}

// ======================= System IDs / Mapping =======================
constexpr uint8_t RIGHT_MOTOR_ID = 1;
constexpr uint8_t LEFT_MOTOR_ID  = 2;

// ======================= Control Timing =======================
// 100 Hz scheduler
constexpr uint32_t SEND_INTERVAL_MS = 10;
uint32_t lastSendMs = 0;

// Safety timeout: if no UDP within this window, force zero torque
constexpr uint32_t COMMAND_TIMEOUT_MS = 100;
uint32_t lastRxMs = 0;

// ======================= Input Filtering / Scaling =======================
// If the sender gives angles instead of torque, use a gain to convert
// τ = k * θ  (super simple; replace with your proper controller as needed)
constexpr float HIP_TO_TORQUE_GAIN = 0.02f;  // A/deg (EDIT to taste)
constexpr float TORQUE_SLEW_A_PER_TICK = 0.6f; // limit torque change per 10 ms tick (A)

// Shared state updated by UDP (latest-sample-wins)
volatile float right_cmd_latest_A = 0.0f;
volatile float left_cmd_latest_A  = 0.0f;
volatile bool  estop_requested    = false;

// State applied at 100 Hz (after slew/filters)
float right_cmd_applied_A = 0.0f;
float left_cmd_applied_A  = 0.0f;

// ======================= Helpers =======================
static inline void connectWiFi() {
  Serial.print("Connecting to WiFi");
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  while (WiFi.status() != WL_CONNECTED) {
    delay(300);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected!");
  Serial.print("ESP32 IP: "); Serial.println(WiFi.localIP());
  udp.begin(UDP_PORT);
  Serial.printf("Listening UDP on %u\n", UDP_PORT);
}

static inline bool initCAN() {
  if (CAN.begin(MCP_STDEXT, CAN_1000KBPS, MCP_8MHZ) != CAN_OK) {
    Serial.println("CAN init FAIL");
    return false;
  }
  CAN.setMode(MCP_NORMAL);
  Serial.println("CAN init OK");
  return true;
}

static inline float clampf(float v, float lo, float hi) {
  return (v < lo) ? lo : (v > hi) ? hi : v;
}

static inline float slewTo(float current, float target, float step) {
  if (target > current + step) return current + step;
  if (target < current - step) return current - step;
  return target;
}

// ======================= Setup =======================
void setup() {
  Serial.begin(115200);
  delay(500);

  connectWiFi();
  if (!initCAN()) {
    while (true) delay(1000);
  }

  // Bring motors to known state
  rmd_enterClosedLoop(RIGHT_MOTOR_ID);
  rmd_enterClosedLoop(LEFT_MOTOR_ID);
  rmd_stop(RIGHT_MOTOR_ID);
  rmd_stop(LEFT_MOTOR_ID);

  lastRxMs = millis();
  lastSendMs = millis();
}

// ======================= Loop =======================
void loop() {
  // -------- 1) ASYNC RECEIVE: read newest UDP packet if available --------
  int packetSize = udp.parsePacket();
  if (packetSize > 0) {
    int len = udp.read(incomingPacket, sizeof(incomingPacket) - 1);
    if (len > 0) incomingPacket[len] = '\0';

    StaticJsonDocument<256> doc;
    DeserializationError err = deserializeJson(doc, incomingPacket);
    if (!err) {
      // Accept commands by torque or by angle (convert with gain)
      float r_A = NAN, l_A = NAN;

      if (doc.containsKey("right_torque")) r_A = doc["right_torque"].as<float>();
      if (doc.containsKey("left_torque"))  l_A = doc["left_torque"].as<float>();

      if (isnan(r_A) && doc.containsKey("right_hip")) {
        r_A = doc["right_hip"].as<float>() * HIP_TO_TORQUE_GAIN;
      }
      if (isnan(l_A) && doc.containsKey("left_hip")) {
        l_A = doc["left_hip"].as<float>() * HIP_TO_TORQUE_GAIN;
      }

      bool eStop = doc.containsKey("estop") ? doc["estop"].as<bool>() : false;

      // Update shared state atomically
      noInterrupts();
      if (!isnan(r_A)) right_cmd_latest_A = r_A;
      if (!isnan(l_A)) left_cmd_latest_A  = l_A;
      if (eStop) estop_requested = true;
      interrupts();

      lastRxMs = millis();
      // Debug (optional):
      // Serial.printf("RX: r=%.3fA l=%.3fA estop=%d\n", r_A, l_A, eStop);
    } else {
      // Serial.println("Invalid JSON");
    }
  }

  // -------- 2) 100 Hz SCHEDULER: send torque to motors every 10 ms --------
  uint32_t now = millis();
  if (now - lastSendMs >= SEND_INTERVAL_MS) {
    lastSendMs = now;

    // Safety: communication timeout or estop → zero torque
    bool timeout = (now - lastRxMs > COMMAND_TIMEOUT_MS);
    bool estop   = estop_requested;

    float target_right_A, target_left_A;
    if (timeout || estop) {
      target_right_A = 0.0f;
      target_left_A  = 0.0f;
    } else {
      // Snapshot latest commands atomically
      float r_A, l_A;
      noInterrupts();
      r_A = right_cmd_latest_A;
      l_A = left_cmd_latest_A;
      interrupts();

      // Optional: clamp again here if desired (extra safety)
      static constexpr float TORQUE_MAX_A = 6.0f; // keep in sync with rmd_setTorque_A
      target_right_A = clampf(r_A, -TORQUE_MAX_A, TORQUE_MAX_A);
      target_left_A  = clampf(l_A, -TORQUE_MAX_A, TORQUE_MAX_A);
    }

    // Slew-rate limit for smoothness (A per tick @ 100 Hz)
    right_cmd_applied_A = slewTo(right_cmd_applied_A, target_right_A, TORQUE_SLEW_A_PER_TICK);
    left_cmd_applied_A  = slewTo(left_cmd_applied_A,  target_left_A,  TORQUE_SLEW_A_PER_TICK);

    // Issue non-blocking torque commands (0xA1)
    rmd_setTorque_A(RIGHT_MOTOR_ID, right_cmd_applied_A);
    rmd_setTorque_A(LEFT_MOTOR_ID,  left_cmd_applied_A);

    // Optional heartbeat for debugging at 10 Hz to reduce spam:
    static uint8_t hb = 0;
    if ((hb++ % 10) == 0) {
      Serial.printf("[100Hz] τR=%.2fA τL=%.2fA  %s%s\n",
                    right_cmd_applied_A, left_cmd_applied_A,
                    (timeout ? "[TIMEOUT→0]" : ""),
                    (estop   ? " [ESTOP→0]"  : ""));
    }
  }
 
  // (No delay; loop stays responsive while 100 Hz pacing is handled above)
}
