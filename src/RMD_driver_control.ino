#include <SPI.h>
#include "mcp_can.h"

// ===== Hardware pins =====
constexpr uint8_t CAN_CS_PIN = 5;          // MCP2515 CS
constexpr uint8_t CAN_INT_PIN = 4;         // MCP2515 INT (optional, not strictly needed)

// ===== Protocol helpers =====
static inline uint16_t le16(uint16_t v){ return v; }
static inline uint32_t le32(uint32_t v){ return v; }

// Host -> Motor: 0x140 + ID(1..32)
// Motor -> Host: 0x240 + ID(1..32)
static inline uint16_t tx_id(uint8_t id){ return 0x140u + id; }
static inline uint16_t rx_id(uint8_t id){ return 0x240u + id; }

MCP_CAN CAN(CAN_CS_PIN);

// --- send 8-byte frame ---
bool canSend(uint16_t id, const uint8_t d[8]) {
  byte rc = CAN.sendMsgBuf(id, 0, 8, (byte*)d);
  return rc == CAN_OK;
}

// --- wait for a reply frame with matching ID and expected command byte in DATA[0] ---
bool waitReply(uint16_t expect_id, uint8_t expect_cmd, uint8_t out[8], uint32_t timeout_ms=20) {
  uint32_t t0 = millis();
  while (millis() - t0 < timeout_ms) {
    if (CAN.checkReceive() != CAN_MSGAVAIL) continue;
    unsigned long rid; byte len; uint8_t buf[8];
    CAN.readMsgBuf(&rid, &len, buf);
    if (len == 8 && rid == expect_id && buf[0] == expect_cmd) {
      memcpy(out, buf, 8);
      return true;
    }
  }
  return false;
}

// --- utilities to pack 16/32-bit little-endian into DATA[4..7] per spec ---
void pack_le16(uint8_t d[8], int16_t v) {
  d[4] = uint8_t(v & 0xFF);
  d[5] = uint8_t((v >> 8) & 0xFF);
  d[6] = 0; d[7] = 0;
}
void pack_le32(uint8_t d[8], int32_t v) {
  d[4] = uint8_t(v & 0xFF);
  d[5] = uint8_t((v >> 8) & 0xFF);
  d[6] = uint8_t((v >> 16) & 0xFF);
  d[7] = uint8_t((v >> 24) & 0xFF);
}
int32_t unpack_le32(const uint8_t d[8]) {
  return (int32_t)((uint32_t)d[4] | ((uint32_t)d[5] << 8) | ((uint32_t)d[6] << 16) | ((uint32_t)d[7] << 24));
}

// ===== High-level commands per RMD protocol =====
// Enter closed-loop (0x80)
bool enterClosedLoop(uint8_t id) {
  uint8_t d[8] = {0x80,0,0,0,0,0,0,0};
  if (!canSend(tx_id(id), d)) return false;
  uint8_t r[8]; return waitReply(rx_id(id), 0x80, r, 50);
}

// Stop motor (0x81) — stops motion but stays in mode
bool stopMotor(uint8_t id) {
  uint8_t d[8] = {0x81,0,0,0,0,0,0,0};
  if (!canSend(tx_id(id), d)) return false;
  uint8_t r[8]; return waitReply(rx_id(id), 0x81, r, 50);
}

// Torque closed-loop (0xA1), iqControl in 0.01 A/LSB
bool setTorque_A(uint8_t id, float iq_ampere) {
  int16_t iq_cnt = (int16_t)round(iq_ampere * 100.0f); // 0.01 A
  uint8_t d[8] = {0xA1,0,0,0,0,0,0,0};
  pack_le16(d, iq_cnt);  // DATA[4..5]
  if (!canSend(tx_id(id), d)) return false;
  uint8_t r[8]; return waitReply(rx_id(id), 0xA1, r, 50);
}

// Speed closed-loop (0xA2), speed in 0.01 deg/s
bool setSpeed_dps(uint8_t id, float dps) {
  int32_t sp_cnt = (int32_t)llround(dps * 100.0); // 0.01 deg/s
  uint8_t d[8] = {0xA2,0,0,0,0,0,0,0};
  pack_le32(d, sp_cnt);
  if (!canSend(tx_id(id), d)) return false;
  uint8_t r[8]; return waitReply(rx_id(id), 0xA2, r, 50);
}

// Absolute position closed-loop (0xA4), angle in 0.01 deg
bool setAbsPos_deg(uint8_t id, float deg) {
  int32_t ang_cnt = (int32_t)llround(deg * 100.0); // 0.01 deg
  uint8_t d[8] = {0xA4,0,0,0,0,0,0,0};
  pack_le32(d, ang_cnt);
  if (!canSend(tx_id(id), d)) return false;
  uint8_t r[8]; return waitReply(rx_id(id), 0xA4, r, 50);
}

// Incremental position closed-loop (0xA8), delta in 0.01 deg
bool addIncPos_deg(uint8_t id, float delta_deg) {
  int32_t inc_cnt = (int32_t)llround(delta_deg * 100.0); // 0.01 deg
  uint8_t d[8] = {0xA8,0,0,0,0,0,0,0};
  pack_le32(d, inc_cnt);
  if (!canSend(tx_id(id), d)) return false;
  uint8_t r[8]; return waitReply(rx_id(id), 0xA8, r, 50);
}

// Read multi-turn absolute angle (0x92), returns degrees
bool readAngle_deg(uint8_t id, float &out_deg) {
  uint8_t d[8] = {0x92,0,0,0,0,0,0,0};
  if (!canSend(tx_id(id), d)) return false;
  uint8_t r[8];
  if (!waitReply(rx_id(id), 0x92, r, 50)) return false;
  int32_t raw = unpack_le32(r);       // 0.01 deg/LSB
  out_deg = (float)raw / 100.0f;
  return true;
}

// Read status1/voltage/temp/error (0x9A)
bool readStatus(uint8_t id, int8_t &tempC, float &voltageV, uint16_t &errFlags, uint8_t &brakeState) {
  uint8_t d[8] = {0x9A,0,0,0,0,0,0,0};
  if (!canSend(tx_id(id), d)) return false;
  uint8_t r[8];
  if (!waitReply(rx_id(id), 0x9A, r, 50)) return false;
  tempC      = (int8_t)r[1];
  brakeState = r[3];                           // 1=released, 0=locked
  uint16_t u = (uint16_t)r[4] | ((uint16_t)r[5] << 8);
  voltageV   = u * 0.1f;                       // 0.1 V/LSB
  errFlags   = (uint16_t)r[6] | ((uint16_t)r[7] << 8);
  return true;
}

void printAngle(uint8_t id) {
  float deg;
  if (readAngle_deg(id, deg)) {
    Serial.printf("[ID %u] angle = %.2f deg\n", id, deg);
  } else {
    Serial.printf("[ID %u] angle read FAILED\n", id);
  }
}

void demo(uint8_t id) {
  Serial.printf("\n=== Demo sequence on ID %u ===\n", id);

  if (!enterClosedLoop(id)) Serial.println("enterClosedLoop FAILED");

  // Speed: +90 deg/s for 2 s, then stop
  if (setSpeed_dps(id, 90.0f)) Serial.println("Speed set to +90 deg/s");
  delay(2000);
  stopMotor(id);

  // Absolute move to +180 deg
  if (setAbsPos_deg(id, 180.0f)) Serial.println("Abs move to 180 deg");
  delay(1500); printAngle(id);

  // Incremental +90 deg
  if (addIncPos_deg(id, +90.0f)) Serial.println("Inc +90 deg");
  delay(1500); printAngle(id);

  // Small torque nudge (0.50 A) – use with care
  setTorque_A(id, 0.50f);
  delay(500);
  stopMotor(id);

  // Read status
  int8_t tC; float v; uint16_t e; uint8_t brake;
  if (readStatus(id, tC, v, e, brake)) {
    Serial.printf("Status: T=%d C, V=%.1f V, Brake=%u, Err=0x%04X\n", tC, v, brake, e);
  }
}

void setup() {
  Serial.begin(115200);
  while(!Serial);

  if (CAN.begin(MCP_STDEXT, CAN_1000KBPS, MCP_8MHZ) != CAN_OK) {
    Serial.println("CAN init FAIL"); while(1) delay(100);
  }
  CAN.setMode(MCP_NORMAL);
  Serial.println("CAN init OK");

  // ---- Pick your motor IDs here ----
  uint8_t ids[] = {1, 2}; // change to the IDs you discovered
  for (uint8_t i=0;i<sizeof(ids);++i) demo(ids[i]);

  Serial.println("\nDemo complete.");
}

void loop() {
  // No continuous loop; 
}
