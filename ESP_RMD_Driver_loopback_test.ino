#include <SPI.h>
#include <mcp_can.h>

#define CS_PIN 5    // Chip Select for MCP2515
#define INT_PIN 4   // Interrupt pin for MCP2515

MCP_CAN CAN(CS_PIN);  // Initialize MCP2515

void setup() {
  Serial.begin(115200);

  // Initialize MCP2515 CAN bus at 1 Mbps (adjust as needed)
  while (CAN_OK != CAN.begin(MCP_ANY, CAN_1000KBPS, MCP_8MHZ)) {
    Serial.println("MCP2515 init failed, retrying...");
    delay(500);
  }
  CAN.setMode(MCP_NORMAL);  // Set to normal mode to receive messages
  pinMode(INT_PIN, INPUT);  // Interrupt pin for MCP2515
  Serial.println("MCP2515 initialized successfully.");
}

void loop() {
  long unsigned int rxId;
  unsigned char len = 0;
  unsigned char buf[8];

  // Check if CAN message is received
  if (CAN.readMsgBuf(&rxId, &len, buf) == CAN_OK) {
    bool ext = rxId & 0x80000000UL;  // Check if it’s an extended ID
    bool rtr = rxId & 0x40000000UL;  // Check if it's a Remote Transmission Request
    rxId &= 0x1FFFFFFF;              // Clear extended ID flag (standard ID)

    // Print the received message to Serial Monitor
    Serial.print("CAN ");
    Serial.print(ext ? "EXT" : "STD");
    Serial.print(" ID=0x"); Serial.print(rxId, HEX);
    Serial.print(" DLC="); Serial.print(len);
    Serial.print(" DATA: ");
    for (byte i = 0; i < len; i++) {
      Serial.print(buf[i], HEX);
      Serial.print(" ");
    }
    Serial.println();

    // Simulate motor feedback based on received command
    if (rxId == 0x141) {  // Command for Motor 1 (speed/position)
      byte reply_data[] = {0x9A, 0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06};  // Simulated feedback data
      CAN.sendMsgBuf(0x241, 0, 8, reply_data);  // Send feedback for Motor 1 (ID 0x241)
    }

    if (rxId == 0x142) {  // Command for Motor 2 (speed/position)
      byte reply_data[] = {0x9A, 0x00, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C};  // Simulated feedback data
      CAN.sendMsgBuf(0x242, 0, 8, reply_data);  // Send feedback for Motor 2 (ID 0x242)
    }
  }
}
