#include <Arduino.h>
#include "CAN_commands.h" // Not strictly needed here, but good practice if main.cpp uses other CAN commands
#include "CAN.cpp" // Include the CAN implementation directly for simplicity in this example

void setup() {
  setup_can_bus(); // Initialize CAN bus (placeholder)
}

void loop() {
  send_motor_off_command(1); // Send motor off command for motor ID 1
  delay(5000); // Wait for 5 seconds before sending again
}
