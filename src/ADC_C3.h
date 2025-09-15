// Arudino Sample Code to use ADS1256 library
// http://www.ti.com/lit/ds/symlink/ads1256.pdf

#include <Arduino.h>
#include "ADS1256.h"
#include <SPI.h>

float clockMHZ = 7.68; // crystal frequency used on ADS1256
float vRef = 2.47; // voltage reference

// Construct and init ADS1256 object
ADS1256 adc(clockMHZ, vRef, false); // RESETPIN is permanently tied to 3.3v

uint32_t rawValue;
float value;

void setup() {
  Serial.begin(115200);
  delay(2000); // Wait for serial to initialize
  Serial.println("Starting ADS1256");

  // Check pin definitions
  Serial.printf("Pin definitions - DRDY: %d, RST: %d, CS: %d\n", pinDRDY, pinRST, pinCS);

  // Manual pin setup for debugging
  pinMode(pinDRDY, INPUT);
  pinMode(pinCS, OUTPUT);
  digitalWrite(pinCS, HIGH); // CS idle high
  
  // start the ADS1256 with data rate of 15 SPS and gain x1
  adc.begin(ADS1256_DRATE_2_5SPS,ADS1256_GAIN_1,false); 
  
  // Check status register
  uint8_t status = adc.getStatus();
  Serial.printf("ADS1256 Status register: 0x%02X\n", status);
  
  // Try to read all registers for debugging
  Serial.println("Reading all registers:");
  for(int i = 0; i <= 0x0A; i++) {
    uint8_t regVal = adc.readRegister(i);
    Serial.printf("Reg 0x%02X: 0x%02X\n", i, regVal);
  }
 
  // Set MUX Register to AIN0 so it start doing the ADC conversion
  Serial.println("Channel set to Single end ch0 (A0)");
  adc.setChannel(0);
  adc.selfcal();
  
  // Check status after calibration
  status = adc.getStatus();
  Serial.printf("Status after calibration: 0x%02X\n", status);
  
  // Don't use continuous mode, use single conversions instead
  adc.sendCommand(ADS1256_CMD_SDATAC); // Stop continuous mode
  Serial.println("ADC initialized successfully");
  
  // Test DRDY pin
  Serial.printf("DRDY pin state: %d\n", digitalRead(pinDRDY));
}

void loop() {
  // Start a single conversion
  adc.sendCommand(ADS1256_CMD_SYNC);
  adc.sendCommand(ADS1256_CMD_WAKEUP);
  
  // Wait for DRDY to go low (data ready)
  adc.waitDRDY();
  
  // Read using the proper public method
  rawValue = adc.readCurrentChannelCRaw();
  value = adc.convertADStoVoltage(rawValue);

  Serial.printf("Ch0 (A0): 0x%08X (%d), U=%.4f V, DRDY=%d\n", 
                rawValue, rawValue, value, digitalRead(pinDRDY));
  
  delay(1000); // Slower for debugging
}