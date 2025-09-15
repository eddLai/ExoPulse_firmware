// Arudino Sample Code to use ADS1256 library
// http://www.ti.com/lit/ds/symlink/ads1256.pdf

#include <Arduino.h>
#include "ADS1256.h"
#include <SPI.h>

// 定義 HardwareSerial 實例，使用 UART1
HardwareSerial mySerial(1);

float clockMHZ = 7.68; // crystal frequency used on ADS1256
float vRef = 2.47; // voltage reference

// Construct and init ADS1256 object
ADS1256 adc(clockMHZ, vRef, false); // RESETPIN is permanently tied to 3.3v

uint32_t rawValue;
float value;
int currentChannel = 0;
const int maxChannels = 8;

void setup() {
  // 初始化 HardwareSerial，GPIO 20 (RX), GPIO 21 (TX)
  mySerial.begin(115200, SERIAL_8N1, 20, 21);
  mySerial.println("Starting ADS1256");

  // start the ADS1256 with data rate of 15 SPS and gain x1
  adc.begin(ADS1256_DRATE_2_5SPS,ADS1256_GAIN_1,false); 
 
  // 初始設定第一個通道
  mySerial.println("Starting dynamic channel scanning...");
  adc.setChannel(currentChannel);
  adc.selfcal();
  adc.sendCommand(ADS1256_CMD_RDATAC);
}

void loop() {
  // 設定當前通道
  adc.setChannel(currentChannel);
  delay(100); // 等待通道切換穩定
  
  rawValue = adc.readCurrentChannelCRaw();
  value = adc.convertADStoVoltage(rawValue);

  mySerial.print("Ch");
  mySerial.print(currentChannel);
  mySerial.print(": 0x");
  mySerial.print(rawValue, HEX);
  mySerial.print(", U=");
  mySerial.print(value, 4);
  mySerial.println(" V");
  
  // 切換到下一個通道
  currentChannel++;
  if (currentChannel >= maxChannels) {
    currentChannel = 0;
    mySerial.println("--- Channel cycle complete ---");
  }
  
  delay(500); // 每個通道間的延遲
}