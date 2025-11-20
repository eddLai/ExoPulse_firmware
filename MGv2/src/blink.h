#include <Arduino.h>
#define LED_BUILTIN 2
#define LED_BUILTIN_C3 8

// 定義 HardwareSerial 實例，使用 UART1
HardwareSerial mySerial(1);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  pinMode(LED_BUILTIN_C3, OUTPUT);
  
  // 初始化 HardwareSerial，GPIO 20 (RX), GPIO 21 (TX)
  mySerial.begin(115200, SERIAL_8N1, 20, 21);
  
  Serial.println("ESP32 LED blink program started");
  Serial.println("Starting LED blink loop...");
  
  // 同步輸出到 hardware UART
  mySerial.println("ESP32 LED blink program started");
  mySerial.println("Starting LED blink loop...");
  mySerial.println("Hardware UART on GPIO 20(RX), 21(TX) is working!");
}  

void loop() {
  // put your main code here, to run repeatedly:
    Serial.println("LED ON");
    mySerial.println("LED ON");  // 同步輸出到 hardware UART
    digitalWrite(LED_BUILTIN_C3, HIGH);
    delay(3000);
    
    Serial.println("LED OFF");
    mySerial.println("LED OFF");  // 同步輸出到 hardware UART
    digitalWrite(LED_BUILTIN_C3, LOW);
    delay(3000);
}