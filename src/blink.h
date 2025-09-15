#include <Arduino.h>
#define LED_BUILTIN 2
#define LED_BUILTIN_C3 8

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  pinMode(LED_BUILTIN_C3, OUTPUT);
  
  Serial.println("ESP32 LED blink program started");
  Serial.println("Starting LED blink loop...");
}  

void loop() {
  // put your main code here, to run repeatedly:
    Serial.println("LED ON");
    digitalWrite(LED_BUILTIN_C3, HIGH);
    delay(3000);
    
    Serial.println("LED OFF");
    digitalWrite(LED_BUILTIN_C3, LOW);
    delay(3000);
}