#include <Arduino.h>

// LED 測試程式
// 用於確認 ESP32 硬體正常工作

// ESP32 內建 LED
// ESP32: GPIO 2
// ESP32-S3: GPIO 48 (RGB LED) 或 GPIO 38
#if defined(CONFIG_IDF_TARGET_ESP32S3)
  #ifndef LED_BUILTIN
    #define LED_BUILTIN 48  // ESP32-S3 DevKitC-1 的 RGB LED
  #endif
#else
  #ifndef LED_BUILTIN
    #define LED_BUILTIN 2   // ESP32 的內建 LED
  #endif
#endif

// 可選的外接 LED
constexpr int LED_PINS[] = {2, 4, 16, 17, 18, 19, 21, 22, 23};
constexpr int NUM_LEDS = sizeof(LED_PINS) / sizeof(LED_PINS[0]);

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("\n========================================");
  Serial.println("   ESP32 LED Test Program");
  Serial.println("========================================");
  Serial.println();

  // 初始化所有 LED 腳位
  Serial.println("Initializing LED pins...");
  for (int i = 0; i < NUM_LEDS; i++) {
    pinMode(LED_PINS[i], OUTPUT);
    digitalWrite(LED_PINS[i], LOW);
    Serial.print("  GPIO ");
    Serial.print(LED_PINS[i]);
    Serial.println(" -> OUTPUT");
  }

  Serial.println("\nLED pins initialized!");
  Serial.println("\nStarting LED test patterns...\n");
  delay(500);
}

void loop() {
  static int testMode = 0;
  static uint32_t lastChange = 0;
  uint32_t now = millis();

  // 每 5 秒切換測試模式
  if (now - lastChange >= 5000) {
    lastChange = now;
    testMode = (testMode + 1) % 5;

    Serial.println("\n========================================");
    switch (testMode) {
      case 0:
        Serial.println("Test 1: Blink all LEDs together");
        break;
      case 1:
        Serial.println("Test 2: Running light (left to right)");
        break;
      case 2:
        Serial.println("Test 3: Running light (right to left)");
        break;
      case 3:
        Serial.println("Test 4: Alternate pattern");
        break;
      case 4:
        Serial.println("Test 5: Random blink");
        break;
    }
    Serial.println("========================================\n");
  }

  static uint32_t lastBlink = 0;
  static bool blinkState = false;
  static int runningPos = 0;

  // 每 200ms 更新一次
  if (now - lastBlink >= 200) {
    lastBlink = now;
    blinkState = !blinkState;

    switch (testMode) {
      case 0: // 全部一起閃爍
        for (int i = 0; i < NUM_LEDS; i++) {
          digitalWrite(LED_PINS[i], blinkState ? HIGH : LOW);
        }
        Serial.print("All LEDs: ");
        Serial.println(blinkState ? "ON" : "OFF");
        break;

      case 1: // 跑馬燈（左到右）
        for (int i = 0; i < NUM_LEDS; i++) {
          digitalWrite(LED_PINS[i], LOW);
        }
        digitalWrite(LED_PINS[runningPos], HIGH);
        Serial.print("LED GPIO ");
        Serial.print(LED_PINS[runningPos]);
        Serial.println(" ON");
        runningPos = (runningPos + 1) % NUM_LEDS;
        break;

      case 2: // 跑馬燈（右到左）
        for (int i = 0; i < NUM_LEDS; i++) {
          digitalWrite(LED_PINS[i], LOW);
        }
        runningPos = (runningPos - 1 + NUM_LEDS) % NUM_LEDS;
        digitalWrite(LED_PINS[runningPos], HIGH);
        Serial.print("LED GPIO ");
        Serial.print(LED_PINS[runningPos]);
        Serial.println(" ON");
        break;

      case 3: // 交替閃爍
        for (int i = 0; i < NUM_LEDS; i++) {
          digitalWrite(LED_PINS[i], (i % 2 == 0) ? (blinkState ? HIGH : LOW) : (blinkState ? LOW : HIGH));
        }
        Serial.println(blinkState ? "Even ON, Odd OFF" : "Even OFF, Odd ON");
        break;

      case 4: // 隨機閃爍
        for (int i = 0; i < NUM_LEDS; i++) {
          digitalWrite(LED_PINS[i], random(2) ? HIGH : LOW);
        }
        Serial.println("Random pattern");
        break;
    }
  }
}
