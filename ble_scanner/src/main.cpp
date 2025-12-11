/*
 * ESP32 BLE Scanner for uMyo 1kHz ADC
 */

#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>

BLEScan* pBLEScan;

class MyCallbacks: public BLEAdvertisedDeviceCallbacks {
    void onResult(BLEAdvertisedDevice dev) {
      uint8_t* payload = dev.getPayload();
      size_t len = dev.getPayloadLength();

      // Check for uMyo in payload
      bool isUmyo = false;
      for (int i = 0; i < (int)len - 3; i++) {
        if (payload[i] == 'u' && payload[i+1] == 'M' && payload[i+2] == 'y' && payload[i+3] == 'o') {
          isUmyo = true;
          break;
        }
      }

      if (!isUmyo) return;

      // Found uMyo - parse manufacturer data
      int pos = 0;
      while (pos < (int)len - 1) {
        int adLen = payload[pos];
        if (adLen == 0 || pos + adLen >= (int)len) break;
        int adType = payload[pos + 1];

        if (adType == 0xFF && adLen >= 4) {
          uint8_t counter = payload[pos + 2];
          uint8_t num_samples = payload[pos + 3];

          // Print in format: PKT:counter,N:samples,ADC:v1,v2,v3...
          Serial.printf("PKT:%d,N:%d,RSSI:%d,ADC:", counter, num_samples, dev.getRSSI());

          int samples_available = (adLen - 3) / 2;  // How many 16-bit samples fit
          int samples_to_print = min((int)num_samples, min(samples_available, 10));

          for (int i = 0; i < samples_to_print; i++) {
            int16_t adc = (payload[pos + 4 + i*2] << 8) | payload[pos + 5 + i*2];
            Serial.printf("%d", adc);
            if (i < samples_to_print - 1) Serial.print(",");
          }
          Serial.println();
        }
        pos += adLen + 1;
      }
    }
};

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("\n=== uMyo 1kHz ADC Scanner ===");

  BLEDevice::init("");
  pBLEScan = BLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(new MyCallbacks());
  pBLEScan->setActiveScan(false);
  pBLEScan->setInterval(100);
  pBLEScan->setWindow(99);

  Serial.println("Scanning for uMyo...");
}

void loop() {
  pBLEScan->start(1, false);
  pBLEScan->clearResults();
}
