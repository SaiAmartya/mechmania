#include <Arduino.h>
#include <WiFi.h>

void setup() {
  Serial.begin(115200);   // Changed to 115200 to match PlatformIO monitor_speed
  WiFi.mode(WIFI_STA);
  Serial.println();
  Serial.print("Receiver ESP32 MAC Address: ");
  Serial.println(WiFi.macAddress());
}

void loop() {
  // Nothing to do here
}
