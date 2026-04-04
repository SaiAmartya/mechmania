#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>

// *** REPLACE WITH YOUR RECEIVER ESP32 MAC ADDRESS ***
// Use the mac_address_finder environment on the receiver to get this MAC address.
uint8_t receiverMAC[] = {0x08, 0xA6, 0xF7, 0x64, 0x8F, 0xCC};

#define JOY_X_PIN   A0    // VRX (Safe ADC1 pin)
#define JOY_Y_PIN   A1    // VRY (Safe ADC1 pin)
#define BTN1_PIN    D10   // RAISE
#define BTN2_PIN    D11   // LOWER

typedef struct ControllerData {
  int   joyX;
  int   joyY;
  bool  btn1;
  bool  btn2;
} ControllerData;

ControllerData myData;
esp_now_peer_info_t peerInfo;

void setup() {
  Serial.begin(115200);   // Changed to 115200 to match PlatformIO monitor_speed
  pinMode(BTN1_PIN, INPUT_PULLUP);
  pinMode(BTN2_PIN, INPUT_PULLUP);
  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);

  WiFi.mode(WIFI_STA);

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed!");
    return;
  }

  memcpy(peerInfo.peer_addr, receiverMAC, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer!");
    return;
  }
  Serial.println("Controller ready! Sending data...");
}

void loop() {
  myData.joyX = analogRead(JOY_X_PIN);
  myData.joyY = analogRead(JOY_Y_PIN);
  myData.btn1 = !digitalRead(BTN1_PIN);
  myData.btn2 = !digitalRead(BTN2_PIN);

  esp_now_send(receiverMAC, (uint8_t *) &myData, sizeof(myData));
  delay(20);
}
