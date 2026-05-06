#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>

// *** REPLACE WITH YOUR RECEIVER ESP32 MAC ADDRESS ***
// Use the mac_address_finder environment on the receiver to get this MAC address.
uint8_t receiverMAC[] = {0x14, 0x33, 0x5C, 0x57, 0x8F, 0xA8};

// 14:33:5C:57:8F:A8 (new receiver MAC address)

#define JOY_X_PIN   A0    // VRX (Safe ADC1 pin)
#define JOY_Y_PIN   A1    // VRY (Safe ADC1 pin)

#define BTN1_PIN    D10   // Single Button
#define BTN2_PIN    D11   // Single Button

// Intake button pin (to reverse robot intake)
#define BTN3_PIN    D12   // Single Button
// Intake switch pin (to turn intake on/off)
#define BTN4_PIN    D13   // Single Switch

#define POWER LED_BUILTIN // POWER 

typedef struct ControllerData {
  int   joyX;
  int   joyY;
  bool  btn1;   // conveyor forward
  bool  btn2;   // conveyor backward
  bool  btn3;   // intake reverse (only meaningful when intake switch is on)
  bool  btn4;   // intake on/off switch
} ControllerData;

ControllerData myData;
esp_now_peer_info_t peerInfo;

void setup() {
  Serial.begin(115200);   // Changed to 115200 to match PlatformIO monitor_speed
  pinMode(BTN1_PIN, INPUT_PULLUP);
  pinMode(BTN2_PIN, INPUT_PULLUP);
  pinMode(BTN3_PIN, INPUT_PULLUP);
  pinMode(BTN4_PIN, INPUT_PULLUP);
  pinMode(POWER, OUTPUT);

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
  digitalWrite(POWER, HIGH); // Power on the receiver (for testing, can be removed if controller is always powered)

  myData.joyX = analogRead(JOY_X_PIN);
  myData.joyY = analogRead(JOY_Y_PIN);
  myData.btn1 = !digitalRead(BTN1_PIN);
  myData.btn2 = !digitalRead(BTN2_PIN);
  myData.btn3 = !digitalRead(BTN3_PIN);
  myData.btn4 = !digitalRead(BTN4_PIN);

  esp_now_send(receiverMAC, (uint8_t *) &myData, sizeof(myData));
  delay(20);

  // Serial output ALL raw data for debugging
  Serial.printf("Sent | X=%d Y=%d b1=%d b2=%d b3=%d b4=%d\n",
                myData.joyX, myData.joyY,
                myData.btn1, myData.btn2, myData.btn3, myData.btn4);
}
