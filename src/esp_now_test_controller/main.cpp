// ESP-NOW connectivity test — CONTROLLER side.
// Reads the real joystick + buttons (same pins as production) and sends them
// to the receiver every 50 ms. Adds a sequence counter so the receiver can
// detect dropped packets. Uses the ESP-NOW send callback to report whether
// every frame was acknowledged by the peer's MAC layer.
//
// Drives no motor pins. Touches no H-bridge / stepper / intake hardware.

#include <Arduino.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <WiFi.h>

// *** MUST match the receiver's printed MAC. ***
uint8_t receiverMAC[] = {0x14, 0x33, 0x5C, 0x57, 0x8F, 0xA8};

// 14:33:5C:57:8F:A8

// Lock both sides to the same WiFi channel.
#define ESPNOW_CHANNEL 1

// ── Pins (same as production controller/main.cpp) ──────────────
#define JOY_X_PIN  A0    // VRX (ADC1)
#define JOY_Y_PIN  A1    // VRY (ADC1)
#define BTN1_PIN   D10   // conveyor forward
#define BTN2_PIN   D11   // conveyor backward
#define BTN3_PIN   D12   // intake reverse
#define BTN4_PIN   D13   // intake on/off switch
#define POWER_PIN  LED_BUILTIN

// Send rate: same as production (~50 Hz).
#define SEND_PERIOD_MS 50
// Stats / human-readable line cadence.
#define LOG_PERIOD_MS  500

// Same payload as production receiver expects, plus a sequence counter for
// drop detection. The receiver test prints both.
typedef struct __attribute__((packed)) ControllerData {
  uint32_t seq;
  int      joyX;
  int      joyY;
  bool     btn1;   // conveyor forward
  bool     btn2;   // conveyor backward
  bool     btn3;   // intake reverse (only meaningful with btn4 on)
  bool     btn4;   // intake on/off switch
} ControllerData;

ControllerData payload;

volatile uint32_t txOk    = 0;
volatile uint32_t txFail  = 0;
volatile bool     lastAck = false;

uint32_t lastSendMs  = 0;
uint32_t lastLogMs   = 0;
uint32_t lastStatsMs = 0;

void onDataSent(const uint8_t *mac, esp_now_send_status_t status) {
  if (status == ESP_NOW_SEND_SUCCESS) {
    txOk++;
    lastAck = true;
  } else {
    txFail++;
    lastAck = false;
  }
}

void setup() {
  Serial.begin(115200);
  delay(300);

  pinMode(BTN1_PIN, INPUT_PULLUP);
  pinMode(BTN2_PIN, INPUT_PULLUP);
  pinMode(BTN3_PIN, INPUT_PULLUP);
  pinMode(BTN4_PIN, INPUT_PULLUP);
  pinMode(POWER_PIN, OUTPUT);
  digitalWrite(POWER_PIN, LOW);

  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  Serial.println();
  Serial.println("=== ESP-NOW TEST :: CONTROLLER (full input) ===");
  Serial.print  ("My MAC:        "); Serial.println(WiFi.macAddress());
  Serial.printf ("Peer MAC:      %02X:%02X:%02X:%02X:%02X:%02X\n",
                 receiverMAC[0], receiverMAC[1], receiverMAC[2],
                 receiverMAC[3], receiverMAC[4], receiverMAC[5]);

  esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);
  Serial.printf ("WiFi channel:  %d (locked)\n", ESPNOW_CHANNEL);
  Serial.printf ("Payload size:  %u bytes\n", (unsigned)sizeof(ControllerData));
  Serial.printf ("Send period:   %d ms\n", SEND_PERIOD_MS);

  if (esp_now_init() != ESP_OK) {
    Serial.println("FATAL: esp_now_init() failed");
    while (true) { delay(1000); }
  }
  esp_now_register_send_cb(onDataSent);

  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, receiverMAC, 6);
  peer.channel = ESPNOW_CHANNEL;
  peer.encrypt = false;
  if (esp_now_add_peer(&peer) != ESP_OK) {
    Serial.println("FATAL: esp_now_add_peer() failed");
    while (true) { delay(1000); }
  }

  Serial.println("Setup complete. Streaming controller input...");
  Serial.println("LED ON = last frame ACKed by receiver. LED OFF = no ACK.");
  Serial.println("--------------------------------------");
}

void loop() {
  uint32_t now = millis();

  if (now - lastSendMs >= SEND_PERIOD_MS) {
    lastSendMs = now;

    payload.seq++;
    payload.joyX = analogRead(JOY_X_PIN);
    payload.joyY = analogRead(JOY_Y_PIN);
    payload.btn1 = !digitalRead(BTN1_PIN);
    payload.btn2 = !digitalRead(BTN2_PIN);
    payload.btn3 = !digitalRead(BTN3_PIN);
    payload.btn4 = !digitalRead(BTN4_PIN);

    esp_err_t r = esp_now_send(receiverMAC, (uint8_t *)&payload, sizeof(payload));
    if (r != ESP_OK) {
      Serial.printf("[#%lu] esp_now_send err=%d\n",
                    (unsigned long)payload.seq, (int)r);
    }

    digitalWrite(POWER_PIN, lastAck ? HIGH : LOW);
  }

  // Human-readable input snapshot every 500 ms (don't spam at 50 Hz).
  if (now - lastLogMs >= LOG_PERIOD_MS) {
    lastLogMs = now;
    Serial.printf("[INPUT seq=%lu] X=%4d Y=%4d  b1=%d b2=%d b3=%d b4=%d  "
                  "lastAck=%s\n",
                  (unsigned long)payload.seq,
                  payload.joyX, payload.joyY,
                  payload.btn1, payload.btn2, payload.btn3, payload.btn4,
                  lastAck ? "OK" : "FAIL");
  }

  // Stats every 5 s.
  if (now - lastStatsMs >= 5000) {
    lastStatsMs = now;
    uint32_t total = txOk + txFail;
    float rate = total ? (100.0f * txOk / total) : 0.0f;
    Serial.printf("[STATS] sent=%lu ok=%lu fail=%lu  ack-rate=%.1f%%\n",
                  (unsigned long)total,
                  (unsigned long)txOk,
                  (unsigned long)txFail,
                  rate);
  }
}
