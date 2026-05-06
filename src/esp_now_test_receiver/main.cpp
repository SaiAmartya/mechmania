// ESP-NOW connectivity test — RECEIVER side.
// Prints its own MAC at boot, then prints every frame it receives from the
// controller test (joystick, buttons, sequence number, sender MAC, age).
// Tracks dropped packets via the sequence counter and reports rolling stats.
//
// Drives no motor pins. Pure radio diagnostic.

#include <Arduino.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <WiFi.h>

#define ESPNOW_CHANNEL 1

// Same payload the controller test sends.
typedef struct __attribute__((packed)) ControllerData {
  uint32_t seq;
  int      joyX;
  int      joyY;
  bool     btn1;
  bool     btn2;
  bool     btn3;
  bool     btn4;
} ControllerData;

ControllerData incoming;

volatile uint32_t rxCount      = 0;
volatile uint32_t lastRxMs     = 0;
volatile uint32_t lastSeq      = 0;
volatile uint32_t droppedTotal = 0;
volatile bool     gotFirst     = false;
volatile uint32_t badLenCount  = 0;

uint32_t lastStatsMs = 0;

// Decode joystick into the same direction labels the production receiver uses.
#define DEAD_LOW   1200
#define DEAD_HIGH  2900

static const char *moveLabel(int x, int y) {
  bool fwd = x > DEAD_HIGH;
  bool bwd = x < DEAD_LOW;
  bool lt  = y < DEAD_LOW;
  bool rt  = y > DEAD_HIGH;
  if (fwd && lt)  return "FWD-LEFT";
  if (fwd && rt)  return "FWD-RIGHT";
  if (bwd && lt)  return "BACK-LEFT";
  if (bwd && rt)  return "BACK-RIGHT";
  if (fwd)        return "FORWARD";
  if (bwd)        return "BACKWARD";
  if (lt)         return "TURN-LEFT";
  if (rt)         return "TURN-RIGHT";
  return "STOP";
}

static const char *btnLabel(const ControllerData &d) {
  if (d.btn1)             return "CONV-FWD";
  if (d.btn2)             return "CONV-REV";
  if (d.btn4 && d.btn3)   return "INTAKE-REV";
  if (d.btn4)             return "INTAKE-ON";
  return "NONE";
}

void onDataReceived(const uint8_t *mac, const uint8_t *data, int len) {
  digitalWrite(LED_BUILTIN, HIGH);

  if (len != (int)sizeof(ControllerData)) {
    badLenCount++;
    Serial.printf("[RX BAD-LEN] from %02X:%02X:%02X:%02X:%02X:%02X  "
                  "got=%d expected=%u\n",
                  mac[0], mac[1], mac[2], mac[3], mac[4], mac[5],
                  len, (unsigned)sizeof(ControllerData));
    digitalWrite(LED_BUILTIN, LOW);
    return;
  }

  memcpy(&incoming, data, sizeof(incoming));

  // Sequence drop tracking.
  if (gotFirst && incoming.seq > lastSeq + 1) {
    droppedTotal += (incoming.seq - lastSeq - 1);
  }
  lastSeq  = incoming.seq;
  lastRxMs = millis();
  rxCount++;
  gotFirst = true;

  Serial.printf("[RX] from %02X:%02X:%02X:%02X:%02X:%02X  seq=%lu  "
                "X=%4d Y=%4d  b1=%d b2=%d b3=%d b4=%d  "
                "move=%-10s  cmd=%-10s  rx=%lu  dropped=%lu\n",
                mac[0], mac[1], mac[2], mac[3], mac[4], mac[5],
                (unsigned long)incoming.seq,
                incoming.joyX, incoming.joyY,
                incoming.btn1, incoming.btn2, incoming.btn3, incoming.btn4,
                moveLabel(incoming.joyX, incoming.joyY),
                btnLabel(incoming),
                (unsigned long)rxCount,
                (unsigned long)droppedTotal);

  digitalWrite(LED_BUILTIN, LOW);
}

void setup() {
  Serial.begin(115200);
  delay(300);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);

  Serial.println();
  Serial.println("=== ESP-NOW TEST :: RECEIVER (full input) ===");
  Serial.print  ("My MAC:        "); Serial.println(WiFi.macAddress());
  Serial.println(">>> Copy the MAC above into the controller's receiverMAC[] array.");
  Serial.printf ("WiFi channel:  %d (locked)\n", ESPNOW_CHANNEL);
  Serial.printf ("Expecting payload size: %u bytes\n",
                 (unsigned)sizeof(ControllerData));

  if (esp_now_init() != ESP_OK) {
    Serial.println("FATAL: esp_now_init() failed");
    while (true) { delay(1000); }
  }
  esp_now_register_recv_cb(onDataReceived);

  Serial.println("Setup complete. Waiting for packets...");
  Serial.println("--------------------------------------");
}

void loop() {
  uint32_t now = millis();

  if (now - lastStatsMs >= 5000) {
    lastStatsMs = now;
    if (!gotFirst) {
      Serial.println("[STATS] NO PACKETS RECEIVED YET");
    } else {
      uint32_t silentMs = now - lastRxMs;
      uint32_t expected = lastSeq;                 // seq starts at 1
      uint32_t received = rxCount;
      float lossPct = expected
                      ? (100.0f * (expected - received) / expected)
                      : 0.0f;
      Serial.printf("[STATS] rx=%lu last_seq=%lu dropped=%lu loss=%.1f%% "
                    "bad_len=%lu silent_for=%lu ms%s\n",
                    (unsigned long)received,
                    (unsigned long)lastSeq,
                    (unsigned long)droppedTotal,
                    lossPct,
                    (unsigned long)badLenCount,
                    (unsigned long)silentMs,
                    silentMs > 1000 ? "  <-- LINK STALLED" : "");
    }
  }

  delay(20);
}
