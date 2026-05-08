#include <Arduino.h>

// ─────────────────────────────────────────────────────────────────────
//  Cherry-switch isolation test
// ─────────────────────────────────────────────────────────────────────
//  Goal: prove whether the cherry limit switch on GPIO 35 is reading
//  reliably, or whether the input is floating / picking up noise.
//
//  IMPORTANT — ESP32 GPIO 35 is INPUT-ONLY and has NO internal pull-up
//  or pull-down resistor. `INPUT_PULLUP` is silently ignored on pins
//  34–39. If your switch is wired NO + COM (COM -> GND), the pin will
//  FLOAT when the switch is open and read randomly. Fix: add an
//  external 10kΩ pull-up from GPIO 35 to 3.3V, or move the switch to
//  a pin that supports INPUT_PULLUP (e.g. 27, 32, 33).
//
//  This sketch:
//    • samples the pin every loop iteration
//    • logs every level transition with a microsecond timestamp
//    • prints a 1 Hz summary: current level + transitions/sec (a high
//      flip count with the switch untouched == floating input)
// ─────────────────────────────────────────────────────────────────────

#define CHERRY_PIN  12

// Try with INPUT_PULLUP first to confirm it's a no-op on GPIO 35.
// If you wire an external pull-up, change to plain INPUT.
#define CHERRY_MODE INPUT_PULLUP

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println();
  Serial.println("=== Cherry switch isolation test ===");
  Serial.printf("Pin: GPIO %d   Mode: %s\n",
                CHERRY_PIN,
                CHERRY_MODE == INPUT_PULLUP ? "INPUT_PULLUP (no-op on 34-39!)"
                                            : "INPUT (external pull-up required)");
  Serial.println("Watching for transitions. Press/release the switch.");
  Serial.println("If you see hundreds of transitions/sec while untouched,");
  Serial.println("the input is FLOATING — add a 10k pull-up to 3.3V.");
  Serial.println();

  pinMode(CHERRY_PIN, CHERRY_MODE);
}

void loop() {
  static int           lastLevel   = -1;
  static unsigned long lastSummary = 0;
  static unsigned long flipCount   = 0;

  int level = digitalRead(CHERRY_PIN);

  if (level != lastLevel) {
    flipCount++;
    Serial.printf("[%lu us] %s\n",
                  micros(),
                  level == LOW ? "LOW  (pressed if NO+COM->GND wiring)"
                               : "HIGH (released)");
    lastLevel = level;
  }

  unsigned long now = millis();
  if (now - lastSummary >= 1000) {
    lastSummary = now;
    Serial.printf("[summary] level=%s   transitions in last 1s: %lu\n",
                  level == LOW ? "LOW " : "HIGH",
                  flipCount);
    flipCount = 0;
  }
}
