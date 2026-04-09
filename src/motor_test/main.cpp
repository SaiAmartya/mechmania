#include <Arduino.h>

// ─────────────────────────────────────────────────────────────
// TEMPORARY DIAGNOSTIC SKETCH — NOT FOR RACE USE
//
// Purpose: Cycle each of the 4 motor-driver inputs individually
// so you can see exactly which pin/wire is broken.
//
// Expected behavior (one second per phase):
//   1) Left motor spins FORWARD        (GPIO25 → M1A)
//   2) Left motor spins REVERSE        (GPIO26 → M1B)
//   3) Right motor spins FORWARD       (GPIO17 → M2A)
//   4) Right motor spins REVERSE       (GPIO16 → M2B)
//   5) Brief all-stop, then repeat.
//
// Any phase where the corresponding motor does NOT move is the
// broken wire / bad pin. Serial monitor @ 115200 prints which
// phase is currently active.
// ─────────────────────────────────────────────────────────────

#define LEFT_IN1   25   // M1A
#define LEFT_IN2   26   // M1B
#define RIGHT_IN1  17   // M2A
#define RIGHT_IN2  16   // M2B

#define PWM_FREQ   1000
#define PWM_RES    8

#define CH_L1  0
#define CH_L2  1
#define CH_R1  2
#define CH_R2  3

#define PULSE_MS  1000
#define REST_MS    400

static void allOff() {
  ledcWrite(CH_L1, 0);
  ledcWrite(CH_L2, 0);
  ledcWrite(CH_R1, 0);
  ledcWrite(CH_R2, 0);
}

static void pulse(int channel, const char *label) {
  Serial.printf(">> %s\n", label);
  allOff();
  ledcWrite(channel, 255);
  delay(PULSE_MS);
  allOff();
  delay(REST_MS);
}

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("\n=== Motor pin diagnostic ===");
  Serial.println("Watch which phases fail to spin a motor.");

  ledcSetup(CH_L1, PWM_FREQ, PWM_RES);
  ledcAttachPin(LEFT_IN1, CH_L1);

  ledcSetup(CH_L2, PWM_FREQ, PWM_RES);
  ledcAttachPin(LEFT_IN2, CH_L2);

  ledcSetup(CH_R1, PWM_FREQ, PWM_RES);
  ledcAttachPin(RIGHT_IN1, CH_R1);

  ledcSetup(CH_R2, PWM_FREQ, PWM_RES);
  ledcAttachPin(RIGHT_IN2, CH_R2);

  allOff();
  delay(500);
}

void loop() {
  Serial.println("\n--- cycle start ---");
  pulse(CH_L1, "LEFT  FORWARD  (GPIO25 -> M1A)");
  pulse(CH_L2, "LEFT  REVERSE  (GPIO26 -> M1B)");
  pulse(CH_R1, "RIGHT FORWARD  (GPIO17 -> M2A)");
  pulse(CH_R2, "RIGHT REVERSE  (GPIO16 -> M2B)");

  Serial.println("--- idle 2s ---");
  allOff();
  delay(2000);
}
