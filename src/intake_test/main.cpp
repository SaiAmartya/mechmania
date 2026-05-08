#include <Arduino.h>

// ─────────────────────────────────────────────────────────────
// TEMPORARY DIAGNOSTIC SKETCH — INTAKE ONLY, NOT FOR RACE USE
//
// Purpose: Cycle the intake motor driver inputs individually so
// you can see exactly which pin/wire is broken. Mirrors the
// drive-motor diagnostic but targets the intake H-bridge.
//
// Pins match receiver/main.cpp INTAKE_* defines:
//   INTAKE_IN1 = GPIO18   (direction A)
//   INTAKE_IN2 = GPIO19   (direction B)
//   INTAKE_ENA = GPIO23   (PWM speed)
//
// Expected behavior (one second per phase):
//   1) IN1 HIGH only (IN2 LOW), ENA = 0   — should NOT spin (PWM off baseline)
//   2) FORWARD: IN1=H, IN2=L, ENA=PWM     — intake spins one way
//   3) REVERSE: IN1=L, IN2=H, ENA=PWM     — intake spins other way
//   4) ENA pulse only (IN1=L, IN2=L)      — should NOT spin (both inputs low = brake)
//   5) Brief all-stop, then repeat.
//
// If phase 2 spins but phase 3 doesn't (or vice versa), the
// failing phase's IN pin / wire is the broken one. If neither
// spins, suspect ENA / power / driver. Serial @ 115200 prints
// which phase is currently active.
// ─────────────────────────────────────────────────────────────

#define INTAKE_IN1   18
#define INTAKE_IN2   19
#define INTAKE_ENA   23

#define PWM_FREQ   1000
#define PWM_RES    8

#define CH_INTAKE  3   // match receiver's INTAKE_CHANNEL

#define PULSE_MS  1000
#define REST_MS    400

static void allOff() {
  digitalWrite(INTAKE_IN1, LOW);
  digitalWrite(INTAKE_IN2, LOW);
  ledcWrite(CH_INTAKE, 0);
}

static void phaseInBaseline(bool in1High, const char *label) {
  Serial.printf(">> %s\n", label);
  allOff();
  digitalWrite(INTAKE_IN1, in1High ? HIGH : LOW);
  digitalWrite(INTAKE_IN2, in1High ? LOW  : HIGH);
  ledcWrite(CH_INTAKE, 0);   // PWM off — should NOT spin
  delay(PULSE_MS);
  allOff();
  delay(REST_MS);
}

static void phaseSpin(bool forward, const char *label) {
  Serial.printf(">> %s\n", label);
  allOff();
  if (forward) {
    digitalWrite(INTAKE_IN1, HIGH);
    digitalWrite(INTAKE_IN2, LOW);
  } else {
    digitalWrite(INTAKE_IN1, LOW);
    digitalWrite(INTAKE_IN2, HIGH);
  }
  ledcWrite(CH_INTAKE, 255);
  delay(PULSE_MS);
  allOff();
  delay(REST_MS);
}

static void phasePwmOnlyBrake(const char *label) {
  Serial.printf(">> %s\n", label);
  allOff();
  digitalWrite(INTAKE_IN1, LOW);
  digitalWrite(INTAKE_IN2, LOW);
  ledcWrite(CH_INTAKE, 255);  // both inputs LOW = brake; should NOT spin
  delay(PULSE_MS);
  allOff();
  delay(REST_MS);
}

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("\n=== Intake pin diagnostic ===");
  Serial.println("Watch which phases fail to spin the intake motor.");

  pinMode(INTAKE_IN1, OUTPUT);
  pinMode(INTAKE_IN2, OUTPUT);

  ledcSetup(CH_INTAKE, PWM_FREQ, PWM_RES);
  ledcAttachPin(INTAKE_ENA, CH_INTAKE);

  allOff();
  delay(500);
}

void loop() {
  Serial.println("\n--- cycle start ---");
  phaseInBaseline(true,  "IN1=H IN2=L ENA=0  (sanity: should NOT spin)");
  phaseSpin(true,        "INTAKE FORWARD     (IN1=H, IN2=L, ENA=PWM)");
  phaseSpin(false,       "INTAKE REVERSE     (IN1=L, IN2=H, ENA=PWM)");
  phasePwmOnlyBrake(     "IN1=L IN2=L ENA=PWM (sanity: should NOT spin)");

  Serial.println("--- idle 2s ---");
  allOff();
  delay(2000);
}
