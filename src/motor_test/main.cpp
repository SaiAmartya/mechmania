#include <Arduino.h>

// ─────────────────────────────────────────────────────────────
// TEMPORARY DIAGNOSTIC SKETCH — NOT FOR RACE USE
//
// Purpose: Cycle each of the 4 motor-driver inputs individually
// so you can see exactly which pin/wire is broken.
//
// Expected behavior (one second per phase):
//   1) Left motor spins FORWARD        (GPIO25 → IN1)
//   2) Left motor spins REVERSE        (GPIO26 → IN2)
//   3) Right motor spins FORWARD       (GPIO17 → IN1)
//   4) Right motor spins REVERSE       (GPIO16 → IN2)
//   5) Brief all-stop, then repeat.
//
// Any phase where the corresponding motor does NOT move is the
// broken wire / bad pin. Serial monitor @ 115200 prints which
// phase is currently active.
// ─────────────────────────────────────────────────────────────

#define LEFT_IN1   25   // D2  – direction pin A
#define LEFT_IN2   26   // D3  – direction pin B
#define LEFT_ENA   14   // D6  – PWM speed (replaced D4/27 since it doesn't exist)

#define RIGHT_IN1  16   // D10 – direction pin A
#define RIGHT_IN2  17   // D11 – direction pin B
#define RIGHT_ENB   4   // D12 – PWM speed (replaced A4 since it's far away)

#define PWM_FREQ   1000
#define PWM_RES    8

#define CH_L  0
#define CH_R  1

#define PULSE_MS  1000
#define REST_MS    400

static void allOff() {
  digitalWrite(LEFT_IN1, LOW);
  digitalWrite(LEFT_IN2, LOW);
  digitalWrite(RIGHT_IN1, LOW);
  digitalWrite(RIGHT_IN2, LOW);
  ledcWrite(CH_L, 0);
  ledcWrite(CH_R, 0);
}

static void pulseLeft(bool forward, const char *label) {
  Serial.printf(">> %s\n", label);
  allOff();
  if (forward) {
    digitalWrite(LEFT_IN1, HIGH);
    digitalWrite(LEFT_IN2, LOW);
  } else {
    digitalWrite(LEFT_IN1, LOW);
    digitalWrite(LEFT_IN2, HIGH);
  }
  ledcWrite(CH_L, 255);
  delay(PULSE_MS);
  allOff();
  delay(REST_MS);
}

static void pulseRight(bool forward, const char *label) {
  Serial.printf(">> %s\n", label);
  allOff();
  if (forward) {
    digitalWrite(RIGHT_IN1, HIGH);
    digitalWrite(RIGHT_IN2, LOW);
  } else {
    digitalWrite(RIGHT_IN1, LOW);
    digitalWrite(RIGHT_IN2, HIGH);
  }
  ledcWrite(CH_R, 255);
  delay(PULSE_MS);
  allOff();
  delay(REST_MS);
}

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("\n=== Motor pin diagnostic ===");
  Serial.println("Watch which phases fail to spin a motor.");

  pinMode(LEFT_IN1, OUTPUT);
  pinMode(LEFT_IN2, OUTPUT);
  pinMode(RIGHT_IN1, OUTPUT);
  pinMode(RIGHT_IN2, OUTPUT);

  ledcSetup(CH_L, PWM_FREQ, PWM_RES);
  ledcAttachPin(LEFT_ENA, CH_L);

  ledcSetup(CH_R, PWM_FREQ, PWM_RES);
  ledcAttachPin(RIGHT_ENB, CH_R);

  allOff();
  delay(500);
}

void loop() {
  Serial.println("\n--- cycle start ---");
  pulseLeft(true,  "LEFT  FORWARD  (IN1=H, IN2=L, ENA=PWM)");
  pulseLeft(false, "LEFT  REVERSE  (IN1=L, IN2=H, ENA=PWM)");
  pulseRight(true, "RIGHT FORWARD  (IN1=H, IN2=L, ENB=PWM)");
  pulseRight(false,"RIGHT REVERSE  (IN1=L, IN2=H, ENB=PWM)");

  Serial.println("--- idle 2s ---");
  allOff();
  delay(2000);
}
