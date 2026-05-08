#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>

// This runs on the RECEIVER ESP32 - Firebeetle2 DFRobot ESP32e

// ── Motor Driver Pins (L298N / similar H-bridge) ──────────────
// Left motor
#define LEFT_IN1   25   // D2  – direction pin A
#define LEFT_IN2   26   // D3  – direction pin B
#define LEFT_ENA   14   // D6  – PWM speed 

// Right motor
#define RIGHT_IN1  16   // D10 – direction pin A
#define RIGHT_IN2  17   // D11 – direction pin B
#define RIGHT_ENB   4   // D12 – PWM speed

// ── Conveyor motor (L298N / similar H-bridge) ─────────────────
#define CONVEYOR_IN1  13   // direction pin A
#define CONVEYOR_IN2   2   // direction pin B
#define CONVEYOR_ENA  21   // PWM speed

// Intake motor
// NOTE: GPIO 0/1/3 are reserved (boot strap + UART0 TX/RX). Using them as
// motor pins kills Serial output. Remapped to safe GPIOs — rewire accordingly.
#define INTAKE_IN1   18  // direction pin A
#define INTAKE_IN2   19  // direction pin B
#define INTAKE_ENA   23  // PWM speed control

// ── Cherry limit switch (conveyor down-stop) ──────────────────
// SPDT microswitch wired NO + COM (NO -> GPIO with INPUT_PULLUP, COM -> GND).
// Reads LOW when the lever is pressed (hook at bottom of travel), HIGH when
// released. Press blocks the conveyor from moving further "backward" (down);
// "forward" (up) is always allowed so the hook can leave the switch.
#define CONVEYOR_LIMIT_PIN     36
#define CONVEYOR_LIMIT_PRESSED HIGH

// If a motor spins the wrong way, swap its IN1/IN2 defines above.

// ── PWM config ────────────────────────────────────────────────
#define PWM_FREQ       1000   // 1 kHz
#define PWM_RES        8      // 8-bit → 0-255
#define LEFT_CHANNEL     0
#define RIGHT_CHANNEL    1
#define CONVEYOR_CHANNEL 2
#define INTAKE_CHANNEL   3

// ── Speed presets ─────────────────────────────────────────────
// Lowered max speed to reduce wheel-spin / improve traction.
#define SPEED_FULL          210   // outer-wheel / straight-line speed (was 250)
#define SPEED_TURN           80   // inner-wheel speed on diagonal (arc) moves
#define SPEED_TURN_INPLACE  160   // (legacy — superseded by the in-place charge curve below)
#define CONVEYOR_SPEED      255   // conveyor PWM duty (0-255)

// ── In-place turn "hold-to-charge" curve ──────────────────────
// At a pivot both wheels fight static friction at once, so a fixed PWM gives
// a sluggish spin. Instead we ramp PWM up the longer the joystick is held in
// a turn direction:
//   • brief tap          → INPLACE_PWM_MIN    (gentle nudge)
//   • held INPLACE_CHARGE_MS → INPLACE_PWM_MAX (snappy spin, can exceed
//                                                straight-line max because
//                                                turning load is symmetric)
// Releasing the stick or switching to a different motion (forward/back/other
// direction) resets the charge so the next tap starts gentle again.
#define INPLACE_PWM_MIN       140   // initial PWM on first packet of a turn
#define INPLACE_PWM_MAX       240   // cap after holding for INPLACE_CHARGE_MS
#define INPLACE_CHARGE_MS     600   // time from MIN → MAX while held

// ── Acceleration ramp (traction control) ──────────────────────
// Drive motors ramp from 0 → target along a smoothstep curve (slow start,
// ease in, ease out) instead of slamming straight to full PWM. This prevents
// the wheels from breaking traction on the initial push. The ramp is reset
// whenever the joystick reverses direction or moves from rest, so each new
// "throw" of the stick gets a fresh controlled acceleration.
#define RAMP_TIME_MS    170    // time from 0 → full target along the curve
#define RAMP_UPDATE_MS   10    // how often loop() updates the motor PWM

// ── Joystick data ─────────────────────────────────────────────
typedef struct ControllerData {
  int   joyX;
  int   joyY;
  bool  btn1;   // conveyor DOWN — one-click toggle (auto-descent until limit)
  bool  btn2;   // conveyor UP   — manual hold (forward)
  bool  btn3;   // intake reverse (only when btn4 enables intake)
  bool  btn4;   // intake on/off switch
} ControllerData;

ControllerData incomingData;

#define DEAD_LOW   1200
#define DEAD_HIGH  2900

// Safety: stop motors if controller goes silent
volatile unsigned long lastReceiveTime = 0;
#define RECEIVE_TIMEOUT  250   // ms

// ── Motor helpers ─────────────────────────────────────────────
// Positive = forward, negative = backward, 0 = brake
void setLeftMotor(int speed) {
  if (speed > 0) {
    digitalWrite(LEFT_IN1, HIGH);
    digitalWrite(LEFT_IN2, LOW);
  } else if (speed < 0) {
    digitalWrite(LEFT_IN1, LOW);
    digitalWrite(LEFT_IN2, HIGH);
    speed = -speed;
  } else {
    digitalWrite(LEFT_IN1, LOW);
    digitalWrite(LEFT_IN2, LOW);
  }
  ledcWrite(LEFT_CHANNEL, constrain(speed, 0, 255));
}

void setRightMotor(int speed) {
  if (speed > 0) {
    digitalWrite(RIGHT_IN1, HIGH);
    digitalWrite(RIGHT_IN2, LOW);
  } else if (speed < 0) {
    digitalWrite(RIGHT_IN1, LOW);
    digitalWrite(RIGHT_IN2, HIGH);
    speed = -speed;
  } else {
    digitalWrite(RIGHT_IN1, LOW);
    digitalWrite(RIGHT_IN2, LOW);
  }
  ledcWrite(RIGHT_CHANNEL, constrain(speed, 0, 255));
}

// Conveyor run state — used by loop() limit-switch guard.
// 0 = stopped, +1 = forward (up), -1 = reverse (down)
volatile int conveyorRun = 0;

// One-click descent state. Set true on btn1 press edge; cleared on:
//   - second btn1 press (cancel)
//   - cherry limit pressed
//   - manual UP override (btn2 held)
//   - controller-timeout safety stop
volatile bool autoDescending = false;

static inline bool conveyorLimitPressed() {
  Serial.println(digitalRead(CONVEYOR_LIMIT_PIN));
  return digitalRead(CONVEYOR_LIMIT_PIN) == CONVEYOR_LIMIT_PRESSED;
}

void setConveyorMotor(int speed) {
  // Cherry switch temporarily disabled — limit no longer blocks downward motion.
  // if (speed < 0 && conveyorLimitPressed()) speed = 0;
  conveyorRun = (speed > 0) ? 1 : (speed < 0) ? -1 : 0;

  if (speed > 0) {
    digitalWrite(CONVEYOR_IN1, HIGH);
    digitalWrite(CONVEYOR_IN2, LOW);
  } else if (speed < 0) {
    digitalWrite(CONVEYOR_IN1, LOW);
    digitalWrite(CONVEYOR_IN2, HIGH);
    speed = -speed;
  } else {
    digitalWrite(CONVEYOR_IN1, LOW);
    digitalWrite(CONVEYOR_IN2, LOW);
  }
  ledcWrite(CONVEYOR_CHANNEL, constrain(speed, 0, 255));
}
void setIntakeMotor(int speed) {
  if (speed > 0) {
    digitalWrite(INTAKE_IN1, HIGH);
    digitalWrite(INTAKE_IN2, LOW);
  } else if (speed < 0) {
    digitalWrite(INTAKE_IN1, LOW);
    digitalWrite(INTAKE_IN2, HIGH);
    speed = -speed;
  } else {
    digitalWrite(INTAKE_IN1, LOW);
    digitalWrite(INTAKE_IN2, LOW);
  }
  ledcWrite(INTAKE_CHANNEL, constrain(speed, 0, 255));
}

void stopMotors() {
  setLeftMotor(0);
  setRightMotor(0);
  setConveyorMotor(0);
  setIntakeMotor(0);
}

// ── Drive motor ramp state ────────────────────────────────────
// Targets are written by the ESP-NOW callback; the actual PWM applied to
// each motor is updated in loop() so the ramp runs at a steady rate even if
// packets arrive late or in bursts.
struct MotorRamp {
  int           target;     // most-recent target speed (signed)
  int           current;    // currently commanded PWM (signed)
  int           rampSign;   // sign of the active ramp (+1 / -1 / 0)
  unsigned long startMs;    // millis() when this ramp started
};

volatile int  driveTargetLeft  = 0;
volatile int  driveTargetRight = 0;
// When true, the next ramp tick snaps to the target instead of curving up.
// Set by the callback for in-place turns — static friction at a pivot is too
// high for the smooth-launch ramp to overcome without slipping.
volatile bool driveInstant     = false;
static MotorRamp leftRamp  = {0, 0, 0, 0};
static MotorRamp rightRamp = {0, 0, 0, 0};

static inline int signOf(int v) { return (v > 0) - (v < 0); }

// Smoothstep curve: f(t) = 3t² − 2t³ for t in [0,1].
// Slow start, accelerates through the middle, eases into the target.
static inline float smoothstep(float t) {
  if (t <= 0.0f) return 0.0f;
  if (t >= 1.0f) return 1.0f;
  return t * t * (3.0f - 2.0f * t);
}

// Advance one motor's ramp toward `target` and return the PWM to write.
// On direction change (or restart from rest) the ramp restarts from 0 so the
// new motion always begins gently.
static int stepRamp(MotorRamp &r, int target, unsigned long now, bool instant) {
  int newSign = signOf(target);

  if (newSign != r.rampSign) {
    // Direction changed (incl. → 0 or 0 → motion). Snap to 0 and restart.
    r.current  = 0;
    r.rampSign = newSign;
    r.startMs  = now;
  }
  r.target = target;

  if (target == 0) {
    r.current = 0;
    return 0;
  }

  if (instant) {
    // Bypass the curve — punch through static friction (used for in-place turns).
    // Fast-forward startMs so a subsequent non-instant tick continues from full
    // rather than re-curving up from zero.
    r.startMs = now - RAMP_TIME_MS;
    r.current = target;
    return r.current;
  }

  unsigned long elapsed = now - r.startMs;
  float t     = (float)elapsed / (float)RAMP_TIME_MS;
  float curve = smoothstep(t);
  int   absT  = abs(target);
  r.current   = newSign * (int)(absT * curve);
  return r.current;
}

static void resetDriveRamps() {
  driveTargetLeft  = 0;
  driveTargetRight = 0;
  driveInstant     = false;
  leftRamp  = {0, 0, 0, 0};
  rightRamp = {0, 0, 0, 0};
  setLeftMotor(0);
  setRightMotor(0);
}

// ── ESP-NOW callback ──────────────────────────────────────────
void onDataReceived(const uint8_t *mac_addr, const uint8_t *data, int len) {
  if (len != sizeof(incomingData)) return;   // ignore malformed packets
  memcpy(&incomingData, data, sizeof(incomingData));
  lastReceiveTime = millis();

  bool goForward  = (incomingData.joyX > DEAD_HIGH);
  bool goBackward = (incomingData.joyX < DEAD_LOW);
  bool turnLeft   = (incomingData.joyY < DEAD_LOW);
  bool turnRight  = (incomingData.joyY > DEAD_HIGH);

  int leftSpeed  = 0;
  int rightSpeed = 0;

  // ── In-place turn charge: track how long the stick has been held in
  // pure left/right and grow PWM along an exponential curve.
  // 1 - e^(-k·t) saturates at INPLACE_PWM_MAX; k=3 reaches ~95% at full charge.
  bool inPlaceTurn = (turnLeft || turnRight) && !goForward && !goBackward;
  static unsigned long inPlaceStartMs = 0;
  static int           inPlaceDir     = 0;   // -1 left, +1 right, 0 none
  unsigned long nowCb = millis();
  int dirNow = inPlaceTurn ? (turnLeft ? -1 : +1) : 0;
  if (dirNow != inPlaceDir) {
    inPlaceDir     = dirNow;
    inPlaceStartMs = nowCb;
  }
  int inPlacePwm = 0;
  if (dirNow != 0) {
    float t = (float)(nowCb - inPlaceStartMs) / (float)INPLACE_CHARGE_MS;
    if (t > 1.0f) t = 1.0f;
    float charge = 1.0f - expf(-3.0f * t);   // exponential saturating curve
    int range = INPLACE_PWM_MAX - INPLACE_PWM_MIN;
    inPlacePwm  = INPLACE_PWM_MIN + (int)(range * charge);
  }

  if (goForward && turnLeft)        { leftSpeed =  SPEED_TURN; rightSpeed =  SPEED_FULL; }
  else if (goForward && turnRight)  { leftSpeed =  SPEED_FULL; rightSpeed =  SPEED_TURN; }
  else if (goBackward && turnLeft)  { leftSpeed = -SPEED_TURN; rightSpeed = -SPEED_FULL; }
  else if (goBackward && turnRight) { leftSpeed = -SPEED_FULL; rightSpeed = -SPEED_TURN; }
  else if (goForward)               { leftSpeed =  SPEED_FULL; rightSpeed =  SPEED_FULL; }
  else if (goBackward)              { leftSpeed = -SPEED_FULL; rightSpeed = -SPEED_FULL; }
  else if (turnLeft)                { leftSpeed = -inPlacePwm; rightSpeed =  inPlacePwm; }
  else if (turnRight)               { leftSpeed =  inPlacePwm; rightSpeed = -inPlacePwm; }

  // Hand targets to the ramp; loop() drives the actual PWM along a smooth curve.
  // In-place turn bypasses the smoothstep ramp because the charge curve is
  // already doing its own (more aggressive) ramp tailored for breaking pivot
  // friction — letting the smoothstep run on top would attenuate the launch.
  driveTargetLeft  = leftSpeed;
  driveTargetRight = rightSpeed;
  driveInstant     = inPlaceTurn;

  // Conveyor mapping (swapped from prior version — buttons were inverted on the controller):
  //   btn2 = manual UP (held, forward)
  //   btn1 = one-click DOWN (toggle: press starts auto-descent, press again cancels,
  //          cherry limit auto-stops). Edge-detected + debounced here in the receiver.
  // The existing convention in this file treats `!incomingData.btnN` as "active".
  bool upActive   = !incomingData.btn2;
  bool downActive = !incomingData.btn1;

  static bool          prevDownActive   = false;
  static unsigned long lastDownEdgeMs   = 0;
  const  unsigned long DOWN_DEBOUNCE_MS = 50;

  unsigned long nowMs = millis();
  if (downActive && !prevDownActive && (nowMs - lastDownEdgeMs > DOWN_DEBOUNCE_MS)) {
    lastDownEdgeMs = nowMs;
    autoDescending = !autoDescending;
    Serial.printf("[DESCENT] Toggled %s\n", autoDescending ? "ON" : "OFF");
  }
  prevDownActive = downActive;

  int conveyorSpeed = 0;
  if (upActive) {
    // Manual UP overrides and cancels any auto-descent in progress.
    if (autoDescending) Serial.println("[DESCENT] Cancelled by manual UP");
    autoDescending = false;
    conveyorSpeed  = CONVEYOR_SPEED;
  } else if (autoDescending) {
    if (conveyorLimitPressed()) {
      Serial.println("[DESCENT] Limit reached — auto-stop");
      autoDescending = false;
      conveyorSpeed  = 0;
    } else {
      conveyorSpeed = -CONVEYOR_SPEED;
    }
  }
  setConveyorMotor(conveyorSpeed);

  // Intake: btn4 (switch) gates power; btn3 reverses direction while gated on.
  if (incomingData.btn4) {
    setIntakeMotor(incomingData.btn3 ? -255 : 255);
  } else {
    setIntakeMotor(0);
  }

  // Debug label for the button state
  String btnCmd = "NONE";
  if      (conveyorSpeed > 0)      btnCmd = "CONV-FWD";
  else if (conveyorSpeed < 0)      btnCmd = "CONV-REV";
  else if (incomingData.btn4 && incomingData.btn3) btnCmd = "INTAKE-REV";
  else if (incomingData.btn4)      btnCmd = "INTAKE-ON";

  // Debug
  String moveCmd = "STOP";
  if (goForward && turnLeft)        moveCmd = "FWD-LEFT";
  else if (goForward && turnRight)  moveCmd = "FWD-RIGHT";
  else if (goBackward && turnLeft)  moveCmd = "BACK-LEFT";
  else if (goBackward && turnRight) moveCmd = "BACK-RIGHT";
  else if (goForward)               moveCmd = "FORWARD";
  else if (goBackward)              moveCmd = "BACKWARD";
  else if (turnLeft)                moveCmd = "TURN-LEFT";
  else if (turnRight)               moveCmd = "TURN-RIGHT";

  int cherryRaw = digitalRead(CONVEYOR_LIMIT_PIN);
  Serial.printf("%-10s | %-10s | L=%4d R=%4d | CHERRY raw=%d pressed=%d | X=%d Y=%d | b1=%d b2=%d b3=%d b4=%d\n",
                moveCmd.c_str(), btnCmd.c_str(),
                leftSpeed, rightSpeed,
                cherryRaw, cherryRaw == CONVEYOR_LIMIT_PRESSED ? 1 : 0,
                incomingData.joyX, incomingData.joyY,
                incomingData.btn1, incomingData.btn2,
                incomingData.btn3, incomingData.btn4);
}

// ── Setup ─────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  Serial.println("Receiver starting up...");
  WiFi.mode(WIFI_STA);
  Serial.println("Test 1");

  // Direction pins
  pinMode(LEFT_IN1,  OUTPUT);
  Serial.println("Test 2");

  pinMode(LEFT_IN2,  OUTPUT);
  Serial.println("Test 3");

  pinMode(RIGHT_IN1, OUTPUT);
  pinMode(RIGHT_IN2, OUTPUT);
  pinMode(CONVEYOR_IN1, OUTPUT);
  pinMode(CONVEYOR_IN2, OUTPUT);
  pinMode(CONVEYOR_LIMIT_PIN, INPUT_PULLUP);
  pinMode(INTAKE_IN1, OUTPUT);
  pinMode(INTAKE_IN2, OUTPUT);

  // PWM on enable pins
  ledcSetup(LEFT_CHANNEL,  PWM_FREQ, PWM_RES);
  ledcAttachPin(LEFT_ENA,  LEFT_CHANNEL);
  ledcSetup(RIGHT_CHANNEL, PWM_FREQ, PWM_RES);
  ledcAttachPin(RIGHT_ENB, RIGHT_CHANNEL);
  ledcSetup(CONVEYOR_CHANNEL, PWM_FREQ, PWM_RES);
  ledcAttachPin(CONVEYOR_ENA, CONVEYOR_CHANNEL);
  ledcSetup(INTAKE_CHANNEL, PWM_FREQ, PWM_RES);
  ledcAttachPin(INTAKE_ENA, INTAKE_CHANNEL);

  Serial.println("Test 4");

  stopMotors();
  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed!");
    return;
  }
  esp_now_register_recv_cb(onDataReceived);
  Serial.println("Receiver ready. Waiting for controller...");
}

void loop() {
  unsigned long now = millis();

  // Kill motors if controller signal is lost
  if (lastReceiveTime > 0 && now - lastReceiveTime > RECEIVE_TIMEOUT) {
    autoDescending = false;
    resetDriveRamps();
    setConveyorMotor(0);
    setIntakeMotor(0);
  } else {
    // Advance the drive-motor ramp at a fixed rate so PWM updates are smooth
    // regardless of ESP-NOW packet jitter.
    static unsigned long lastRampMs = 0;
    if (now - lastRampMs >= RAMP_UPDATE_MS) {
      lastRampMs = now;
      int l = stepRamp(leftRamp,  driveTargetLeft,  now, driveInstant);
      int r = stepRamp(rightRamp, driveTargetRight, now, driveInstant);
      setLeftMotor(l);
      setRightMotor(r);
    }
  }

  // Heartbeat: report when no packets are arriving so it's obvious whether
  // Serial is dead or the controller is silent.
  static unsigned long lastHeartbeat = 0;
  if (millis() - lastHeartbeat >= 1000) {
    lastHeartbeat = millis();
    if (lastReceiveTime == 0) {
      Serial.println("No data received yet (waiting for first packet from controller)");
    } else if (millis() - lastReceiveTime > RECEIVE_TIMEOUT) {
      Serial.printf("No data received for %lu ms (controller silent / out of range)\n",
                    millis() - lastReceiveTime);
    }
  }

  // Edge-detect the cherry limit switch so we log every transition.
  // Cherry switch logic is disabled — this is log-only, no motor effect.
  static int lastLimitState = -1;   // -1 = uninitialized
  int limitState = conveyorLimitPressed() ? 1 : 0;
  if (limitState != lastLimitState) {
    if (limitState) {
      Serial.println("[LIMIT] Cherry switch CLOSED (pressed) — logging only, no effect");
    } else if (lastLimitState != -1) {
      Serial.println("[LIMIT] Cherry switch OPEN (released) — logging only, no effect");
    }
    lastLimitState = limitState;
  }

  // Mid-stroke safety: if descent is active and the cherry closes between packets,
  // stop immediately rather than waiting for the next ESP-NOW packet (~20ms).
  if (conveyorRun < 0 && conveyorLimitPressed()) {
    Serial.println("[LIMIT] Down-motion stopped mid-stroke by cherry switch");
    autoDescending = false;
    setConveyorMotor(0);
  }
}
