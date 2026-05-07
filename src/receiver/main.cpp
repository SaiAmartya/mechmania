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

// ── Conveyor: A4988 stepstick (stepper driver) ────────────────
// Only STEP + DIR are driven from the MCU. See wiring notes at bottom of file.
#define CONVEYOR_STEP 13   // STEP pulse to A4988 (reused from old conveyor IN2)
#define CONVEYOR_DIR  2   // DIR to A4988 (NEW pin — wire this)

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
#define CONVEYOR_LIMIT_PIN     27
#define CONVEYOR_LIMIT_PRESSED LOW

// If a motor spins the wrong way, swap its IN1/IN2 defines above.

// ── PWM config ────────────────────────────────────────────────
#define PWM_FREQ       1000   // 1 kHz
#define PWM_RES        8      // 8-bit → 0-255
#define LEFT_CHANNEL   0
#define RIGHT_CHANNEL  1
#define INTAKE_CHANNEL   3

// ── Speed presets ─────────────────────────────────────────────
#define SPEED_FULL    250    // mid speed for better control
#define SPEED_TURN    70     // inner-wheel speed on diagonal moves

// ── Conveyor stepper config ───────────────────────────────────
// Step interval = half-period of the STEP square wave. Smaller = faster.
// 600 µs → ~833 steps/s (~5 rev/s on a 200-step NEMA17 in full-step).
// Increase if motor stalls, decrease for more speed.
#define CONVEYOR_STEP_HALF_US  600
// Direction sign — flip if conveyor runs backwards.
#define CONVEYOR_DIR_FORWARD   HIGH

// ── Joystick data ─────────────────────────────────────────────
typedef struct ControllerData {
  int   joyX;
  int   joyY;
  bool  btn1;   // conveyor forward
  bool  btn2;   // conveyor backward
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

// Conveyor stepper state — driven non-blockingly from loop()
// 0 = stopped, +1 = forward, -1 = reverse
volatile int      conveyorRun         = 0;
volatile bool     conveyorStepLevel   = false;
volatile uint32_t conveyorLastStepUs  = 0;

static inline bool conveyorLimitPressed() {
  return digitalRead(CONVEYOR_LIMIT_PIN) == CONVEYOR_LIMIT_PRESSED;
}

void setConveyorStepper(int dir) {
  // Honor the bottom-of-travel limit: block downward motion when pressed.
  // Upward motion is always allowed so the hook can leave the switch.
  if (dir < 0 && conveyorLimitPressed()) dir = 0;
  conveyorRun = (dir > 0) ? 1 : (dir < 0) ? -1 : 0;
  if (conveyorRun == 0) {
    digitalWrite(CONVEYOR_STEP, LOW);
    conveyorStepLevel = false;
  } else {
    digitalWrite(CONVEYOR_DIR,
                 (conveyorRun > 0) ? CONVEYOR_DIR_FORWARD
                                   : !CONVEYOR_DIR_FORWARD);
  }
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
  setConveyorStepper(0);
  setIntakeMotor(0);
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

  if (goForward && turnLeft)        { leftSpeed =  SPEED_TURN; rightSpeed =  SPEED_FULL; }
  else if (goForward && turnRight)  { leftSpeed =  SPEED_FULL; rightSpeed =  SPEED_TURN; }
  else if (goBackward && turnLeft)  { leftSpeed = -SPEED_TURN; rightSpeed = -SPEED_FULL; }
  else if (goBackward && turnRight) { leftSpeed = -SPEED_FULL; rightSpeed = -SPEED_TURN; }
  else if (goForward)               { leftSpeed =  SPEED_FULL; rightSpeed =  SPEED_FULL; }
  else if (goBackward)              { leftSpeed = -SPEED_FULL; rightSpeed = -SPEED_FULL; }
  else if (turnLeft)                { leftSpeed = -SPEED_FULL; rightSpeed =  SPEED_FULL; }
  else if (turnRight)               { leftSpeed =  SPEED_FULL; rightSpeed = -SPEED_FULL; }

  setLeftMotor(leftSpeed);
  setRightMotor(rightSpeed);

  // Conveyor: btn1 = forward, btn2 = backward.
  // If both pressed at once, prefer forward and ignore backward (safer than oscillating).
  int conveyorDir = 0;
  if      (incomingData.btn1) conveyorDir =  1;
  else if (incomingData.btn2) conveyorDir = -1;
  setConveyorStepper(conveyorDir);

  // Intake: btn4 (switch) gates power; btn3 reverses direction while gated on.
  if (incomingData.btn4) {
    setIntakeMotor(incomingData.btn3 ? -255 : 255);
  } else {
    setIntakeMotor(0);
  }

  // Debug label for the button state
  String btnCmd = "NONE";
  if      (conveyorDir > 0)        btnCmd = "CONV-FWD";
  else if (conveyorDir < 0)        btnCmd = "CONV-REV";
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

  Serial.printf("%-10s | %-10s | L=%4d R=%4d | LIM=%d | X=%d Y=%d | b1=%d b2=%d b3=%d b4=%d\n",
                moveCmd.c_str(), btnCmd.c_str(),
                leftSpeed, rightSpeed,
                conveyorLimitPressed() ? 1 : 0,
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
  pinMode(CONVEYOR_STEP, OUTPUT);
  pinMode(CONVEYOR_DIR,  OUTPUT);
  digitalWrite(CONVEYOR_STEP, LOW);
  digitalWrite(CONVEYOR_DIR,  CONVEYOR_DIR_FORWARD);
  pinMode(CONVEYOR_LIMIT_PIN, INPUT_PULLUP);
  pinMode(INTAKE_IN1, OUTPUT);
  pinMode(INTAKE_IN2, OUTPUT);

  // PWM on enable pins
  ledcSetup(LEFT_CHANNEL,  PWM_FREQ, PWM_RES);
  ledcAttachPin(LEFT_ENA,  LEFT_CHANNEL);
  ledcSetup(RIGHT_CHANNEL, PWM_FREQ, PWM_RES);
  ledcAttachPin(RIGHT_ENB, RIGHT_CHANNEL);
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
  // Kill motors if controller signal is lost
  if (lastReceiveTime > 0 && millis() - lastReceiveTime > RECEIVE_TIMEOUT) {
    stopMotors();
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

  // Stop mid-stroke if the bottom limit trips between ESP-NOW packets.
  // Without this, the conveyor would keep going down for up to RECEIVE_TIMEOUT
  // (or until the next packet) after the switch is hit.
  if (conveyorRun < 0 && conveyorLimitPressed()) {
    setConveyorStepper(0);
  }

  // Non-blocking STEP pulse generator for the A4988-driven conveyor.
  // Toggles CONVEYOR_STEP every CONVEYOR_STEP_HALF_US microseconds while running.
  if (conveyorRun != 0) {
    uint32_t now = micros();
    if ((uint32_t)(now - conveyorLastStepUs) >= CONVEYOR_STEP_HALF_US) {
      conveyorLastStepUs = now;
      conveyorStepLevel = !conveyorStepLevel;
      digitalWrite(CONVEYOR_STEP, conveyorStepLevel ? HIGH : LOW);
    }
  }
}
