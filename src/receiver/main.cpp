#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>

// This runs on the RECEIVER ESP32

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

// Intake motor (no enable pin — runs at full speed whenever powered)
#define INTAKE_IN1   0   // direction pin A
#define INTAKE_IN2   1   // direction pin B
#define INTAKE_ENA   3   // PWM speed control



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
  bool  btn1;
  bool  btn2;
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

void setConveyorStepper(int dir) {
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

  // Button commands
  String btnCmd = "NONE";
  if (incomingData.btn1)      btnCmd = "RAISE";
  else if (incomingData.btn2) btnCmd = "REVERSE";

  // btn1 runs the conveyor forward; release stops it.
  setConveyorStepper(incomingData.btn1 ? 1 : 0);

  // Intake runs continuously at 100% (255/255); btn2 reverses its direction.
  setIntakeMotor(incomingData.btn2 ? -255 : 255);

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

  Serial.printf("%-10s | %-5s | L=%4d R=%4d | X=%d Y=%d\n",
                moveCmd.c_str(), btnCmd.c_str(),
                leftSpeed, rightSpeed,
                incomingData.joyX, incomingData.joyY);
}

// ── Setup ─────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);

  // Direction pins
  pinMode(LEFT_IN1,  OUTPUT);
  pinMode(LEFT_IN2,  OUTPUT);
  pinMode(RIGHT_IN1, OUTPUT);
  pinMode(RIGHT_IN2, OUTPUT);
  pinMode(CONVEYOR_STEP, OUTPUT);
  pinMode(CONVEYOR_DIR,  OUTPUT);
  digitalWrite(CONVEYOR_STEP, LOW);
  digitalWrite(CONVEYOR_DIR,  CONVEYOR_DIR_FORWARD);
  pinMode(INTAKE_IN1, OUTPUT);
  pinMode(INTAKE_IN2, OUTPUT);

  // PWM on enable pins
  ledcSetup(LEFT_CHANNEL,  PWM_FREQ, PWM_RES);
  ledcAttachPin(LEFT_ENA,  LEFT_CHANNEL);
  ledcSetup(RIGHT_CHANNEL, PWM_FREQ, PWM_RES);
  ledcAttachPin(RIGHT_ENB, RIGHT_CHANNEL);
  ledcSetup(INTAKE_CHANNEL, PWM_FREQ, PWM_RES);
  ledcAttachPin(INTAKE_ENA, INTAKE_CHANNEL);

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

  Serial.println("Receiver loop running..."); // Debug: confirm loop is active
  
  // Serial output ALL raw data
  Serial.printf("Raw Data | X=%d Y=%d btn1=%d btn2=%d\n",
                incomingData.joyX, incomingData.joyY,
                incomingData.btn1, incomingData.btn2);

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
