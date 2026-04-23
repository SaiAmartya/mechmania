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

// ── Motor Driver 2 Pins (L298N / similar H-bridge) ──────────────
// Conveyor motor
// Define 3 pins here
#define CONVEYOR_IN1  2 // D9  – direction pin A
#define CONVEYOR_IN2  13  // D7 – direction pin B
#define CONVEYOR_ENA  12  // D13 – PWM speed

// If a motor spins the wrong way, swap its IN1/IN2 defines above.

// ── PWM config ────────────────────────────────────────────────
#define PWM_FREQ       1000   // 1 kHz
#define PWM_RES        8      // 8-bit → 0-255
#define LEFT_CHANNEL   0
#define RIGHT_CHANNEL  1
#define CONVEYOR_CHANNEL 2

// ── Speed presets ─────────────────────────────────────────────
#define SPEED_FULL    250    // mid speed for better control
#define SPEED_TURN    70     // inner-wheel speed on diagonal moves
#define SPEED_CONVEYOR  150    // moderate conveyor speed

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

void setConveyorMotor(int speed) {
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

void stopMotors() {
  setLeftMotor(0);
  setRightMotor(0);
  setConveyorMotor(0);
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
  else if (incomingData.btn2) btnCmd = "LOWER";

  setConveyorMotor(incomingData.btn1 ? SPEED_CONVEYOR : 0);

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
  pinMode(CONVEYOR_IN1, OUTPUT);
  pinMode(CONVEYOR_IN2, OUTPUT);

  // PWM on enable pins
  ledcSetup(LEFT_CHANNEL,  PWM_FREQ, PWM_RES);
  ledcAttachPin(LEFT_ENA,  LEFT_CHANNEL);
  ledcSetup(RIGHT_CHANNEL, PWM_FREQ, PWM_RES);
  ledcAttachPin(RIGHT_ENB, RIGHT_CHANNEL);
  ledcSetup(CONVEYOR_CHANNEL, PWM_FREQ, PWM_RES);
  ledcAttachPin(CONVEYOR_ENA, CONVEYOR_CHANNEL);

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
  delay(10);
}
