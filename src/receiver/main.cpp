#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>

// This runs on the RECEIVER ESP32

// ── Motor Driver Pins (Cytron MDD3A) ────────────────────────
// Left motor
#define LEFT_IN1   25   // M1A
#define LEFT_IN2   26   // M1B

// Right motor
#define RIGHT_IN1  17   // M2A
#define RIGHT_IN2  16   // M2B

// If a motor spins the wrong way, swap its IN1/IN2 defines above.

// ── PWM config ────────────────────────────────────────────────
#define PWM_FREQ       1000   // 1 kHz
#define PWM_RES        8      // 8-bit → 0-255
#define LEFT_CH1       0
#define LEFT_CH2       1
#define RIGHT_CH1      2
#define RIGHT_CH2      3

// ── Speed presets ─────────────────────────────────────────────
#define SPEED_FULL  255
#define SPEED_TURN  180   // inner-wheel speed on diagonal moves

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
    ledcWrite(LEFT_CH1, constrain(speed, 0, 255));
    ledcWrite(LEFT_CH2, 0);
  } else if (speed < 0) {
    ledcWrite(LEFT_CH1, 0);
    ledcWrite(LEFT_CH2, constrain(-speed, 0, 255));
  } else {
    ledcWrite(LEFT_CH1, 0);
    ledcWrite(LEFT_CH2, 0);
  }
}

void setRightMotor(int speed) {
  if (speed > 0) {
    ledcWrite(RIGHT_CH1, constrain(speed, 0, 255));
    ledcWrite(RIGHT_CH2, 0);
  } else if (speed < 0) {
    ledcWrite(RIGHT_CH1, 0);
    ledcWrite(RIGHT_CH2, constrain(-speed, 0, 255));
  } else {
    ledcWrite(RIGHT_CH1, 0);
    ledcWrite(RIGHT_CH2, 0);
  }
}

void stopMotors() {
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

  // Button commands (for future actuator use)
  String btnCmd = "NONE";
  if (incomingData.btn1)      btnCmd = "RAISE";
  else if (incomingData.btn2) btnCmd = "LOWER";

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

  // PWM on direction pins for MDD3A
  ledcSetup(LEFT_CH1,  PWM_FREQ, PWM_RES);
  ledcAttachPin(LEFT_IN1, LEFT_CH1);
  
  ledcSetup(LEFT_CH2,  PWM_FREQ, PWM_RES);
  ledcAttachPin(LEFT_IN2, LEFT_CH2);
  
  ledcSetup(RIGHT_CH1, PWM_FREQ, PWM_RES);
  ledcAttachPin(RIGHT_IN1, RIGHT_CH1);
  
  ledcSetup(RIGHT_CH2, PWM_FREQ, PWM_RES);
  ledcAttachPin(RIGHT_IN2, RIGHT_CH2);

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
