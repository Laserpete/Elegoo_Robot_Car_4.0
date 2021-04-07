#include <Arduino.h>

#include "IRremote.h"

#define IR_RECEIVE_PIN 9

// Moving forwards
#define CALIBRATION_TIME 2
#define CALIBRATION_AT_PWM_100 75
#define CALIBRATION_AT_PWM_255 188

// Turning on the spot
#define TURN_LR_PWM 125
// The gradient determined by the turning on the spot linear regression
// PWM = 125
// f(x) == 248 x - 40
// f(x) + 40 == 248 x
// f(x) + 40 / 248 == x
// f(x) = a x -b
#define TURN_SPOT_F_OF_X_A 0.248
#define TURN_SPOT_F_OF_X_B 40

// Driving forward and back trim
#define STEERING_TRIM_VAL 20
#define MILLISECONDS 1000

int AIN1 = 8;  // direction low = forward
int AIN2 = 5;  // speed PWM
int BIN1 = 7;  // direction high = forward
int BIN2 = 6;  // speed PWM

int leftSpeed;
int rightSpeed;
int wheelPWM;
float CMpS;

// Use calibration definitions to calculate gradient for PWM : CM per second
float speed100 = CALIBRATION_AT_PWM_100 / CALIBRATION_TIME;  // speed at PWM 100
float speed225 = CALIBRATION_AT_PWM_255 / CALIBRATION_TIME;  // speed at PWM 255
float m = (speed225 - speed100) / 155;  // (speed225 - speed100) / 255 - 100

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    ;
  }
  Serial.println("IR Receiver Begin");
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  Serial.println(F("START " __FILE__ " from "__DATE__
                   "\r\nUsing library version " VERSION_IRREMOTE));
  IrReceiver.begin(IR_RECEIVE_PIN, false);
}

int calcWheelPWM(float desiredVelocity) {
  int calculatedPWM = desiredVelocity / m + (m * 100) - speed100;
  return calculatedPWM;
}

void setSpeed(int leftVal, int rightVal) {
  analogWrite(AIN2, leftVal);
  analogWrite(BIN2, rightVal);
}

int calculateRotationTime(int angle) {
  int rotationTime = (angle + TURN_SPOT_F_OF_X_B) / TURN_SPOT_F_OF_X_A;
  Serial.print("Rotation time for angle ");
  Serial.print(angle);
  Serial.print(" is ");
  Serial.print(rotationTime);
  return rotationTime;
}

void forward(int d, float velocity) {
  // Set wheel turning direction
  digitalWrite(AIN1, LOW);
  digitalWrite(BIN1, HIGH);

  // Calculate wheel PWM value and et steering trim

  leftSpeed = rightSpeed = calcWheelPWM(velocity);
  rightSpeed = leftSpeed - STEERING_TRIM_VAL;

  // Set wheel PWM values
  analogWrite(AIN2, leftSpeed);
  analogWrite(BIN2, rightSpeed);

  // Calculate time to drive
  float t = d / velocity;
  delay(t * 1000);
}

void backward(int d, float velocity) {
  digitalWrite(AIN1, HIGH);
  digitalWrite(BIN1, LOW);

  // Calculate wheel PWM value and et steering trim
  leftSpeed = rightSpeed = calcWheelPWM(velocity);
  rightSpeed = leftSpeed + STEERING_TRIM_VAL;

  // Set wheel PWM values
  analogWrite(AIN2, leftSpeed);
  analogWrite(BIN2, rightSpeed);

  // Calculate time to drive
  float t = d / velocity;
  delay(t * MILLISECONDS);
}

void stopCar(int d) {
  digitalWrite(AIN2, LOW);
  digitalWrite(BIN2, LOW);
  delay(d);
}
void turnLeft(int angle) {
  stopCar(500);
  digitalWrite(AIN1, LOW);
  digitalWrite(BIN1, LOW);
  analogWrite(AIN2, TURN_LR_PWM);
  analogWrite(BIN2, TURN_LR_PWM);
  float rotationTime = calculateRotationTime(angle);
  delay(rotationTime);
}

void turnRight(int angle) {
  stopCar(500);
  digitalWrite(AIN1, HIGH);
  digitalWrite(BIN1, HIGH);
  analogWrite(AIN2, TURN_LR_PWM);
  analogWrite(BIN2, TURN_LR_PWM);
  float rotationTime = calculateRotationTime(angle);
  delay(rotationTime);
}

void calF() {
  digitalWrite(AIN1, LOW);
  digitalWrite(BIN1, HIGH);
  delay(CALIBRATION_TIME * MILLISECONDS);
}

void calB() {
  digitalWrite(AIN1, HIGH);
  digitalWrite(BIN1, LOW);
  delay(5000);
}

void calL(float d) {
  digitalWrite(AIN1, LOW);
  analogWrite(AIN2, 125);
  digitalWrite(BIN1, LOW);
  analogWrite(BIN2, 125);
  delay(5000);
}

void calR() {
  digitalWrite(AIN1, HIGH);
  digitalWrite(BIN1, HIGH);
  delay(5000);
}

int calculateMaximumSpeed(int CMpS) {
  float maxCMpS = (m * 255) - (m * 100) + speed100;
  if (CMpS >= maxCMpS) {
    CMpS = maxCMpS;
    Serial.print("Max velocity = ");
    Serial.print(maxCMpS);
    Serial.println("CM per Second");
  }
  return maxCMpS;
}

void loop() {
  // Serial.println("This is working");

  if (IrReceiver.decode()) {
    IrReceiver.printIRResultShort(&Serial);
    Serial.println();
    IrReceiver.resume();

    if (IrReceiver.decodedIRData.command == 0x10) {
      Serial.println("Do something");
    } else if (IrReceiver.decodedIRData.command == 0x11) {
      Serial.println("Do something else");
    }
  }
  delay(500);

  /*
  leftSpeed = 100;
  rightSpeed = 100;

  // Calculate wv from left+right speeds
  wv = (((float)leftSpeed+(float)rightSpeed)/2);
  Serial.print("wv =  ");
  Serial.println(wv);
 */
  // Calculate CM per second (v) for given PWM value (wv)
  // v - v1 = m(wv-wv1)
  // CMpS - speed100 = m(wv - 100)
  // CMpS = m(wv - 100) + speed100

  CMpS = 50;

  CMpS = calculateMaximumSpeed(CMpS);
  wheelPWM = calcWheelPWM(CMpS);
}
/*
  forward(50, 50);
  turnRight(180);
  forward(50, 25);
  turnLeft(180);
  stopCar(1);
  while (1)
    ;
}*/