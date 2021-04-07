#include <Arduino.h>

#include "FastLED.h"
#include "IRremote.h"
#include "Setup.h"

int AIN1 = 8;  // direction low = forward
int AIN2 = 5;  // speed PWM
int BIN1 = 7;  // direction high = forward
int BIN2 = 6;  // speed PWM

int leftSpeed;
int rightSpeed;
int wheelPWM;
float CMpS;

// Variables for the adjustable IR controlled speed/distance
int IrDriveDistance, IrDriveSpeed = 50;

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
                   "\r\nUsing IR Remote library version " VERSION_IRREMOTE));
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
  rightSpeed = leftSpeed - FORWARD_STEERING_TRIM;

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
  rightSpeed = leftSpeed + BACKWARD_STEERING_TRIM;

  // Set wheel PWM values
  analogWrite(AIN2, leftSpeed);
  analogWrite(BIN2, rightSpeed);

  // Calculate time to drive
  float t = d / velocity;
  delay(t * MILLISECONDS);
}

void stopCar() {
  digitalWrite(AIN2, LOW);
  digitalWrite(BIN2, LOW);
}
void turnLeft(int angle) {
  stopCar();
  digitalWrite(AIN1, LOW);
  digitalWrite(BIN1, LOW);
  analogWrite(AIN2, TURN_LR_PWM);
  analogWrite(BIN2, TURN_LR_PWM);
  float rotationTime = calculateRotationTime(angle);
  delay(rotationTime);
}

void turnRight(int angle) {
  stopCar();
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

void IrControlInterpreter() {
  IrReceiver.printIRResultShort(&Serial);
  int IrCommand = IrReceiver.decodedIRData.command;

  switch (IrCommand) {
    case IR_FORWARD:
      forward(IrDriveDistance, IrDriveSpeed);
      Serial.println(F("IR_FORWARD"));
      stopCar();
      break;

    case IR_BACKWARD:
      backward(IrDriveDistance, IrDriveSpeed);
      Serial.println(F("IR_BACKWARD"));
      stopCar();
      break;

    case IR_LEFT:
      turnLeft(IR_TURN_MAGNITUDE_DEGREES);
      stopCar();
      break;

    case IR_RIGHT:
      turnRight(IR_TURN_MAGNITUDE_DEGREES);
      stopCar();
      break;

    case IR_ASTERISK:
      Serial.println("IR_ASTERISK");
      delay(250);
      IrReceiver.resume();
      while (IrReceiver.decode() == 0) {
        ;
      }
      IrCommand = IrReceiver.decodedIRData.command;
      switch (IrCommand) {
        case IR_0:
          IrDriveSpeed = IR_DRIVE_SPEED_INCREMENT;
          Serial.println("IR_0");
          break;
        case IR_1:
          IrDriveSpeed = 2 * IR_DRIVE_SPEED_INCREMENT;
          break;
        case IR_2:
          IrDriveSpeed = 3 * IR_DRIVE_SPEED_INCREMENT;
          break;
        case IR_3:
          IrDriveSpeed = 4 * IR_DRIVE_SPEED_INCREMENT;
          break;
        case IR_4:
          IrDriveSpeed = 5 * IR_DRIVE_SPEED_INCREMENT;
          break;
        case IR_5:
          IrDriveSpeed = 6 * IR_DRIVE_SPEED_INCREMENT;
          break;
        case IR_6:
          IrDriveSpeed = 7 * IR_DRIVE_SPEED_INCREMENT;
          break;
        case IR_7:
          IrDriveSpeed = 8 * IR_DRIVE_SPEED_INCREMENT;
          break;
        case IR_8:
          IrDriveSpeed = 9 * IR_DRIVE_SPEED_INCREMENT;
          break;
        case IR_9:
          IrDriveSpeed = 10 * IR_DRIVE_SPEED_INCREMENT;
          break;
      }
      Serial.print("IR Drive Speed =");
      Serial.println(IrDriveSpeed);
      break;

    case IR_HASH:
      Serial.println("IR_HASH");
      delay(250);
      IrReceiver.resume();
      while (IrReceiver.decode() == 0) {
        ;
      }
      IrCommand = IrReceiver.decodedIRData.command;
      switch (IrCommand) {
        case IR_0:
          IrDriveDistance = IR_DRIVE_DISTANCE_INCREMENT;
          Serial.println("IR_0");
          break;
        case IR_1:
          IrDriveDistance = 2 * IR_DRIVE_DISTANCE_INCREMENT;
          break;
        case IR_2:
          IrDriveDistance = 3 * IR_DRIVE_DISTANCE_INCREMENT;
          break;
        case IR_3:
          IrDriveDistance = 4 * IR_DRIVE_DISTANCE_INCREMENT;
          break;
        case IR_4:
          IrDriveDistance = 5 * IR_DRIVE_DISTANCE_INCREMENT;
          break;
        case IR_5:
          IrDriveDistance = 6 * IR_DRIVE_DISTANCE_INCREMENT;
          break;
        case IR_6:
          IrDriveDistance = 7 * IR_DRIVE_DISTANCE_INCREMENT;
          break;
        case IR_7:
          IrDriveDistance = 8 * IR_DRIVE_DISTANCE_INCREMENT;
          break;
        case IR_8:
          IrDriveDistance = 9 * IR_DRIVE_DISTANCE_INCREMENT;
          break;
        case IR_9:
          IrDriveDistance = 10 * IR_DRIVE_DISTANCE_INCREMENT;
          break;
      }
      Serial.print("IR Drive Distance =");
      Serial.println(IrDriveDistance);
      break;
  }
  IrReceiver.resume();
}

void loop() {
  // Serial.println("This is working");

  if (IrReceiver.decode()) {
    IrControlInterpreter();
  }
  /*
  IrReceiver.printIRResultShort(&Serial);
  Serial.println(IrReceiver.decodedIRData.command, HEX);
  Serial.println();
  IrReceiver.resume();

  if (IrReceiver.decodedIRData.command == 0x10) {
    Serial.println("Do something");
  } else if (IrReceiver.decodedIRData.command == 0x11) {
    Serial.println("Do something else");
  }
}
delay(500);
*/
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