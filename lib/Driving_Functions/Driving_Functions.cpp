#include "Driving_Functions.h"

#include <Arduino.h>

#include "IRremote.h"

int leftSpeed;
int rightSpeed;
int wheelPWM;
float CMpS;

// Variables for the adjustable IR controlled speed/distance
int IrDriveDistance = 50;
int IrDriveSpeedMin, IrDriveSpeedMax, IrDriveSpeed;

int UARTDriveDistance = 50;
int UARTDriveSpeedMin, UARTDriveSpeedMax, UARTDriveSpeed;

// Use calibration definitions to calculate gradient for PWM : CM per second
float speed100 = CALIBRATION_AT_PWM_100 / CALIBRATION_TIME;  // speed at PWM 100
float speed225 = CALIBRATION_AT_PWM_255 / CALIBRATION_TIME;  // speed at PWM 255
float m = (speed225 - speed100) / 155;  // (speed225 - speed100) / 255 - 100

void setupDrivingPins() {
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
}

void setupIrReceiver() {
  IrReceiver.begin(IR_RECEIVE_PIN, false);
  while (!Serial) {
    ;
  }
}

void calculateCalibration() {
  IrDriveSpeedMax = (m * 255) - (m * 100) + speed100;
  // Serial.println(IrDriveSpeedMax);
  IrDriveSpeedMin = (m * 100) - (m * 100) + speed100;
  // Serial.println(IrDriveSpeedMin);
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
  Serial.println(rotationTime);
  return rotationTime;
}
void stopCar() {
  digitalWrite(AIN2, LOW);
  digitalWrite(BIN2, LOW);
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
  stopCar();
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
  stopCar();
}

void turnLeft(int angle) {
  stopCar();
  digitalWrite(AIN1, LOW);
  digitalWrite(BIN1, LOW);
  analogWrite(AIN2, TURN_LR_PWM);
  analogWrite(BIN2, TURN_LR_PWM);
  float rotationTime = calculateRotationTime(angle);
  delay(rotationTime);
  stopCar();
}

void turnRight(int angle) {
  stopCar();
  digitalWrite(AIN1, HIGH);
  digitalWrite(BIN1, HIGH);
  analogWrite(AIN2, TURN_LR_PWM);
  analogWrite(BIN2, TURN_LR_PWM);
  float rotationTime = calculateRotationTime(angle);
  delay(rotationTime);
  stopCar();
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
  if (IrReceiver.decode()) {
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
        Serial.println("IR_ASTERISK, set IR speed");
        delay(250);
        IrReceiver.resume();
        while (IrReceiver.decode() == 0) {
          ;
        }
        IrCommand = IrReceiver.decodedIRData.command;
        int IrDriveSpeedSelect;
        switch (IrCommand) {
          case IR_0:
            IrDriveSpeedSelect = 0;
            Serial.println("IR_0");
            break;
          case IR_1:
            IrDriveSpeedSelect = 1;
            break;
          case IR_2:
            IrDriveSpeedSelect = 2;
            break;
          case IR_3:
            IrDriveSpeedSelect = 3;
            break;
          case IR_4:
            IrDriveSpeedSelect = 4;
            break;
          case IR_5:
            IrDriveSpeedSelect = 5;
            break;
          case IR_6:
            IrDriveSpeedSelect = 6;
            break;
          case IR_7:
            IrDriveSpeedSelect = 7;
            break;
          case IR_8:
            IrDriveSpeedSelect = 8;
            break;
          case IR_9:
            IrDriveSpeedSelect = 9;
            break;
          default:
            break;
        }

        IrDriveSpeed = (6.33 * IrDriveSpeedSelect) + 37;

        Serial.print("IR Drive Speed = ");
        Serial.println(IrDriveSpeed);
        break;

      case IR_HASH:
        Serial.println("IR_HASH, set IR Distance");
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
          default:
            break;
        }
        Serial.print("IR Drive Distance = ");
        Serial.println(IrDriveDistance);
        break;
    }
    IrReceiver.resume();
  }
}

void UARTHash() {
  Serial.println("UART_HASH, set UART distance");
  delay(250);
  while (Serial.available() == 0) {
    ;
  }
  char UARTCommand = Serial.read();

  if (UARTCommand == UART_0) {
    UARTDriveDistance = UART_DRIVE_DISTANCE_INCREMENT;
  } else if (UARTCommand == UART_1) {
    UARTDriveDistance = 1 * UART_DRIVE_DISTANCE_INCREMENT;
  } else if (UARTCommand == UART_2) {
    UARTDriveDistance = 2 * UART_DRIVE_DISTANCE_INCREMENT;
  } else if (UARTCommand == UART_3) {
    UARTDriveDistance = 3 * UART_DRIVE_DISTANCE_INCREMENT;
  } else if (UARTCommand == UART_4) {
    UARTDriveDistance = 4 * UART_DRIVE_DISTANCE_INCREMENT;
  } else if (UARTCommand == UART_5) {
    UARTDriveDistance = 5 * UART_DRIVE_DISTANCE_INCREMENT;
  } else if (UARTCommand == UART_6) {
    UARTDriveDistance = 6 * UART_DRIVE_DISTANCE_INCREMENT;
  } else if (UARTCommand == UART_7) {
    UARTDriveDistance = 7 * UART_DRIVE_DISTANCE_INCREMENT;
  } else if (UARTCommand == UART_8) {
    UARTDriveDistance = 8 * UART_DRIVE_DISTANCE_INCREMENT;
  } else if (UARTCommand == UART_9) {
    UARTDriveDistance = 9 * UART_DRIVE_DISTANCE_INCREMENT;
  } else {
    Serial.println("Not a valid value, enter 1 - 9");
    UARTHash();
  }
  Serial.print("UARTDriveDistance = ");
  Serial.println(UARTDriveDistance);
}

void UARTAsterisk() {
  Serial.println("UART_ASTERISK, set UART speed");
  delay(250);
  while (Serial.available() == 0) {
    ;
  }
  char UARTCommand = Serial.read();
  int UARTDriveSpeedSelect;

  if (UARTCommand == UART_0) {
    UARTDriveSpeedSelect = 0;
  } else if (UARTCommand == UART_1) {
    UARTDriveSpeedSelect = 1;
  } else if (UARTCommand == UART_2) {
    UARTDriveSpeedSelect = 2;
  } else if (UARTCommand == UART_3) {
    UARTDriveSpeedSelect = 3;
  } else if (UARTCommand == UART_4) {
    UARTDriveSpeedSelect = 4;
  } else if (UARTCommand == UART_5) {
    UARTDriveSpeedSelect = 5;
  } else if (UARTCommand == UART_6) {
    UARTDriveSpeedSelect = 6;
  } else if (UARTCommand == UART_7) {
    UARTDriveSpeedSelect = 7;
  } else if (UARTCommand == UART_8) {
    UARTDriveSpeedSelect = 8;
  } else if (UARTCommand == UART_9) {
    UARTDriveSpeedSelect = 9;
  } else {
    Serial.println("Not a valid value, enter 1 - 9");
    UARTAsterisk();
  }

  UARTDriveSpeed = (6.33 * UARTDriveSpeedSelect) + 37;

  Serial.print("UART Drive Speed = ");
  Serial.println(UARTDriveSpeed);
}

void UARTControlInterpreter() {
  char UARTCommand = Serial.read();

  switch (UARTCommand) {
    case UART_FORWARD:
      forward(UARTDriveDistance, UARTDriveSpeed);
      Serial.println(F("UART_FORWARD"));
      stopCar();
      break;

    case UART_BACKWARD:
      backward(UARTDriveDistance, UARTDriveSpeed);
      Serial.println(F("UART_BACKWARD"));
      stopCar();
      break;

    case UART_LEFT:
      turnLeft(UART_TURN_MAGNITUDE_DEGREES);
      Serial.println(F("UART_LEFT"));
      stopCar();
      break;

    case UART_JOG_LEFT:
      turnLeft(UART_JOG_MAGNITUDE_DEGREES);
      Serial.println(F("UART_JOG_LEFT"));
      stopCar();
      break;

    case UART_RIGHT:
      turnRight(UART_TURN_MAGNITUDE_DEGREES);
      Serial.println(F("UART_RIGHT"));
      stopCar();
      break;
    case UART_JOG_RIGHT:
      turnRight(UART_JOG_MAGNITUDE_DEGREES);
      Serial.println(F("UART_JOG_LEFT"));
      stopCar();
      break;

    case UART_180:
      turnRight(180);
      Serial.println(F("UART_180"));
      stopCar();
      break;

    case UART_ASTERISK:
      UARTAsterisk();

    case UART_HASH:
      UARTHash();
  }
}