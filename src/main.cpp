#include <Arduino.h>

#define calTime 2
#define degsSec 207
#define cal100 75
#define cal255 188
#define steeringTrimVal 20

int AIN1 = 8;  // direction low = forward
int AIN2 = 5;  // speed PWM
int BIN1 = 7;  // direction high = forward
int BIN2 = 6;  // speed PWM

int leftSpeed;
int rightSpeed;
int wheelPWM;
float CMpS;

// Use calibration definitions to calculate gradient for PWM : CM per second
float speed100 = cal100 / calTime;      // speed at PWM 100
float speed225 = cal255 / calTime;      // speed at PWM 255
float m = (speed225 - speed100) / 155;  // (speed225 - speed100) / 255 - 100

void setup() {
  Serial.begin(9600);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
}

void loop() {
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

  // Calculate maximum speed
  float maxCMpS = (m * 255) - (m * 100) + speed100;
  if (CMpS >= maxCMpS) {
    CMpS = maxCMpS;
  }

  Serial.print("Max velocity = ");
  Serial.print(maxCMpS);
  Serial.println("CM per Second");

  // int wheelPWM = CMpS/m + (m*100) - speed100;

  wheelPWM = calcWheelPWM(CMpS);
  Serial.print("wheelPWM = ");
  Serial.println(wheelPWM);

  Serial.print("CMpS = ");
  Serial.println(CMpS);

  // setSpeed(leftSpeed, rightSpeed);

  forward(100, CMpS);
  stopCar(1);

  while (1)
    ;
}

int calcWheelPWM(float desiredVelocity) {
  int calculatedPWM = desiredVelocity / m + (m * 100) - speed100;
  return calculatedPWM;
}

void setSpeed(int leftVal, int rightVal) {
  analogWrite(AIN2, leftVal);
  analogWrite(BIN2, rightVal);
}

void forward(int d, float velocity) {
  // Set wheel turning direction
  digitalWrite(AIN1, LOW);
  digitalWrite(BIN1, HIGH);

  // Calculate wheel PWM value and et steering trim

  leftSpeed = rightSpeed = calcWheelPWM(velocity);
  rightSpeed = leftSpeed - steeringTrimVal;

  // Set wheel PWM values
  analogWrite(AIN2, leftSpeed);
  analogWrite(BIN2, rightSpeed);

  // Calculate time to drive
  float t = d / velocity;
  delay(t * 1000);
}

void backward(int d) {
  digitalWrite(AIN1, HIGH);
  digitalWrite(BIN1, LOW);
  // float t=(d/metersSec)*1000;
  //  delay(t);
}

void turnLeft(int d) {
  digitalWrite(AIN1, LOW);
  digitalWrite(BIN1, LOW);
  float t = 1000 / degsSec;
  delay(t * d);
}

void turnRight(int d) {
  digitalWrite(AIN1, HIGH);
  digitalWrite(BIN1, HIGH);
  float t = 1000 / degsSec;
  delay(t * d);
}

void stopCar(int d) {
  digitalWrite(AIN2, LOW);
  digitalWrite(BIN2, LOW);
  delay(d * 1000);
}

void calF() {
  digitalWrite(AIN1, LOW);
  digitalWrite(BIN1, HIGH);
  delay(calTime * 1000);
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