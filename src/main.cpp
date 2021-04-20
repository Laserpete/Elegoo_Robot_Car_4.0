#include <Arduino.h>

#include "Driving_Functions.h"
#include "FastLED.h"
#include "Setup.h"

CRGB leds[NUM_LEDS];

float calibratedSpeedOfSound;

int calibrateUltrasonic() {
  // Calibration. Distance / Time = speed.
  long pingTravelTime, speedOfSound;
  long calibrationDistance = 30;
  digitalWrite(ULTRASONIC_TRIGGER_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(ULTRASONIC_TRIGGER_PIN, HIGH);
  delayMicroseconds(20);
  digitalWrite(ULTRASONIC_TRIGGER_PIN, LOW);
  pingTravelTime = pulseIn(ULTRASONIC_ECHO_PIN, HIGH);
  Serial.print("pingTravelTime = ");
  Serial.println(pingTravelTime);

  speedOfSound = (calibrationDistance * 1000000) / pingTravelTime;
  Serial.print("Calibrated Speed of sound = ");
  Serial.println(speedOfSound);
  return speedOfSound;
}

void setup() {
  Serial.begin(115200);
  setupDrivingPins();
  setupIrReceiver();
  calculateCalibration();
  pinMode(ULTRASONIC_ECHO_PIN, INPUT);
  pinMode(ULTRASONIC_TRIGGER_PIN, OUTPUT);
  calibratedSpeedOfSound = calibrateUltrasonic();
  Serial.println("IR Receiver Begin");

  // Serial.println(F("START " __FILE__ " from "__DATE__
  //                  "\r\nUsing IR Remote library version " VERSION_IRREMOTE));

  FastLED.addLeds<NEOPIXEL, PIN_RBGLED>(leds, NUM_LEDS);
  FastLED.setBrightness(20);
}

int pingDistance() {
  int pingTravelTime;
  digitalWrite(ULTRASONIC_TRIGGER_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(ULTRASONIC_TRIGGER_PIN, HIGH);
  delayMicroseconds(20);
  digitalWrite(ULTRASONIC_TRIGGER_PIN, LOW);
  pingTravelTime = pulseIn(ULTRASONIC_ECHO_PIN, HIGH);
  // speed of sound divided by time = distance of object
  // speed of sound in air = 343 meters per second at 20 C
  // or 34300 CM per second
  // or 34.3 CM per milisecond
  // or 0.0343 CM per microsecond

  float pingDistance = pingTravelTime * (calibratedSpeedOfSound / 1000000);

  return pingDistance;
}

uint32_t Colour(uint8_t r, uint8_t g, uint8_t b) {
  return (((uint32_t)r << 16) | ((uint32_t)g << 8) | b);
}

void loop() {
  // Serial.println("This is working");
  UARTControlInterpreter();

  IrControlInterpreter();

  int pingD = pingDistance();
  Serial.print("Ping distance = ");
  Serial.print(pingD);
  Serial.println(" Centimeters");
  delay(500);
}

// CMpS = 50;

// CMpS = calculateMaximumSpeed(CMpS);
// wheelPWM = calcWheelPWM(CMpS);

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