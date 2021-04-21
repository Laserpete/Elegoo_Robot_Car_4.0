#include <Arduino.h>

#include "FastLED.h"
#include "drivingFunctions.h"
#include "hardwareConfig.h"

CRGB leds[NUM_LEDS];

void setup() {
  Serial.begin(115200);
  setupDrivingPins();
  setupIrReceiver();
  setupUltrasonic();

  // Serial.println(F("START " __FILE__ " from "__DATE__
  //                  "\r\nUsing IR Remote library version " VERSION_IRREMOTE));

  FastLED.addLeds<NEOPIXEL, PIN_RBGLED>(leds, NUM_LEDS);
  FastLED.setBrightness(20);
}

uint32_t Colour(uint8_t r, uint8_t g, uint8_t b) {
  return (((uint32_t)r << 16) | ((uint32_t)g << 8) | b);
}

void loop() {
  // Serial.println("This is working");
  UARTControlInterpreter();

  IrControlInterpreter();

  // int pingD = ultrasonicPingDistance();
  // Serial.print("Ping distance = ");
  // Serial.print(pingD);
  // Serial.println(" Centimeters");
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