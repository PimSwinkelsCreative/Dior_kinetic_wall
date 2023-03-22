#include "globals.h"
#include "motorControl.h"
#include "pinout.h"
#include "I2C_expander.h"
#include "config.h"

unsigned long lastPositionUpdate = 0;

uint8_t currentLed = 0;

void setup() {
  Serial.begin(115200);
  setupI2CExpanders();
  // setupMotors();

  // // perform a homing procedure on startup:
  // for (int i = 0; i < NUM_MOTORS; i++) {
  //   startMotorHoming(i);
  // }
}

void loop() {
  // if (millis() - lastPositionUpdate > 2000) {
  //   lastPositionUpdate = millis();
  //   for (int i = 0; i < NUM_MOTORS; i++) {
  //     moveMotorToPosition(i, float(random(-100, 100)) / 100.0, 10);
  //   }
  // }

  // update the steppermotors, update as often as possible!
  // updateMotors();

  // getConfigSettings();
  delay(100);
  digitalWriteI2CExpanderPin(debugLeds[currentLed], LOW);
  currentLed++;
  currentLed %= 4;
  digitalWriteI2CExpanderPin(debugLeds[currentLed], HIGH);
}
