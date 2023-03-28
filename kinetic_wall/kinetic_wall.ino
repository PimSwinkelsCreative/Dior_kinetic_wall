#include "I2C_expander.h"
#include "buildFlags.h"
#include "config.h"
#include "motorControl.h"
#include "pinout.h"

unsigned long lastPositionUpdate = 0;

uint8_t currentLed = 0;

void setup() {
  Serial.begin(115200);
  Serial.println("\n\n\n");  // create some space between the startup header and
                             // the debug info

  // fetch the  config settings:
  setupConfigI2CExpander();
  getConfigSettings();

  // now that we know the config we can complete the setup:
  setupDirectionIoExpander();
  setupSensorIoExpander();
  setupMotors();

// perform a homing procedure on startup:
#ifndef SKIP_HOMING
  for (int i = 0; i < nMotors; i++) {
    startMotorHoming(i, true);
  }
#endif
}

void loop() {
  unsigned long startTime = micros();
  if (millis() - lastPositionUpdate > 2000) {
    lastPositionUpdate = millis();
    for (int i = 0; i < nMotors; i++) {
      moveMotor(i, 0.25, 0.5, 5);
    }
  }

  // // update the steppermotors, update as often as possible!
  // updateMotors();

  unsigned long loopTime = micros() - startTime;
  // Serial.println("looptime: "+String(loopTime)+"us");
}
