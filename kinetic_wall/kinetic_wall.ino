#include <Arduino.h>

#include "I2C_expander.h"
#include "animations.h"
#include "buildFlags.h"
#include "config.h"
#include "motorControl.h"
#include "pinout.h"

unsigned long lastPositionUpdate = 0;
uint8_t prevLedmask =
    100;  // set the ledmask out of range to force an initial trigger

uint8_t cyclingAnimations[] = {0, 1, 2, 3};
uint8_t nAnimations = sizeof(cyclingAnimations);
unsigned long animationCyclingDuration =
    CYCLE_LENGTH_MINUTES * 60 * 1000;  // make the total cycle 5 mins

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

  setupAnimations();

  // perform a homing procedure on startup:
#ifndef SKIP_HOMING
  for (int i = 0; i < nMotors; i++) {
    startMotorHoming(i, true);
  }
#endif
}

void loop() {
  // update the animation every 10ms
  if (millis() - lastPositionUpdate > 10) {
    uint8_t animationToPlay = (millis() % animationCyclingDuration) /
                              (animationCyclingDuration / nAnimations);
    Serial.println("playing Animation: " + String(animationToPlay));
    playAnimation(animationToPlay);
  }
}
