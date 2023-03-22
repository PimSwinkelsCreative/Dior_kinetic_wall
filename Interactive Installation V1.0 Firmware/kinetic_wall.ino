

#include "globals.h"
#include "motorControl.h"
#include "pinout.h"
#include "serialInterface.h"

unsigned long lastPositionUpdate = 0;

void setup() {
  setupSerialInterface(115200);
  setupMotors();

  // perform a homing procedure on startup:
  for (int i = 0; i < NUM_MOTORS; i++) {
    startMotorHoming(i);
  }
}

void loop() {
  // if (millis() - lastPositionUpdate > 2000) {
  //   lastPositionUpdate = millis();
  //   for (int i = 0; i < NUM_MOTORS; i++) {
  //     moveMotorToPosition(i, float(random(-100, 100)) / 100.0, 10);
  //   }
  // }

  // update the steppermotors, update as often as possible!
  updateMotors();

  // check for new serial messages
  updateSerial();
}
