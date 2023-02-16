

#include "globals.h"
#include "motorControl.h"
#include "pinout.h"

unsigned long lastPositionUpdate = 0;

void setup() {
  Serial.begin(115200);
  setupMotors();
}

void loop() {
  if (millis() - lastPositionUpdate > 2000) {
    lastPositionUpdate = millis();
    for (int i = 0; i < NUM_MOTORS; i++) {
      moveMotorToPosition(i, float(random(-100, 100)) / 100.0, 10);
    }
  }
  updateMotors();
}
