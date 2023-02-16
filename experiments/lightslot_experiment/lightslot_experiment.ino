#include "globals.h"

void setup() {
  Serial.begin(115200);
  setupI2CExpander();
}

void loop() {
  if (lightSensorChangeFlag) {
    bool sensorStates[NUM_MOTORS];
    for (int i = 0; i < NUM_MOTORS; i++) {
      sensorStates[i] = digitalReadI2CExpanderPin(lightSensorPins[i]);
      Serial.println("Sensor " + String(i) + " is " +
                     String(sensorStates[i] ? "HIGH" : "LOW"));
    }
    lightSensorChangeFlag = false;
  }
}