#include "motorControl.h"
#include "pinout.h"
#include "I2C_expander.h"
#include "config.h"
#include "buildFlags.h"

unsigned long lastPositionUpdate = 0;

uint8_t currentLed = 0;

void setup() {
  Serial.begin(115200);
  Serial.println("\n\n\n"); //create some space between the startup header and the debug info

  //fetch the  config settings:
  setupConfigI2CExpander();
  getConfigSettings();


  //now that we know the config we can complete the setup:
  setupDirectionIoExpander();
  setupSensorIoExpander();
  setupMotors();


  // perform a homing procedure on startup:
#ifndef SKIP_HOMING
  for (int i = 0;i < nMotors;i++) {
    startMotorHoming(i, true);
  }
#endif
}

void loop() {
  unsigned long startTime = micros();
  // if (millis() - lastPositionUpdate > 2000) {
  //   lastPositionUpdate = millis();
  //   for (int i = 0; i < nMotors; i++) {
  //     moveMotorToPosition(i, float(random(-100, 100)) / 100.0, 1, 5);
  //   }
  // }

  // // update the steppermotors, update as often as possible!
  // updateMotors();

  if (millis() - lastPositionUpdate > 10) {
    playAnimation(0);
  }

  unsigned long loopTime = micros() - startTime;
  // Serial.println("looptime: "+String(loopTime)+"us");
}

void playAnimation(uint8_t currentAnimation) {

  switch (currentAnimation) {
  case 0: {
    int animationDuration = 8000;  //animation duration in milliseconds
    float motorOffsets[nMotors];
    for (int i = 0;i < nMotors;i++) {
      motorOffsets[i] = float(i) / float(nMotors);
    }
    float animationProgress = float(millis() % animationDuration) / float(animationDuration);
    for (int i = 0;i < nMotors;i++) {
      float pos = animationProgress+motorOffsets[i];
      if(pos>1) pos-=1;
      pos = pos*2;
      if(pos>1) pos = 2-pos;
      moveMotorToPosition(i, pos, 10, 10);
    }
    break;
  } 
  case 1:
  case 2:
  case 3:
  default:
    Serial.println("ERROR: animation out of range!");
    break;

  }
}
