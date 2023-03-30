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

  // if (millis() - lastPositionUpdate > 2000) {
  //   lastPositionUpdate = millis();
  //   for (int i = 0; i < nMotors; i++) {
  //     moveMotorToPosition(i, float(random(-100, 100)) / 100.0, 1, 5);
  //   }
  // }

  // // update the steppermotors, update as often as possible!
  // updateMotors();

  if (millis() - lastPositionUpdate > 10) {
    playAnimation(animationNumber);
  }

  unsigned long loopTime = micros() - startTime;
  // Serial.println("looptime: "+String(loopTime)+"us");
}

void playAnimation(uint8_t currentAnimation) {
  switch (currentAnimation) {
    case 0: {
      // wave with offset back and forth
      int animationDuration = 16000;  // animation duration in milliseconds
      float motorOffsets[nMotors];
      for (int i = 0; i < nMotors; i++) {
        motorOffsets[i] = float(i) / float(nMotors);
      }
      float animationProgress =
          float(millis() % animationDuration) / float(animationDuration);
      for (int i = 0; i < nMotors; i++) {
        float pos = animationProgress + motorOffsets[i];
        if (pos > 1) pos -= 1;
        pos = pos * 2;
        if (pos > 1) pos = 2 - pos;
        moveMotorToPosition(i, pos, ANIMATION_SPEED, ANIMATION_ACCELERATION);
      }
      break;
    }
    case 1: {
      // wave with offset left to right
      int animationDuration = 10000;  // animation duration in milliseconds
      float motorOffsets[nMotors];
      for (int i = 0; i < nMotors; i++) {
        motorOffsets[i] = float(i) / float(nMotors);
      }
      float animationProgress = float(millis()) / float(animationDuration);
      for (int i = 0; i < nMotors; i++) {
        float pos = animationProgress + motorOffsets[i];
        moveMotorToPosition(i, pos, ANIMATION_SPEED, ANIMATION_ACCELERATION);
      }
      break;
    }
    case 2: {
      // wave interleaving
      int animationDuration = 10000;  // animation duration in milliseconds
      float animationProgress = float(millis()) / float(animationDuration);
      for (int i = 0; i < nMotors; i++) {
        float pos = animationProgress;
        pos += .12;  // create an offset to make the waves meet in the middle
        if (i % 2 == 0) {
          moveMotorToPosition(i, pos, ANIMATION_SPEED, ANIMATION_ACCELERATION);
        } else {
          moveMotorToPosition(i, -pos, ANIMATION_SPEED, ANIMATION_ACCELERATION);
        }
      }
      break;
    }
    case 3: {
      // wave with offset interleaving
      int animationDuration = 10000;  // animation duration in milliseconds
      float motorOffsets[nMotors];
      for (int i = 0; i < nMotors; i++) {
        motorOffsets[i] = float(i) / float(nMotors);
      }
      float animationProgress = float(millis()) / float(animationDuration);
      for (int i = 0; i < nMotors; i++) {
        float pos = animationProgress + motorOffsets[i];
        pos += .1;  // create an offset to make the waves meet in the middle
        if (i % 2 == 0) {
          moveMotorToPosition(i, pos, ANIMATION_SPEED, ANIMATION_ACCELERATION);
        } else {
          moveMotorToPosition(i, -pos, ANIMATION_SPEED, ANIMATION_ACCELERATION);
        }
      }
      break;
    }
    default:
      // Serial.println("ERROR: animation out of range!");
      break;
  }
}
