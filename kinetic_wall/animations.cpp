#include "animations.h"

#include "buildFlags.h"
#include "config.h"
#include "motorControl.h"

// shooting star variables:
const uint8_t shootingStarQueLength = MAX_NUM_MOTORS;
shootingStar shootingStars[shootingStarQueLength];
uint8_t shootingStarIndex = 0;
unsigned long lastShootingStargenerated = 0;
unsigned int shootingStarInterval = 0;

void setupAnimations() {
  // initialize all the shooting star parameters to 0
  for (int i = 0; i < shootingStarQueLength; i++) {
    shootingStars[i].direction = true;
    shootingStars[i].duration = 0;
    shootingStars[i].motorChannel = 0;
    shootingStars[i].startTime = 0;
  }
}

void playAnimation(uint8_t currentAnimation) {
  switch (currentAnimation) {
    case 0: {
      // wave with offset back and forth
      playOscillatingWaveWithOffset(16000);
      break;
    }
    case 1: {
      // wave with offset left to right
      playWaveWithOffset(10000);
      break;
    }
    case 2: {
      // wave interleaving
      playWaveInterleaving(10000);
      break;
    }
    case 3: {
      // wave with offset interleaving
      playWaveWithOffsetInterleaving(10000);
      break;
    }
    case 4: {
      // random "shooting stars"
      playShootingStars(200, 2000, 3000, 5000, true);
    }
    default:
      // Serial.println("ERROR: animation out of range!");
      break;
  }
}

void playOscillatingWaveWithOffset(unsigned int animationDuration) {
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
    moveMotorToNearestPosition(i, pos, ANIMATION_SPEED, ANIMATION_ACCELERATION);
  }
}

void playWaveWithOffset(unsigned int animationDuration) {
  float motorOffsets[nMotors];
  for (int i = 0; i < nMotors; i++) {
    motorOffsets[i] = float(i) / float(nMotors);
  }
  float animationProgress = float(millis()) / float(animationDuration);
  for (int i = 0; i < nMotors; i++) {
    float pos = animationProgress + motorOffsets[i];
    moveMotorToNearestPosition(i, pos, ANIMATION_SPEED, ANIMATION_ACCELERATION);
  }
}

void playWaveInterleaving(unsigned int animationDuration) {
  float animationProgress = float(millis()) / float(animationDuration);
  for (int i = 0; i < nMotors; i++) {
    float pos = animationProgress;
    pos += .12;  // create an offset to make the waves meet in the middle
    if (i % 2 == 0) {
      moveMotorToNearestPosition(i, pos, ANIMATION_SPEED,
                                 ANIMATION_ACCELERATION);
    } else {
      moveMotorToNearestPosition(i, -pos, ANIMATION_SPEED,
                                 ANIMATION_ACCELERATION);
    }
  }
}

void playWaveWithOffsetInterleaving(unsigned int animationDuration) {
  float motorOffsets[nMotors];
  for (int i = 0; i < nMotors; i++) {
    motorOffsets[i] = float(i) / float(nMotors);
  }
  float animationProgress = float(millis()) / float(animationDuration);
  for (int i = 0; i < nMotors; i++) {
    float pos = animationProgress + motorOffsets[i];
    pos += .1;  // create an offset to make the waves meet in the middle
    if (i % 2 == 0) {
      moveMotorToNearestPosition(i, pos, ANIMATION_SPEED,
                                 ANIMATION_ACCELERATION);
    } else {
      moveMotorToNearestPosition(i, -pos, ANIMATION_SPEED,
                                 ANIMATION_ACCELERATION);
    }
  }
}

void playShootingStars(unsigned int minInterval, unsigned int maxInterval,
                       unsigned int minStarDuration,
                       unsigned int maxStarDuration, bool bothDirections) {
  // generate a new shooting star with a random interval:
  if (millis() > lastShootingStargenerated + shootingStarInterval) {
#ifdef DEBUG_ANIMATIONS
    Serial.println("generating shooting star");
#endif
    // check what motorChannels are busy:
    bool motorBusy[nMotors];
    uint8_t nBusyChannels = 0;
    // set all the busy flags to false:
    for (int i = 0; i < nMotors; i++) {
      motorBusy[i] = false;
    }
    // set the busy flags to true if there is an animation busy on that channel
    for (int i = 0; i < shootingStarQueLength; i++) {
      if (millis() >= shootingStars[i].startTime &&
          millis() < shootingStars[i].startTime + shootingStars[i].duration) {
        motorBusy[shootingStars[i].motorChannel] = true;
        nBusyChannels++;
      }
    }
    if (nBusyChannels < nMotors) {
      // pick a channel for the shooting star to play on. keep trying new
      // channels if the channel is already busy
      uint8_t channel = random(nMotors);
      while (motorBusy[channel]) {
        channel = random(nMotors);
      }
      shootingStars[shootingStarIndex].motorChannel = channel;
      shootingStars[shootingStarIndex].duration =
          random(minStarDuration, maxStarDuration);
      shootingStars[shootingStarIndex].startTime = millis();
      if (bothDirections) {
        shootingStars[shootingStarIndex].direction =
            random(2) == 1 ? true : false;
      } else {
        shootingStars[shootingStarIndex].direction = true;
      }
#ifdef DEBUG_ANIMATIONS
      Serial.println("channel: " +
                     String(shootingStars[shootingStarIndex].motorChannel));
      Serial.println("direction: " +
                     String(shootingStars[shootingStarIndex].direction));
      Serial.println("duration: " +
                     String(shootingStars[shootingStarIndex].duration));
      Serial.println("startTime: " +
                     String(shootingStars[shootingStarIndex].startTime));
#endif

      // raise and wrap the shooting star index
      shootingStarIndex = (shootingStarIndex + 1) % shootingStarQueLength;

      // set the timer for when to generate the next shooting star:
      shootingStarInterval = random(minInterval, maxInterval);
      lastShootingStargenerated = millis();
    }
  }
  // cycle through the shooting star list and update the aniamtions:
  for (int i = 0; i < shootingStarQueLength; i++) {
    if (millis() >= shootingStars[i].startTime &&
        millis() < shootingStars[i].startTime + shootingStars[i].duration) {
      float shootingStarProgress =
          float(millis() - shootingStars[i].startTime) /
          float(shootingStars[i].duration);
      if (shootingStars[i].direction) {
        moveMotorToNearestPosition(shootingStars[i].motorChannel,
                                   shootingStarProgress, ANIMATION_SPEED,
                                   ANIMATION_ACCELERATION);
      } else {
        moveMotorToNearestPosition(shootingStars[i].motorChannel,
                                   -shootingStarProgress, ANIMATION_SPEED,
                                   ANIMATION_ACCELERATION);
      }
    }
  }
}