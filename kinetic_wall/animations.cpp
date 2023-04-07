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
    case 0:
      // wave with offset back and forth
      playOscillatingWaveWithOffset(22000);
      break;
    case 1:
      // wave with offset left to right
      playWaveWithOffset(12000);
      break;
    case 2:
      // wave interleaving
      playWaveInterleaving(12000);
      break;
    case 3:
      // wave with offset interleaving
      playWaveWithOffsetInterleaving(12000);
      break;
    case 4:
      // random "shooting stars"
      playShootingStars(500, 6000, 5000, 20000, true);
      break;
    case 5:
      playOscillatingWaveWithSmallOffset(20000);
      break;
    case 6:
      playVerticalSnake(20000);
      break;
    case 7:
      playArrow(20000);
      break;
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
    moveMotorToNearestPosition(
        i, pos, MAX_SPEED_FACTOR * 1000.0 / float(animationDuration / 2),
        ANIMATION_ACCELERATION);
  }
}

void playOscillatingWaveWithSmallOffset(unsigned int animationDuration) {
  float motorOffsets[nMotors];
  for (int i = 0; i < nMotors; i++) {
    motorOffsets[i] = float(i) / (float(2 * nMotors));
  }
  float animationProgress =
      float(millis() % animationDuration) / float(animationDuration);
  for (int i = 0; i < nMotors; i++) {
    float pos = animationProgress + motorOffsets[i];
    if (pos > 1) pos -= 1;
    pos = pos * 2;
    if (pos > 1) pos = 2 - pos;
    moveMotorToNearestPosition(
        i, pos, MAX_SPEED_FACTOR * 1000.0 / float(animationDuration / 2),
        ANIMATION_ACCELERATION);
  }
}

void playVerticalSnake(unsigned int animationDuration) {
  float motorOffsets[nMotors];
  for (int i = 0; i < nMotors; i++) {
    motorOffsets[i] = float(i) / (float(4 * nMotors));
  }
  float animationProgress =
      float(millis() % animationDuration) / float(animationDuration);
  for (int i = 0; i < nMotors; i++) {
    float pos = animationProgress + motorOffsets[i];
    if (pos > 1) pos -= 1;
    pos = pos * 2;
    if (pos > 1) pos = 2 - pos;
    pos = 0.1 + pos * 0.8;
    moveMotorToNearestPosition(
        i, pos, MAX_SPEED_FACTOR * 1000.0 / float(animationDuration / 2),
        ANIMATION_ACCELERATION);
  }
}

void playArrow(unsigned int animationDuration) {
  float motorOffsets[nMotors];
  uint8_t middleRow = nMotors / 2;
  for (int i = 0; i < nMotors; i++) {
    motorOffsets[i] = float(-abs(i - middleRow)) / (float(3 * nMotors));
  }
  float animationProgress =
      float(millis() % animationDuration) / float(animationDuration);
  for (int i = 0; i < nMotors; i++) {
    float pos = animationProgress + motorOffsets[i];
    if (pos > 1) pos -= 1;
    if (pos < 0) pos += 1;
    pos = pos * 2;
    if (pos > 1) pos = 2 - pos;
    pos = 0.1 + pos * 0.8;
    moveMotorToNearestPosition(
        i, pos, MAX_SPEED_FACTOR * 1000.0 / float(animationDuration / 2),
        ANIMATION_ACCELERATION);
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
    moveMotorToNearestPosition(
        i, pos, MAX_SPEED_FACTOR * 1000.0 / float(animationDuration),
        ANIMATION_ACCELERATION);
  }
}

void playWaveInterleaving(unsigned int animationDuration) {
  float animationProgress = float(millis()) / float(animationDuration);
  for (int i = 0; i < nMotors; i++) {
    float pos = animationProgress;
    if (i % 2 == 0) {
      moveMotorToNearestPosition(
          i, pos, MAX_SPEED_FACTOR * 1000.0 / float(animationDuration),
          ANIMATION_ACCELERATION);
    } else {
      moveMotorToNearestPosition(
          i, -pos, MAX_SPEED_FACTOR * 1000.0 / float(animationDuration),
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
      moveMotorToNearestPosition(
          i, pos, MAX_SPEED_FACTOR * 1000.0 / float(animationDuration),
          ANIMATION_ACCELERATION);
    } else {
      moveMotorToNearestPosition(
          i, -pos, MAX_SPEED_FACTOR * 1000.0 / float(animationDuration / 2),
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

    // set the inactive motor channels to their zero position
    for (int i = 0; i < nMotors; i++) {
      if (!motorBusy[i]) {
        moveMotorToNearestPosition(
            i, 0, 1.0 * 1000.0 / ((maxStarDuration + minStarDuration) / 2.0),
            ANIMATION_ACCELERATION);
      }
    }
  }
  // cycle through the shooting star list and update the animations:
  for (int i = 0; i < shootingStarQueLength; i++) {
    if (millis() >= shootingStars[i].startTime &&
        millis() < shootingStars[i].startTime + shootingStars[i].duration) {
      float shootingStarProgress =
          float(millis() - shootingStars[i].startTime) /
          float(shootingStars[i].duration);
      if (shootingStars[i].direction) {
        moveMotorToNearestPosition(
            shootingStars[i].motorChannel, shootingStarProgress,
            MAX_SPEED_FACTOR * 1000.0 / float(shootingStars[i].duration),
            ANIMATION_ACCELERATION);
      } else {
        moveMotorToNearestPosition(
            shootingStars[i].motorChannel, -shootingStarProgress,
            MAX_SPEED_FACTOR * 1000.0 / float(shootingStars[i].duration),
            ANIMATION_ACCELERATION);
      }
    }
  }
}