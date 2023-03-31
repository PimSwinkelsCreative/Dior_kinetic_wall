#pragma once
#include <Arduino.h>

struct shootingStar {
  uint8_t motorChannel;
  unsigned long startTime;
  unsigned int duration;
  bool direction;
};

void setupAnimations();

void playAnimation(uint8_t currentAnimation);

void playOscillatingWaveWithOffset(unsigned int animationDuration);

void playWaveWithOffset(unsigned int animationDuration);

void playWaveInterleaving(unsigned int animationDuration);

void playWaveWithOffsetInterleaving(unsigned int animationDuration);

void playShootingStars(unsigned int minInterval, unsigned int maxInterval,
                       unsigned int minStarDuration,
                       unsigned int maxStarDuration, bool bothDirections);

void playDancingSilhouette(float speed, float variation, float spread);