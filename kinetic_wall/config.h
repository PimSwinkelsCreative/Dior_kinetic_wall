#pragma once
#include <Arduino.h>

#define MAX_NUM_MOTORS 12

#define MICROSTEP_HIGHRES 16
#define MICROSTEP_LOWRES 8

// motor speed and acceleration settings:
#define MAX_SPEED 5               // motor speed will never exceed this limit
#define ANIMATION_SPEED 1         // speed setting for the animation
#define ANIMATION_ACCELERATION 2  // accleration setting for the animation

#define SENSORPINS_INVERTED true

extern uint8_t nMotors;
extern uint8_t tmc220x_version;
extern uint8_t microStep_setting;
extern uint8_t animationNumber;

void getConfigSettings();
