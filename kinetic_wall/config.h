#pragma once
#include <Arduino.h>

// hardware definitions:
#define MAX_NUM_MOTORS 12
#define MICROSTEP_HIGHRES 16
#define MICROSTEP_LOWRES 8
#define SENSORPINS_INVERTED true

// motor speed and acceleration settings:
#define MAX_SPEED_FACTOR \
  1.5  // facotr at which the max speed is set relative to the required speed.
       // Will determine how fast the transitions go
#define ANIMATION_ACCELERATION 0.02  // acceleration setting for the animation

// anination sequencing:
#define CYCLE_LENGTH_MINUTES 5  // minutes that the entire cycle takes

// global variables:
extern uint8_t nMotors;
extern uint8_t tmc220x_version;
extern uint8_t microStep_setting;
extern uint8_t animationNumber;
extern bool cycleThroughAnimations;
extern float positionOffset;

void getConfigSettings();
