#pragma once
#include <Arduino.h>

#define MAX_NUM_MOTORS 12

#define MICROSTEP_HIGHRES 16
#define MICROSTEP_LOWRES 8

#define MAX_SPEED 5
#define MIN_POS -1
#define MAX_POS 2

#define SENSORPINS_INVERTED true

extern uint8_t nMotors;
extern uint8_t tmc220x_version;
extern uint8_t microStep_setting;
extern uint8_t animationNumber;

void getConfigSettings();

