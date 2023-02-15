#pragma once

#include "I2C_expander.h"

// ESP pins:
#define MOTOR1_STEP 12
#define MOTOR2_STEP 13
#define MOTOR3_STEP 16
#define MOTOR4_STEP 17
#define MOTOR5_STEP 18
#define MOTOR6_STEP 19
#define MOTOR7_STEP 23
#define MOTOR8_STEP 27
#define SDA_PIN 21
#define SCL_PIN 22
#define MS1_PIN 25
#define MS2_PIN 26
#define MOTOR_ENABLE_PIN 4
#define EXPANDER_INTERRUPT_PIN 34

const uint8_t motorStepPins[] = {MOTOR1_STEP, MOTOR2_STEP, MOTOR3_STEP,
                                 MOTOR4_STEP, MOTOR5_STEP, MOTOR6_STEP,
                                 MOTOR7_STEP, MOTOR8_STEP};

// i2C expander pins:
#define MOTOR1_DIR 0
#define MOTOR2_DIR 1
#define MOTOR3_DIR 2
#define MOTOR4_DIR 3
#define MOTOR5_DIR 4
#define MOTOR6_DIR 5
#define MOTOR7_DIR 6
#define MOTOR8_DIR 7
#define LIGHTSENSOR1 8
#define LIGHTSENSOR2 9
#define LIGHTSENSOR3 10
#define LIGHTSENSOR4 11
#define LIGHTSENSOR5 12
#define LIGHTSENSOR6 13
#define LIGHTSENSOR7 14
#define LIGHTSENSOR8 15

const uint8_t motorDirPins[] = {MOTOR1_DIR, MOTOR2_DIR, MOTOR3_DIR, MOTOR4_DIR,
                                MOTOR5_DIR, MOTOR6_DIR, MOTOR7_DIR, MOTOR8_DIR};

const uint8_t lightSensorPins[] = {LIGHTSENSOR1, LIGHTSENSOR2, LIGHTSENSOR3,
                                   LIGHTSENSOR4, LIGHTSENSOR5, LIGHTSENSOR6,
                                   LIGHTSENSOR7, LIGHTSENSOR8};