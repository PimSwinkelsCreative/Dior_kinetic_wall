#include <Arduino.h>
#include <PCF8575.h>

#include "pinout.h"

#define NUM_MOTORS 8
#define STEPS_PER_ROTATION 200
#define MICROSTEP_SCALE_FACTOR 16

// Set i2c address
PCF8575 expander(0x20);

void setup() {
  Serial.begin(115200);
  //   expander init
  expander.begin();
  for (int i = 0; i < NUM_MOTORS; i++) {
    expander.pinMode(motorDirPins[i], OUTPUT);
    if (i % 2 == 0) {
      expander.digitalWrite(motorDirPins[i], HIGH);
      Serial.println("set expander pin " + String(motorDirPins[i]) + " high");
    } else {
      expander.digitalWrite(motorDirPins[i], LOW);
      Serial.println("set expander pin " + String(motorDirPins[i]) + " low");
    }
  }

  // step pin init:
  for (int i = 0; i < NUM_MOTORS; i++) {
    pinMode(motorStepPins[i], OUTPUT);
  }

  // configure microstepping pins:
  pinMode(MS1_PIN, OUTPUT);
  pinMode(MS2_PIN, OUTPUT);
  digitalWrite(MS1_PIN, HIGH);
  digitalWrite(MS2_PIN, HIGH);

  // enable the motors:
  pinMode(MOTOR_ENABLE_PIN, OUTPUT);
  digitalWrite(MOTOR_ENABLE_PIN, LOW);
}

void loop() {
  for (int i = 0; i < NUM_MOTORS; i++) {
    digitalWrite(motorStepPins[i], HIGH);
  }
  delayMicroseconds(50);
  for (int i = 0; i < NUM_MOTORS; i++) {
    digitalWrite(motorStepPins[i], LOW);
  }
  delayMicroseconds(200);
  //   Serial.println(millis());
}