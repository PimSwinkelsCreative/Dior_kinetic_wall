#include <AccelStepper.h>

#define M0_PIN 27
#define M1_PIN 13
#define M2_PIN 32
// #define STEP_PIN 25
// #define DIR_PIN 26

// second driver:
#define DIR_PIN 23
#define STEP_PIN 22

const int steps_per_rev = 360.0 / 1.8;
const int microStepFactor = 32;

const float animationDuration =
    10;  // time in seconds for a back and forth movement
const float rpm = 2 * 60 / animationDuration;

#define motorInterfaceType 1
AccelStepper motor(motorInterfaceType, STEP_PIN, DIR_PIN);

void setup() {
  Serial.begin(115200);
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(M0_PIN, OUTPUT);
  pinMode(M1_PIN, OUTPUT);
  pinMode(M2_PIN, OUTPUT);

  // configure microstepping:
  digitalWrite(M0_PIN, HIGH);
  digitalWrite(M1_PIN, HIGH);
  digitalWrite(M2_PIN, HIGH);

  // setup motor settings:
  motor.setMaxSpeed(rpm * float(steps_per_rev * microStepFactor) /
                    60.0);  // (mirco)steps per second
  motor.setAcceleration(1000 * microStepFactor);
  motor.moveTo(100 * microStepFactor);
}
void loop() {
  if (motor.distanceToGo() == 0) {
    motor.moveTo(-motor.currentPosition());
  }

  motor.run();
}