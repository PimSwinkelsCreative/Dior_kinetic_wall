#include <AccelStepper.h>

// settings:
#define USE_POTMETERS

// microstepping pins:
#define M0_PIN 27
#define M1_PIN 13
#define M2_PIN 32

// potmeters:
#define POT1_PIN 4
#define POT2_PIN 14
const int potmeters[] = {POT1_PIN, POT2_PIN};

// motor 1 pins:
#define MOTOR1_STEP_PIN 25
#define MOTOR1_DIR_PIN 26

// motor 2 pins:
#define MOTOR2_DIR_PIN 23
#define MOTOR2_STEP_PIN 22

const int steps_per_rev = 360.0 / 1.8;
const int microStepFactor = 32;

const float minAnimationDuration =
    4;  // fastest possible time in seconds for a back and forth movement
const float rpm = 2 * 60 / minAnimationDuration;

#define motorInterfaceType 1
AccelStepper motor1(motorInterfaceType, MOTOR1_STEP_PIN, MOTOR1_DIR_PIN);
AccelStepper motor2(motorInterfaceType, MOTOR2_STEP_PIN, MOTOR2_DIR_PIN);

AccelStepper motors[] = {motor1, motor2};
const int nMotors = sizeof(motors) / sizeof(motors[0]);

#ifdef USE_POTMETERS
unsigned long lastPotPoll = 0;
unsigned int potMeasureInterval = 100;  // 100ms between polls
unsigned int potMeasurements[nMotors];
#endif

void setup() {
  Serial.begin(115200);
  pinMode(M0_PIN, OUTPUT);
  pinMode(M1_PIN, OUTPUT);
  pinMode(M2_PIN, OUTPUT);
  pinMode(POT1_PIN, INPUT);
  pinMode(POT2_PIN, INPUT);

  // configure microstepping:
  digitalWrite(M0_PIN, HIGH);
  digitalWrite(M1_PIN, HIGH);
  digitalWrite(M2_PIN, HIGH);

  // setup motor settings:
  for (int i = 0; i < nMotors; i++) {
    motors[i].setMaxSpeed(rpm * float(steps_per_rev * microStepFactor) /
                          60.0);  // (mirco)steps per second
    motors[i].setAcceleration(1000 * microStepFactor);
    motors[i].moveTo(100 * microStepFactor);
  }
}

void loop() {
  for (int i = 0; i < nMotors; i++) {
    if (motors[i].distanceToGo() == 0) {
      motors[i].moveTo(-motors[i].currentPosition());
    }
    motors[i].run();
  }

#ifdef USE_POTMETERS
  if (millis() - lastPotPoll > potMeasureInterval) {
    lastPotPoll = millis();
    for (int i = 0; i < nMotors; i++) {
      potMeasurements[i] = analogRead(potmeters[i]);
      motors[i].setMaxSpeed(rpm * ((4096 - potMeasurements[i]) / 4096.0) *
                            float(steps_per_rev * microStepFactor) / 60.0);
      Serial.println("readout: " + String(potMeasurements[i]) +
                     "\tspeed: " + String(motors[i].maxSpeed()));
    }
  }
#endif
}