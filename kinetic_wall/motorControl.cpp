#include "motorControl.h"

#include "I2C_expander.h"
#include "buildFlags.h"
#include "config.h"
#include "pinout.h"

// #define DEBUG_HOMING

AccelStepperI2CDir* motors[MAX_NUM_MOTORS];

TaskHandle_t motorUpdateTask;

void updateMotorTaskCode(void* pvParameters) {
  for (;;) {
    updateMotors();
    // vTaskDelay(1);
  }
}

void setupMotors() {
  for (int i = 0; i < nMotors; i++) {
    // generate motor objects
    motors[i] = new AccelStepperI2CDir(motorStepPins[i], motorDirPins[i],
                                       SENSORPINS_INVERTED);
    motors[i]->setMaxSpeed(1 * microStep_setting * STEPS_PER_REVOLUTION);
    motors[i]->setAcceleration(1 * microStep_setting * STEPS_PER_REVOLUTION);
    motors[i]->moveTo(0);
    motors[i]->setMinPulseWidth(2);  // time in mircoseconds

    // setup the step pins:
    pinMode(motorStepPins[i], OUTPUT);
  }

  // setup the global control pins
  setMicroSteppingPins();
  enableMotors(true);

  // create the motor update task:
  xTaskCreatePinnedToCore(updateMotorTaskCode, "motorUpdateTask", 100000, NULL,
                          1, &motorUpdateTask, 0);
  disableCore0WDT();  // disable the watchdog on core0 so it can run
                      // continuously
}

void moveMotorToPosition(uint8_t index, float position, float speed,
                         float acceleration) {
  position += positionOffset;
  float speedValue = speed * microStep_setting * STEPS_PER_REVOLUTION;
  float accelValue = acceleration * microStep_setting * STEPS_PER_REVOLUTION;
  long positionValue = position * microStep_setting * STEPS_PER_REVOLUTION;

  motors[index]->setMaxSpeed(speedValue);
  motors[index]->moveTo(positionValue);
  if (acceleration > 0) motors[index]->setAcceleration(accelValue);
}

void moveMotorToNearestPosition(uint8_t index, float position, float speed,
                                float acceleration) {
  float currentPos = float(motors[index]->currentPosition()) /
                     float(STEPS_PER_REVOLUTION * microStep_setting);

  float distanceToTravel = position - currentPos;
#ifdef DEBUG_MOTORCONTROL
  if (index == 0) {
    Serial.print("Motor Channel: " + String(index) + "\tCurrent Position: " +
                 String(currentPos) + "\ttarget Positon: " + String(position) +
                 "\tDistance to travel: " + String(distanceToTravel));
  }
#endif
  // offset the position target so the distance to the current position is
  // maximum 0.5 rotation:
  if (distanceToTravel > 0.5) {
    // position has to be decreased by a number of whole rotations
    position -= int(distanceToTravel + 0.5);

  } else if (distanceToTravel < -0.5) {
    // position has to be increased by a number of whole rotations
    position +=
        int(-distanceToTravel +
            0.5);  // distance to travel is negative, soit needs to be inverted
  }

#ifdef DEBUG_MOTORCONTROL
  if (index == 0) {
    Serial.println("\tadjusted Position: " + String(position));
  }
#endif
  moveMotorToPosition(index, position, speed, acceleration);
}

void moveMotor(uint8_t index, float amount, float speed, float acceleration) {
  float speedValue = speed * microStep_setting * STEPS_PER_REVOLUTION;
  float accelValue = acceleration * microStep_setting * STEPS_PER_REVOLUTION;
  long movementValue = amount * microStep_setting * STEPS_PER_REVOLUTION;
  motors[index]->setMaxSpeed(speedValue);
  motors[index]->move(movementValue);
  if (acceleration > 0) motors[index]->setAcceleration(accelValue);
}

void updateMotors() {
  // first check the sensors if a change has been detected
  // only read the statusses if the motor is homing to save on reads
  if (lightSensorChangeFlag) {
#ifdef DEBUG_HOMING
    Serial.println("sensor change detected!");
#endif
    for (int i = 0; i < nMotors; i++) {
      // save the state of the expander pins, pin
      // polarity is handled by function
      bool sensorPinRead = digitalReadI2CExpanderPin(lightSensorPins[i]);
#ifdef INVERT_SENSORS
      // invert the state if they should stop when
      // the gate is closed. Only used for
      // debugging
      motors[i]->setSensorState(!sensorPinRead);
#else
      motors[i]->setSensorState(sensorPinRead);
#endif
#ifdef DEBUG_HOMING
      Serial.println("sensor " + String(i) +
                     " state: " + String(motors[i]->getSensorState()));
#endif
    }
    lightSensorChangeFlag = false;  // clear the flag
  }

  // Set the zero positions for the motors that are rotating in a positive
  // direction and rotating in a clockwise direction
  for (int i = 0; i < nMotors; i++) {
    if (motors[i]->getSensorTrigger()) {
#ifdef DEBUG_HOMING
      Serial.println("motor " + String(i) + " reached zero position");
#endif
      if (motors[i]->isHoming()) {
        // zero position reached!
        // initial homing, always set the positon
        // to zero:
        motors[i]->setHoming(false);
        motors[i]->setCurrentPosition(0);  // set the current position to be
                                           // the 0 coordinate
      } else {
        if (motors[i]->getDirection()) {
          // zero position reached! round the
          // position to the nearest whole
          // rotation and set that to the current
          // value
          long position = motors[i]->getPosition();
          long stepsPerRotation = STEPS_PER_REVOLUTION * microStep_setting;
          position += stepsPerRotation / 2;  // add a half rotation to the
                                             // current position
          int nFullRotations = position / stepsPerRotation;  // calculate the
                                                             // whole
                                                             // rotations
                                                             // (rounding
                                                             // down)
          position = nFullRotations * stepsPerRotation;
          motors[i]->setCurrentPosition(position);  // set the current
                                                    // position to be an
                                                    // exact amount of full
                                                    // rotations
        }
      }
    }
  }

  // update the motor movement controls:
  for (int i = 0; i < nMotors; i++) {
    // check if the motor is in a homing
    // procedure, if so handle the homing
    // (overwrites all other commands)
    if (motors[i]->isHoming()) {
      motors[i]->move(HOMINGSTEPSINCREMENT);  // move a few
                                              // steps
                                              // relative to
      // the current position
      motors[i]->setAcceleration(HOMINGACCELERATION);
      motors[i]->setMaxSpeed(HOMINGSPEED);
    }
    // update the time untill the next step needs
    // to be executed:
    motors[i]->run();
  }
}

// set the pins based on the globally defined MICROSTEP_SCALE_FACTOR
void setMicroSteppingPins() {
  if (tmc220x_version == 8) {
    switch (microStep_setting) {
      case 2:
        digitalWriteI2CExpanderPin(MOTORS_MS1, HIGH);
        digitalWriteI2CExpanderPin(MOTORS_MS2, LOW);
        break;
      case 4:
        digitalWriteI2CExpanderPin(MOTORS_MS1, LOW);
        digitalWriteI2CExpanderPin(MOTORS_MS2, HIGH);
        break;
      case 8:
        digitalWriteI2CExpanderPin(MOTORS_MS1, LOW);
        digitalWriteI2CExpanderPin(MOTORS_MS2, LOW);
        break;
      case 16:
        digitalWriteI2CExpanderPin(MOTORS_MS1, HIGH);
        digitalWriteI2CExpanderPin(MOTORS_MS2, HIGH);
        break;
      default:
        Serial.print(
            "ERROR, could not set microstepping "
            "pins. setting not available "
            "for TMC220");
        Serial.println(tmc220x_version);
        break;
    }
  } else if (tmc220x_version == 9) {
    switch (microStep_setting) {
      case 8:
        digitalWriteI2CExpanderPin(MOTORS_MS1, LOW);
        digitalWriteI2CExpanderPin(MOTORS_MS2, LOW);
        break;
      case 16:
        digitalWriteI2CExpanderPin(MOTORS_MS1, HIGH);
        digitalWriteI2CExpanderPin(MOTORS_MS2, HIGH);
        break;
      case 32:
        digitalWriteI2CExpanderPin(MOTORS_MS1, HIGH);
        digitalWriteI2CExpanderPin(MOTORS_MS2, LOW);
        break;
      case 64:
        digitalWriteI2CExpanderPin(MOTORS_MS1, LOW);
        digitalWriteI2CExpanderPin(MOTORS_MS2, HIGH);
        break;
      default:
        Serial.print(
            "ERROR, could not set microstepping "
            "pins. setting not available "
            "for TMC220");
        Serial.println(tmc220x_version);
        break;
    }
  } else {
    Serial.println("ERROR, selected unknown MotorDriver");
  }
}

void enableMotors(bool state) {
  digitalWriteI2CExpanderPin(MOTOR_ENABLE, !state);
}

void startMotorHoming(uint8_t motorIndex, bool resetSensors) {
  if (motorIndex < 0 || motorIndex >= nMotors) return;
  motors[motorIndex]->setHoming(true);
  if (resetSensors)
    lightSensorChangeFlag = true;  // force the sensor value to be
                                   // updated
}

AccelStepperI2CDir::AccelStepperI2CDir(uint8_t stepPin, uint8_t dirPin,
                                       bool sensorPinInverted) {
  _stepPin = stepPin;
  _dirPin = dirPin;
  _currentPos = 0;
  _targetPos = 0;
  _speed = 0.0;
  _maxSpeed = 0.0;
  _acceleration = 0.0;
  _sqrt_twoa = 1.0;
  _stepInterval = 0;
  _minPulseWidth = 1;
  _enablePin = 0xff;
  _lastStepTime = 0;
  _stepPinInverted = true;
  _dirPinInverted = false;
  _homingActive = false;
  _sensorDetectFlag = false;
  _prevSensorDetectFlag = false;
  _sensorPinInverted = sensorPinInverted;

  // NEW
  _n = 0;
  _c0 = 0.0;
  _cn = 0.0;
  _cmin = 1.0;
  _direction = DIRECTION_CCW;

  // int i;
  // Some reasonable default
  setAcceleration(1);
  setMaxSpeed(1);
}

void AccelStepperI2CDir::moveTo(long absolute) {
  if (_targetPos != absolute) {
    _targetPos = absolute;
    computeNewSpeed();
    // compute new n?
  }
}

void AccelStepperI2CDir::move(long relative) { moveTo(_currentPos + relative); }

// Implements steps according to the current step interval
// You must call this at least once per step
// returns true if a step occurred
boolean AccelStepperI2CDir::runSpeed() {
  // Dont do anything unless we actually have a step interval
  if (!_stepInterval) return false;

  unsigned long time = micros();
  if (time - _lastStepTime >= _stepInterval) {
    if (_direction == DIRECTION_CW) {
      // Clockwise
      _currentPos += 1;
    } else {
      // Anticlockwise
      _currentPos -= 1;
    }
    step(_currentPos);

    _lastStepTime = time;  // Caution: does not account for
                           // costs in step()

    return true;
  } else {
    return false;
  }
}

// Useful during initialisations or after initial positioning
// Sets speed to 0
void AccelStepperI2CDir::setCurrentPosition(long position) {
  _targetPos = _currentPos = position;
  _n = 0;
  _stepInterval = 0;
  _speed = 0.0;
}

long AccelStepperI2CDir::distanceToGo() { return _targetPos - _currentPos; }

long AccelStepperI2CDir::targetPosition() { return _targetPos; }

long AccelStepperI2CDir::currentPosition() { return _currentPos; }

// Subclasses can override
unsigned long AccelStepperI2CDir::computeNewSpeed() {
  long distanceTo = distanceToGo();  // +ve is clockwise from curent location

  long stepsToStop =
      (long)((_speed * _speed) / (2.0 * _acceleration));  // Equation 16

  if (distanceTo == 0 && stepsToStop <= 1) {
    // We are at the target and its time to stop
    _stepInterval = 0;
    _speed = 0.0;
    _n = 0;
    return _stepInterval;
  }

  if (distanceTo > 0) {
    // We are anticlockwise from the target
    // Need to go clockwise from here, maybe
    // decelerate now
    if (_n > 0) {
      // Currently accelerating, need to decel
      // now? Or maybe going the wrong way?
      if ((stepsToStop >= distanceTo) || _direction == DIRECTION_CCW)
        _n = -stepsToStop;  // Start deceleration
    } else if (_n < 0) {
      // Currently decelerating, need to accel
      // again?
      if ((stepsToStop < distanceTo) && _direction == DIRECTION_CW)
        _n = -_n;  // Start accceleration
    }
  } else if (distanceTo < 0) {
    // We are clockwise from the target
    // Need to go anticlockwise from here, maybe
    // decelerate
    if (_n > 0) {
      // Currently accelerating, need to decel
      // now? Or maybe going the wrong way?
      if ((stepsToStop >= -distanceTo) || _direction == DIRECTION_CW)
        _n = -stepsToStop;  // Start deceleration
    } else if (_n < 0) {
      // Currently decelerating, need to accel
      // again?
      if ((stepsToStop < -distanceTo) && _direction == DIRECTION_CCW)
        _n = -_n;  // Start accceleration
    }
  }

  // Need to accelerate or decelerate
  if (_n == 0) {
    // First step from stopped
    _cn = _c0;
    _direction = (distanceTo > 0) ? DIRECTION_CW : DIRECTION_CCW;
  } else {
    // Subsequent step. Works for accel (n is
    // +_ve) and decel (n is -ve).
    _cn = _cn - ((2.0 * _cn) / ((4.0 * _n) + 1));  // Equation 13
    _cn = max(_cn, _cmin);
  }
  _n++;
  _stepInterval = _cn;
  _speed = 1000000.0 / _cn;
  if (_direction == DIRECTION_CCW) _speed = -_speed;

#if 0
  Serial.println(_speed);
  Serial.println(_acceleration);
  Serial.println(_cn);
  Serial.println(_c0);
  Serial.println(_n);
  Serial.println(_stepInterval);
  Serial.println(distanceTo);
  Serial.println(stepsToStop);
  Serial.println("-----");
#endif
  return _stepInterval;
}

// Run the motor to implement speed and acceleration in order to proceed to
// the target position You must call this at least once per step, preferably
// in your main loop If the motor is in the desired position, the cost is very
// small returns true if the motor is still running to the target position.
boolean AccelStepperI2CDir::run() {
  if (runSpeed()) computeNewSpeed();
  return _speed != 0.0 || distanceToGo() != 0;
}

void AccelStepperI2CDir::setMaxSpeed(float speed) {
  if (speed < 0.0) speed = -speed;
  if (_maxSpeed != speed) {
    _maxSpeed = speed;
    _cmin = 1000000.0 / speed;
    // Recompute _n from current speed and adjust
    // speed if accelerating or cruising
    if (_n > 0) {
      _n = (long)((_speed * _speed) / (2.0 * _acceleration));  // Equation 16
      computeNewSpeed();
    }
  }
}

float AccelStepperI2CDir::maxSpeed() { return _maxSpeed; }

void AccelStepperI2CDir::setAcceleration(float acceleration) {
  if (acceleration == _acceleration) return;
  if (acceleration == 0.0) return;
  if (acceleration < 0.0) acceleration = -acceleration;

  // Recompute _n per Equation 17
  _n = _n * (_acceleration / acceleration);
  // New c0 per Equation 7, with correction per Equation 15
  _c0 = 0.676 * sqrt(2.0 / acceleration) * 1000000.0;  // Equation 15
  _acceleration = acceleration;
  computeNewSpeed();
}

float AccelStepperI2CDir::acceleration() { return _acceleration; }

void AccelStepperI2CDir::setSpeed(float speed) {
  if (speed == _speed) return;
  speed = constrain(speed, -_maxSpeed, _maxSpeed);
  if (speed == 0.0)
    _stepInterval = 0;
  else {
    _stepInterval = fabs(1000000.0 / speed);
    _direction = (speed > 0.0) ? DIRECTION_CW : DIRECTION_CCW;
  }
  _speed = speed;
}

float AccelStepperI2CDir::speed() { return _speed; }

// Subclasses can override
void AccelStepperI2CDir::step(long step) {
  // _pin[0] is step, _pin[1] is direction
  setOutputPins(
      _direction ? 0b10 : 0b00);  // Set direction first else get rogue pulses
  setOutputPins(_direction ? 0b11 : 0b01);  // step HIGH
  // Caution 200ns setup time
  // Delay the minimum allowed pulse width
  delayMicroseconds(_minPulseWidth);
  setOutputPins(_direction ? 0b10 : 0b00);  // step LOW
}

long AccelStepperI2CDir::stepForward() {
  // Clockwise
  _currentPos += 1;
  step(_currentPos);
  _lastStepTime = micros();
  return _currentPos;
}

long AccelStepperI2CDir::stepBackward() {
  // Counter-clockwise
  _currentPos -= 1;
  step(_currentPos);
  _lastStepTime = micros();
  return _currentPos;
}

// You might want to override this to implement eg serial output
// bit 0 of the mask corresponds to step pin
// bit 1 of the mask corresponds to dir pin
// ....
void AccelStepperI2CDir::setOutputPins(uint8_t mask) {
  // always first set the direction pin (will only be set if required to
  // minimize I2C traffic):
  digitalWriteI2CExpanderPin(_dirPin, (mask & 0b10) ? (HIGH ^ _dirPinInverted)
                                                    : (LOW ^ _dirPinInverted));
  // set the step pin:
  digitalWrite(_stepPin, (mask & 0b1) ? (HIGH ^ _stepPinInverted)
                                      : (LOW ^ _stepPinInverted));
}

// Prevents power consumption on the outputs
void AccelStepperI2CDir::disableOutputs() {
  setOutputPins(0);  // Handles inversion automatically
  if (_enablePin != 0xff) {
    pinMode(_enablePin, OUTPUT);
    digitalWrite(_enablePin, LOW ^ _enableInverted);
  }
}

void AccelStepperI2CDir::enableOutputs() {
  pinMode(_stepPin, OUTPUT);

  if (_enablePin != 0xff) {
    pinMode(_enablePin, OUTPUT);
    digitalWrite(_enablePin, HIGH ^ _enableInverted);
  }
}

void AccelStepperI2CDir::setMinPulseWidth(unsigned int minWidth) {
  _minPulseWidth = minWidth;
}

void AccelStepperI2CDir::setEnablePin(uint8_t enablePin) {
  _enablePin = enablePin;

  // This happens after construction, so init pin now.
  if (_enablePin != 0xff) {
    pinMode(_enablePin, OUTPUT);
    digitalWrite(_enablePin, HIGH ^ _enableInverted);
  }
}

void AccelStepperI2CDir::setPinsInverted(bool directionInvert, bool stepInvert,
                                         bool enableInvert) {
  _stepPinInverted = stepInvert;
  _dirPinInverted = directionInvert;
  _enableInverted = enableInvert;
}

// Blocks until the target position is reached and stopped
void AccelStepperI2CDir::runToPosition() {
  while (run()) yield();  // Let system housekeeping occur
}

boolean AccelStepperI2CDir::runSpeedToPosition() {
  if (_targetPos == _currentPos) return false;
  if (_targetPos > _currentPos)
    _direction = DIRECTION_CW;
  else
    _direction = DIRECTION_CCW;
  return runSpeed();
}

// Blocks until the new target position is reached
void AccelStepperI2CDir::runToNewPosition(long position) {
  moveTo(position);
  runToPosition();
}

void AccelStepperI2CDir::stop() {
  if (_speed != 0.0) {
    long stepsToStop = (long)((_speed * _speed) / (2.0 * _acceleration)) +
                       1;  // Equation 16 (+integer rounding)
    if (_speed > 0)
      move(stepsToStop);
    else
      move(-stepsToStop);
  }
}

bool AccelStepperI2CDir::isRunning() {
  return !(_speed == 0.0 && _targetPos == _currentPos);
}

void AccelStepperI2CDir::setHoming(bool value) { _homingActive = value; }
bool AccelStepperI2CDir::isHoming() { return _homingActive; }

void AccelStepperI2CDir::setSensorState(bool state) {
  // copy the current flag to the previous flag:

  if (_sensorPinInverted) {
    _sensorDetectFlag = !state;
  } else {
    _sensorDetectFlag = state;
  }
}

long AccelStepperI2CDir::getPosition() { return _currentPos; }
bool AccelStepperI2CDir::getSensorState() { return _sensorDetectFlag; }
bool AccelStepperI2CDir::getSensorTrigger() {
  // check whether the sensor reading differs from the last sensor reading:
  bool result = _sensorDetectFlag && !_prevSensorDetectFlag;
  _prevSensorDetectFlag = _sensorDetectFlag;  // copy the current sensor reading
                                              // into the previous reading
  return result;
}

bool AccelStepperI2CDir::getDirection() { return _direction == DIRECTION_CW; }
