#pragma once

#include "globals.h"

#define STEPS_PER_REVOLUTION 200
#define MICROSTEP_SCALE_FACTOR 8

#define HOMINGACCELERATION (1 * MICROSTEP_SCALE_FACTOR * STEPS_PER_REVOLUTION)
#define HOMINGSPEED (0.1 * STEPS_PER_REVOLUTION * MICROSTEP_SCALE_FACTOR)
#define HOMINGSTEPSINCREMENT \
  1000  // just made it a large number to ensure continuous rotation. set the
        // speed by varying the homingspeed

void setupMotors();

void moveMotorToPosition(uint8_t index, float position, float speed,
                         float acceleration = 0);

void updateMotors();

void setMicroSteppingPins();

void enableMotors(bool state);

void startMotorHoming(uint8_t motorIndex);

class AccelStepperI2CDir {
 public:
  AccelStepperI2CDir(uint8_t stepPin, uint8_t dirPin,
                     bool sensorPinInverted = true);

  /// Set the target position. The run() function will try to move the motor (at
  /// most one step per call) from the current position to the target position
  /// set by the most recent call to this function. Caution: moveTo() also
  /// recalculates the speed for the next step. If you are trying to use
  /// constant speed movements, you should call setSpeed() after calling
  /// moveTo(). \param[in] absolute The desired absolute position. Negative is
  /// anticlockwise from the 0 position.
  void moveTo(long absolute);

  /// Set the target position relative to the current position.
  /// \param[in] relative The desired position relative to the current position.
  /// Negative is anticlockwise from the current position.
  void move(long relative);

  /// Poll the motor and step it if a step is due, implementing
  /// accelerations and decelerations to achieve the target position. You must
  /// call this as frequently as possible, but at least once per minimum step
  /// time interval, preferably in your main loop. Note that each call to run()
  /// will make at most one step, and then only when a step is due, based on the
  /// current speed and the time since the last step. \return true if the motor
  /// is still running to the target position.
  boolean run();

  /// Poll the motor and step it if a step is due, implementing a constant
  /// speed as set by the most recent call to setSpeed(). You must call this as
  /// frequently as possible, but at least once per step interval,
  /// \return true if the motor was stepped.
  boolean runSpeed();

  /// Sets the maximum permitted speed. The run() function will accelerate
  /// up to the speed set by this function.
  /// Caution: the maximum speed achievable depends on your processor and clock
  /// speed. The default maxSpeed is 1.0 steps per second. \param[in] speed The
  /// desired maximum speed in steps per second. Must be > 0. Caution: Speeds
  /// that exceed the maximum speed supported by the processor may Result in
  /// non-linear accelerations and decelerations.
  void setMaxSpeed(float speed);

  /// Returns the maximum speed configured for this stepper
  /// that was previously set by setMaxSpeed();
  /// \return The currently configured maximum speed
  float maxSpeed();

  /// Sets the acceleration/deceleration rate.
  /// \param[in] acceleration The desired acceleration in steps per second
  /// per second. Must be > 0.0. This is an expensive call since it requires a
  /// square root to be calculated. Dont call more ofthen than needed
  void setAcceleration(float acceleration);

  /// Returns the acceleration/deceleration rate configured for this stepper
  /// that was previously set by setAcceleration();
  /// \return The currently configured acceleration/deceleration
  float acceleration();

  /// Sets the desired constant speed for use with runSpeed().
  /// \param[in] speed The desired constant speed in steps per
  /// second. Positive is clockwise. Speeds of more than 1000 steps per
  /// second are unreliable. Very slow speeds may be set (eg 0.00027777 for
  /// once per hour, approximately. Speed accuracy depends on the Arduino
  /// crystal. Jitter depends on how frequently you call the runSpeed()
  /// function. The speed will be limited by the current value of setMaxSpeed()
  void setSpeed(float speed);

  /// The most recently set speed.
  /// \return the most recent speed in steps per second
  float speed();

  /// The distance from the current position to the target position.
  /// \return the distance from the current position to the target position
  /// in steps. Positive is clockwise from the current position.
  long distanceToGo();

  /// The most recently set target position.
  /// \return the target position
  /// in steps. Positive is clockwise from the 0 position.
  long targetPosition();

  /// The current motor position.
  /// \return the current motor position
  /// in steps. Positive is clockwise from the 0 position.
  long currentPosition();

  /// Resets the current position of the motor, so that wherever the motor
  /// happens to be right now is considered to be the new 0 position. Useful
  /// for setting a zero position on a stepper after an initial hardware
  /// positioning move.
  /// Has the side effect of setting the current motor speed to 0.
  /// \param[in] position The position in steps of wherever the motor
  /// happens to be right now.
  void setCurrentPosition(long position);

  /// Moves the motor (with acceleration/deceleration)
  /// to the target position and blocks until it is at
  /// position. Dont use this in event loops, since it blocks.
  void runToPosition();

  /// Executes runSpeed() unless the targetPosition is reached.
  /// This function needs to be called often just like runSpeed() or run().
  /// Will step the motor if a step is required at the currently selected
  /// speed unless the target position has been reached.
  /// Does not implement accelerations.
  /// \return true if it stepped
  boolean runSpeedToPosition();

  /// Moves the motor (with acceleration/deceleration)
  /// to the new target position and blocks until it is at
  /// position. Dont use this in event loops, since it blocks.
  /// \param[in] position The new target position.
  void runToNewPosition(long position);

  /// Sets a new target position that causes the stepper
  /// to stop as quickly as possible, using the current speed and acceleration
  /// parameters.
  void stop();

  /// Disable motor pin outputs by setting them all LOW
  /// Depending on the design of your electronics this may turn off
  /// the power to the motor coils, saving power.
  /// This is useful to support Arduino low power modes: disable the outputs
  /// during sleep and then reenable with enableOutputs() before stepping
  /// again.
  /// If the enable Pin is defined, sets it to OUTPUT mode and clears the pin to
  /// disabled.
  virtual void disableOutputs();

  /// Enable motor pin outputs by setting the motor pins to OUTPUT
  /// mode. Called automatically by the constructor.
  /// If the enable Pin is defined, sets it to OUTPUT mode and sets the pin to
  /// enabled.
  virtual void enableOutputs();

  /// Sets the minimum pulse width allowed by the stepper driver. The minimum
  /// practical pulse width is approximately 20 microseconds. Times less than 20
  /// microseconds will usually result in 20 microseconds or so. \param[in]
  /// minWidth The minimum pulse width in microseconds.
  void setMinPulseWidth(unsigned int minWidth);

  /// Sets the enable pin number for stepper drivers.
  /// 0xFF indicates unused (default).
  /// Otherwise, if a pin is set, the pin will be turned on when
  /// enableOutputs() is called and switched off when disableOutputs()
  /// is called.
  /// \param[in] enablePin Arduino digital pin number for motor enable
  /// \sa setPinsInverted
  void setEnablePin(uint8_t enablePin = 0xff);

  /// Sets the inversion for stepper driver pins
  /// \param[in] directionInvert True for inverted direction pin, false for
  /// non-inverted \param[in] stepInvert      True for inverted step pin, false
  /// for non-inverted \param[in] enableInvert    True for inverted enable pin,
  /// false (default) for non-inverted
  void setPinsInverted(bool directionInvert = false, bool stepInvert = false,
                       bool enableInvert = false);

  /// Checks to see if the motor is currently running to a target
  /// \return true if the speed is not zero or not at the target position
  bool isRunning();

  /// Virtual destructor to prevent warnings during delete
  virtual ~AccelStepperI2CDir(){};

  void setHoming(bool value);

  bool isHoming();

  // function that sets the state of the zero position sensor. state is true if
  // the sensor is at the zero position
  void setSensorState(bool state);

  // function that returns the state of the zero position sensor value.
  // state is true if the sensor is at the zero position
  bool getSensorState();

 protected:
  /// \brief Direction indicator
  /// Symbolic names for the direction the motor is turning
  typedef enum {
    DIRECTION_CCW = 0,  ///< Counter-Clockwise
    DIRECTION_CW = 1    ///< Clockwise
  } Direction;

  /// Forces the library to compute a new instantaneous speed and set that as
  /// the current speed. It is called by
  /// the library:
  /// \li  after each step
  /// \li  after change to maxSpeed through setMaxSpeed()
  /// \li  after change to acceleration through setAcceleration()
  /// \li  after change to target position (relative or absolute) through
  /// move() or moveTo()
  /// \return the new step interval
  virtual unsigned long computeNewSpeed();

  /// Low level function to set the motor output pins
  /// bit 0 of the mask corresponds to _pin[0]
  /// bit 1 of the mask corresponds to _pin[1]
  /// You can override this to impment, for example serial chip output insted of
  /// using the output pins directly
  virtual void setOutputPins(uint8_t mask);

  /// Called to execute a step. Only called when a new step is
  /// required. Subclasses may override to implement new stepping
  /// interfaces. The default calls step1(), step2(), step4() or step8()
  /// depending on the number of pins defined for the stepper. \param[in] step
  /// The current step phase number (0 to 7)
  virtual void step(long step);

  /// Called to execute a clockwise(+) step. Only called when a new step is
  /// required. This increments the _currentPos and calls step()
  /// \return the updated current position
  long stepForward();

  /// Called to execute a counter-clockwise(-) step. Only called when a new step
  /// is required. This decrements the _currentPos and calls step() \return the
  /// updated current position
  long stepBackward();

  /// Current direction motor is spinning in
  /// Protected because some peoples subclasses need it to be so
  boolean _direction;  // 1 == CW

  /// The current interval between steps in microseconds.
  /// 0 means the motor is currently stopped with _speed == 0
  unsigned long _stepInterval;

 private:
  uint8_t _stepPin;  // esp32 pin that sets the steps
  uint8_t _dirPin;  // I2C expander pin that controls the direction of the motor

  boolean _stepPinInverted;
  boolean _dirPinInverted;

  /// The current absolution position in steps.
  long _currentPos;  // Steps

  /// The target position in steps. The AccelStepper library will move the
  /// motor from the _currentPos to the _targetPos, taking into account the
  /// max speed, acceleration and deceleration
  long _targetPos;  // Steps

  /// The current motos speed in steps per second
  /// Positive is clockwise
  float _speed;  // Steps per second

  /// The maximum permitted speed in steps per second. Must be > 0.
  float _maxSpeed;

  /// The acceleration to use to accelerate or decelerate the motor in steps
  /// per second per second. Must be > 0
  float _acceleration;
  float _sqrt_twoa;  // Precomputed sqrt(2*_acceleration)

  /// The last step time in microseconds
  unsigned long _lastStepTime;

  /// The minimum allowed pulse width in microseconds
  unsigned int _minPulseWidth;

  /// Is the enable pin inverted?
  bool _enableInverted;

  /// Enable pin for stepper driver, or 0xFF if unused.
  uint8_t _enablePin;

  /// The pointer to a forward-step procedure
  void (*_forward)();

  /// The pointer to a backward-step procedure
  void (*_backward)();

  /// The step counter for speed calculations
  long _n;

  /// Initial step size in microseconds
  float _c0;

  /// Last step size in microseconds
  float _cn;

  /// Min step size in microseconds based on maxSpeed
  float _cmin;  // at max speed

  // flag to see if the homing procedure is active
  bool _homingActive;

  // bool that holds the light detector status:
  bool _sensorDetectFlag;

  bool _sensorPinInverted;
};