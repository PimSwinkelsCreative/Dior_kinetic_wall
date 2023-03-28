#include "I2C_expander.h"

#include <Wire.h>
#include <clsPCA9555.h>

#include "config.h"
#include "pinout.h"

PCA9555 sensorIoExp(SENSOR_EXP_ADDR);
PCA9555 directionIoExp(DIR_EXP_ADDR);
PCA9555 configIoExp(CONFIG_EXP_ADDR);

bool lightSensorChangeFlag = false;

uint8_t directionPinStates[MAX_NUM_MOTORS];

bool I2CBusy = false;

void IRAM_ATTR setLightSensorChangeFlag() { lightSensorChangeFlag = true; }

void setupConfigI2CExpander() {
  configIoExp.begin();
  Wire.setClock(400000);  // set the clock to fast mode
  for (int i = 0; i < 16; i++) {
    if (i < 8) {
      configIoExp.pinMode(i, INPUT);
      configIoExp.digitalWrite(i, LOW);
    } else {
      configIoExp.pinMode(i, OUTPUT);
      configIoExp.digitalWrite(i, LOW);
    }
  }
}

void setupDirectionIoExpander() {
  directionIoExp.begin();
  Wire.setClock(400000);  // set the clock to fast mode
  for (int i = 0; i < 16; i++) {
    directionIoExp.pinMode(i, OUTPUT);
    directionIoExp.digitalWrite(i, LOW);
  }
}

void setupSensorIoExpander() {
  sensorIoExp.begin();
  Wire.setClock(400000);  // set the clock to fast mode
  for (int i = 0; i < 16; i++) {
    if (i < nMotors) {
      // pin has a sensor connected, set as input
      sensorIoExp.pinMode(i, INPUT);
    } else {
      // pin is floating, set as output and drive high
      sensorIoExp.pinMode(i, OUTPUT);
      sensorIoExp.digitalWrite(i, HIGH);
    }
  }

  // setup the hardware interrupt pin
  pinMode(EXPANDER_INTERRUPT_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(EXPANDER_INTERRUPT_PIN),
                  setLightSensorChangeFlag, FALLING);
}

void digitalWriteI2CExpanderPin(uint16_t pin, uint8_t value) {
  if (pin < SENSOR_PIN_OFFSET) {
    Serial.println("ERROR: pin is not an I2C expander pin!");
  }
  if (pin < MOTOR_PIN_OFFSET) {
    Serial.println(
        "ERROR: cannot write to sensor pins. Pins are set as output");
    return;
  } else if (pin < CONFIG_PIN_OFFSET) {
    uint8_t dirPin = pin - MOTOR_PIN_OFFSET;
    if (dirPin >= nMotors) {
      Serial.println("ERROR: pin is not connected to a motor");
      return;
    }
    if (directionPinStates[dirPin] != value) {
      while (I2CBusy)
        ;
      I2CBusy = true;
      directionIoExp.digitalWrite(dirPin, value);
      directionPinStates[dirPin] = value;
      I2CBusy = false;
    }
  } else {
    while (I2CBusy)
      ;
    I2CBusy = true;
    configIoExp.digitalWrite(pin - CONFIG_PIN_OFFSET, value);
    I2CBusy = false;
  }
}

bool digitalReadI2CExpanderPin(uint16_t pin) {
  if (pin < SENSOR_PIN_OFFSET) {
    Serial.println("ERROR: pin is not an I2C expander pin!");
  }
  bool result;
  if (pin < MOTOR_PIN_OFFSET) {
    while (I2CBusy)
      ;
    I2CBusy = true;
    result = sensorIoExp.digitalRead(pin - SENSOR_PIN_OFFSET);
    I2CBusy = false;
  } else if (pin < CONFIG_PIN_OFFSET) {
    while (I2CBusy)
      ;
    I2CBusy = true;
    result = sensorIoExp.digitalRead(pin - MOTOR_PIN_OFFSET);
    I2CBusy = false;
  } else {
    while (I2CBusy)
      ;
    I2CBusy = true;
    result = configIoExp.digitalRead(pin - CONFIG_PIN_OFFSET);
    I2CBusy = false;
  }
  return result;
}