#include "I2C_expander.h"

#include <clsPCA9555.h>
#include <Wire.h>

PCA9555 sensorIoExp(SENSOR_EXP_ADDR);
PCA9555 directionIoExp(DIR_EXP_ADDR);
PCA9555 configIoExp(CONFIG_EXP_ADDR);

bool outputPinStates[NUM_MOTORS];

bool lightSensorChangeFlag = false;

void setLightSensorChangeFlag() { lightSensorChangeFlag = true; }

void setupI2CExpanders() {
  sensorIoExp.begin();
  directionIoExp.begin();
  configIoExp.begin();

  Wire.setClock(400000); // set the clock to fast mode

  // configure all expander pins:
  for (int i = 0; i < 16; i++) {
    sensorIoExp.pinMode(i, INPUT);
    directionIoExp.pinMode(i, OUTPUT);
    directionIoExp.digitalWrite(i, LOW);
    if (i < 8) {
      configIoExp.pinMode(i, INPUT);
      configIoExp.digitalWrite(i,LOW);
    }
    else {
      configIoExp.pinMode(i, OUTPUT);
      configIoExp.digitalWrite(i, LOW);
    }
  }


  // setup the hardware interrupt pin
  pinMode(EXPANDER_INTERRUPT_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(EXPANDER_INTERRUPT_PIN),
    setLightSensorChangeFlag, FALLING);
}

void digitalWriteI2CExpanderPin(uint16_t pin, bool value) {
  if (pin < SENSOR_PIN_OFFSET) { Serial.println("ERROR: pin is not an I2C expander pin!"); }
  if (pin < MOTOR_PIN_OFFSET) {
    sensorIoExp.digitalWrite(pin - SENSOR_PIN_OFFSET, value);
  }
  else if (pin < CONFIG_PIN_OFFSET) {
    sensorIoExp.digitalWrite(pin - MOTOR_PIN_OFFSET, value);
  }
  else {
    configIoExp.digitalWrite(pin - CONFIG_PIN_OFFSET, value);
  }
}

bool digitalReadI2CExpanderPin(uint16_t pin) {
  if (pin < SENSOR_PIN_OFFSET) { Serial.println("ERROR: pin is not an I2C expander pin!"); }
  if (pin < MOTOR_PIN_OFFSET) {
    return sensorIoExp.digitalRead(pin - SENSOR_PIN_OFFSET);
  }
  else if (pin < CONFIG_PIN_OFFSET) {
    return sensorIoExp.digitalRead(pin - MOTOR_PIN_OFFSET);
  }
  else {
    return configIoExp.digitalRead(pin - CONFIG_PIN_OFFSET);
  }
}