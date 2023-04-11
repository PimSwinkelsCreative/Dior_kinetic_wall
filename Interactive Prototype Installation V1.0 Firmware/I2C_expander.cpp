#include "I2C_expander.h"

PCF8575 expander(0x20);

bool outputPinStates[NUM_MOTORS];

bool lightSensorChangeFlag = false;

void setLightSensorChangeFlag() { lightSensorChangeFlag = true; }

void setupI2CExpander() {
  expander.begin();

  // // setup the motor pins:
  for (int i = 0; i < NUM_MOTORS; i++) {
    expander.pinMode(motorDirPins[i], OUTPUT);
    expander.digitalWrite(motorDirPins[i], LOW);
    outputPinStates[i] = false;
  }

  // setup the light sensor pins:
  for (int i = 0; i < NUM_MOTORS; i++) {
    expander.pinMode(lightSensorPins[i], INPUT);
  }

  // setup the hardware interrupt pin
  pinMode(EXPANDER_INTERRUPT_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(EXPANDER_INTERRUPT_PIN),
                  setLightSensorChangeFlag, FALLING);
}

void digitalWriteI2CExpanderPin(uint8_t pin, bool value) {
  if (pin < 0 || pin >= NUM_MOTORS) return;

  if (outputPinStates[pin] != value) {
    expander.digitalWrite(pin, value);
    outputPinStates[pin] = value;
    // Serial.println("Set pin " + String(pin) + " to " + String(value));
  }
}

bool digitalReadI2CExpanderPin(uint8_t pin) {
  bool value = expander.digitalRead(pin);
  return value;
}