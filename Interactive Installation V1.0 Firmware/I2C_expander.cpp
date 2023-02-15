#include "I2C_expander.h"

PCF8575 expander(0x20);

bool outputPinStates[NUM_MOTORS];

void setupI2CExpander() {
  expander.begin();
  // Serial.println("I2C expander init");
  for (int i = 0; i < NUM_MOTORS; i++) {
    expander.pinMode(motorDirPins[i], OUTPUT);
    expander.digitalWrite(motorDirPins[i], LOW);
    outputPinStates[i] = false;
    // Serial.println(String(i) + " Set pin " + String(motorDirPins[i]) +
    //                " as output");
  }
  Serial.println("1");
  pinMode(EXPANDER_INTERRUPT_PIN, INPUT_PULLUP);
  Serial.println("2");
}

void digitalWriteI2CExpanderPin(uint8_t pin, bool value) {
  if (pin < 0 || pin >= NUM_MOTORS) return;

  if (outputPinStates[pin] != value) {
    expander.digitalWrite(pin, value);
    outputPinStates[pin] = value;
    Serial.println("Set pin " + String(pin) + " to " + String(value));
  }
}

bool digitalReadI2CExpanderPin(uint8_t pin) {
  return expander.digitalRead(pin);
}