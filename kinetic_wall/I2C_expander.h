#pragma once
#include <Arduino.h>
extern bool lightSensorChangeFlag;
extern bool I2CBusy;

void IRAM_ATTR setLightSensorChangeFlag();

void setupConfigI2CExpander();

void setupDirectionIoExpander();

void setupSensorIoExpander();

void digitalWriteI2CExpanderPin(uint16_t pin, uint8_t value);

bool digitalReadI2CExpanderPin(uint16_t pin);
