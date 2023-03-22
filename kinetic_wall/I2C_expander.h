#pragma once

#include "globals.h"

extern bool lightSensorChangeFlag;

void setupI2CExpanders();

void digitalWriteI2CExpanderPin(uint16_t pin, bool value);

bool digitalReadI2CExpanderPin(uint16_t pin);
