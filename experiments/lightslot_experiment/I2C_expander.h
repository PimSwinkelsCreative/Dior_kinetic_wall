#pragma once
#include <PCF8575.h>

#include "globals.h"

extern bool lightSensorChangeFlag;

void setLightSensorChangeFlag();


void setupI2CExpander();

void digitalWriteI2CExpanderPin(uint8_t pin, bool value);

bool digitalReadI2CExpanderPin(uint8_t pin);
