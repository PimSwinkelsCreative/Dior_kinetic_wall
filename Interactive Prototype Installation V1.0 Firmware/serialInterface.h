#pragma once
#include "globals.h"
#include "motorControl.h"

void setupSerialInterface(unsigned int baud);

void updateSerial();

void parseSerialBuffer();

int getIntFromCharArray(char *sourceArray, int length);
float getfloatFromCharArray(char *sourceArray, int length);
void printChararray(char *arrayToPrint, int length);