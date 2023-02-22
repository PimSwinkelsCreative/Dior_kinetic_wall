#include "serialInterface.h"

// #define DEBUG_SERIAL_INTERFACE
#define USE_RX_LED

#define BUFFERLENGTH 500
char serialBuffer[BUFFERLENGTH + 1];
unsigned int bufferIndex = 0;

void setupSerialInterface(unsigned int baud) {
  Serial.begin(baud);
#ifdef USE_RX_LED
  // define the onboard led as output to us as a serial RX pin:
  pinMode(LEDPIN, OUTPUT);
#endif
}

void updateSerial() {
  while (Serial.available()) {
#ifdef USE_RX_LED
    digitalWrite(LEDPIN, HIGH);
#endif
    char inChar = Serial.read();
    // convert all to lower case
    if (inChar >= 'A' && inChar <= 'Z') inChar += ('a' - 'A');
    if (inChar == '\n') {
#ifdef DEBUG_SERIAL_INTERFACE
      Serial.println("newline found, about to start parsing the message");
#endif
      serialBuffer[bufferIndex] = '\0';
      parseSerialBuffer();
      bufferIndex = 0;  // reset the index to "clear" the buffer
    } else {
      if (bufferIndex < BUFFERLENGTH) {
        serialBuffer[bufferIndex] = inChar;
        bufferIndex++;
      } else {
        Serial.println("ERROR: serial buffer overflow");
        bufferIndex = 0;
      }
    }
  }
#ifdef USE_RX_LED
  digitalWrite(LEDPIN, LOW);
#endif
}

void parseSerialBuffer() {
  unsigned int scanIndex = 0;
  while (scanIndex < bufferIndex) {
    char message[50];
    int messageIndex = 0;
    while (serialBuffer[scanIndex] != '_' && scanIndex < bufferIndex) {
      message[messageIndex] = serialBuffer[scanIndex];
      scanIndex++;
      messageIndex++;
    }
    message[messageIndex] =
        '\0';     // add a null terminator at the end of the message
    scanIndex++;  // skip the tab for the next loop

#ifdef DEBUG_SERIAL_INTERFACE
    Serial.print("message: ");
    printChararray(message, strlen(message));
#endif
    messageIndex = 0;  // reset the index to start parsing the message

    // chop the message into its separate arguments
    char messageArguments[5][10];
    int argument = 0;
    int argumentIndex = 0;
    for (int i = 0; i < strlen(message); i++) {
      if (message[i] == ';') {
        // null terminate the character array
        messageArguments[argument][argumentIndex] = '\0';
#ifdef DEBUG_SERIAL_INTERFACE
        Serial.print("argument " + String(argument) + ": ");
        printChararray(messageArguments[argument],
                       strlen(messageArguments[argument]));
#endif
        // prepare for the next argument:
        argument++;
        argumentIndex = 0;

      } else {
        messageArguments[argument][argumentIndex] = message[i];
        argumentIndex++;
        if (i == strlen(message) - 1) {
          // null terminate the character array
          messageArguments[argument][argumentIndex] = '\0';
#ifdef DEBUG_SERIAL_INTERFACE
          Serial.print("argument " + String(argument) + ": ");
          printChararray(messageArguments[argument],
                         strlen(messageArguments[argument]));
#endif
        }
      }
    }

#ifdef DEBUG_SERIAL_INTERFACE
    Serial.println("Number of arguments found: " + String(argument));
#endif
    if (strcmp(messageArguments[0], "home") == 0) {
      for (int i = 0; i < NUM_MOTORS; i++) {
        startMotorHoming(i);
      }
    } else if (messageArguments[0][0] == 'm') {
      // first check if all fields are set correctly:
      if (strlen(messageArguments[0]) && strlen(messageArguments[1]) &&
          strlen(messageArguments[2])) {
        uint8_t motorAddress = getIntFromCharArray(messageArguments[0],
                                                   strlen(messageArguments[0]));
        float goalPosition = getfloatFromCharArray(messageArguments[1],
                                                   strlen(messageArguments[1]));
        float motorSpeed = getfloatFromCharArray(messageArguments[2],
                                                 strlen(messageArguments[2]));
        // optional: get the acceleration value and add it to the function
        float motorAcceleration = 0;
        if (strlen(messageArguments[3]) && argument >= 3) {
#ifdef DEBUG_SERIAL_INTERFACE
          Serial.print("argument 3: ");
          printChararray(messageArguments[3], strlen(messageArguments[3]));
#endif
          motorAcceleration = getfloatFromCharArray(
              messageArguments[3], strlen(messageArguments[3]));
        }
        if (motorAddress >= 0 && motorAddress < NUM_MOTORS) {
          moveMotorToPosition(motorAddress, goalPosition, motorSpeed,
                              motorAcceleration);
#ifdef DEBUG_SERIAL_INTERFACE
          Serial.print("Trying to move motor " + String(motorAddress) +
                       " to position " + String(goalPosition) + " with speed " +
                       String(motorSpeed));
          if (motorAcceleration > 0)
            Serial.print(" and acceleration " + String(motorAcceleration));
          Serial.println();
#endif
        } else {
          Serial.println("ERROR: motor index out of range");
        }
      } else {
#ifdef DEBUG_SERIAL_INTERFACE
        Serial.println(
            "ERROR: Missing some arguments, will ignore this command");
#endif
      }
    } else {
      Serial.println("ERROR: unknown command, command is ignored");
    }
  }
}

int getIntFromCharArray(char *sourceArray, int length) {
  char numberArray[length];
  int numberIndex = 0;
  for (int i = 0; i < length; i++) {
    if (sourceArray[i] >= '0' && sourceArray[i] <= '9' ||
        sourceArray[i] == '-' || sourceArray[i] == '.') {
      numberArray[numberIndex] = sourceArray[i];
      numberIndex++;
    }
  }
  numberArray[numberIndex] = '\0';

  return atoi(numberArray);
}

float getfloatFromCharArray(char *sourceArray, int length) {
  char numberArray[length];
  int numberIndex = 0;
  for (int i = 0; i < length; i++) {
    if ((sourceArray[i] >= '0' && sourceArray[i] <= '9') ||
        sourceArray[i] == '-' || sourceArray[i] == '.') {
      numberArray[numberIndex] = sourceArray[i];
      numberIndex++;
    }
  }
  numberArray[numberIndex] = '\0';

  return atof(numberArray);
}

void printChararray(char *arrayToPrint, int length) {
  for (int i = 0; i < length; i++) {
    Serial.print(arrayToPrint[i]);
  }
  Serial.println();
}