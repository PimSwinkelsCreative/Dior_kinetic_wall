#include "config.h"
#include "I2C_expander.h"

uint8_t nMotors;
uint8_t tmc220x_version;

void getConfigSettings() {
    uint8_t configByte = 0;
    for (int i = 0;i < 8;i++) {
        if (digitalReadI2CExpanderPin(configBits[i])) {
            configByte += 1 << i;
        }
    }
    Serial.print("0b");
    Serial.println(configByte,BIN);
}