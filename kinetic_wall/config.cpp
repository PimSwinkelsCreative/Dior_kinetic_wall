#include "config.h"
#include "I2C_expander.h"
#include "buildFlags.h"
#include "pinout.h"

uint8_t nMotors;
uint8_t microStep_setting = MICROSTEP_LOWRES;
uint8_t tmc220x_version = 8;
uint8_t animationNumber;

void getConfigSettings() {
    uint8_t configByte = 0;
    for (int i = 0;i < 8;i++) {
        if (!digitalReadI2CExpanderPin(configBits[i])) {
            configByte += 1 << i;
        }
    }
#ifdef DEBUG_CONFIG
    Serial.print("ConfigByte: 0x");
    Serial.println(configByte, HEX);
#endif

    //determine the number of motors, stored in the first 4 bits:
    nMotors = configByte & 0b1111;
#ifdef DEBUG_CONFIG
    Serial.print("Number of motors:: ");
    Serial.println(nMotors);
#endif

    //determine the mircostep setting, stored in the 5th bit:
    //default is MIRCOSTEP_LOWRES
    if (configByte & 0b10000) {
        microStep_setting = MICROSTEP_HIGHRES;
    }
#ifdef DEBUG_CONFIG
    Serial.print("Microstep setting: ");
    Serial.println(microStep_setting);
#endif

    //determine the motordriver variant, stored in the 6th bit:
    //default is 8 (tmc2208)
    if (configByte & 0b100000) {
        tmc220x_version = 9;
    }
#ifdef DEBUG_CONFIG
    Serial.print("Driver version: TMC220");
    Serial.println(tmc220x_version);
#endif

    //determine the animation number to play:
    animationNumber = (configByte & 0b11000000) >> 6;
#ifdef DEBUG_CONFIG
    Serial.print("Animation to play: ");
    Serial.println(animationNumber);
#endif
}