#include "config.h"

#include "I2C_expander.h"
#include "buildFlags.h"
#include "pinout.h"

char installationVariant;
uint8_t nMotors;
uint8_t microStep_setting = MICROSTEP_LOWRES;
uint8_t tmc220x_version = 8;
uint8_t animationNumber;
bool cycleThroughAnimations = true;
float positionOffset = 0;  // variable to create an offset to make the 0.5
                           // position be the exact middle of the installation

void getConfigSettings() {
  uint8_t configByte = 0;
  for (int i = 0; i < 8; i++) {
    if (!digitalReadI2CExpanderPin(configBits[i])) {
      configByte += 1 << i;
    }
  }
#ifdef DEBUG_CONFIG
  Serial.print("ConfigByte: 0b");
  Serial.println(configByte, BIN);
#endif

  // determine the installation variant, stored in the first two bits:
  installationVariant = 'A' + (configByte & 0b11);

  // determine the numebr of motors based on the installation variant
  switch (installationVariant) {
    case 'A':
      // small square variant
      nMotors = 8;
      positionOffset = 0.05;
      break;
    case 'B':
      // small rectangular variant
      nMotors = 7;
      positionOffset = -0.03;
      break;
    case 'C':
      // high variant
      nMotors = 12;
      positionOffset = 0.02;
      break;
    case 'D':
      // wide variant
      nMotors = 6;
      positionOffset = 0;
      break;
    default:
      break;
  }

  // determine the animation number to play:
  animationNumber = (configByte & 0b111100) >>
                    2;  // use the 4 bits after the variant for the animation

  // determine whether to cycle through the animations (set by bit 7):
  if (configByte & 0b1000000) {
    cycleThroughAnimations = false;
  }

  // determine whether the device is in hardware test mode. This overrules all
  // animation cycling and set animations. (set by bit 8)
  bool debugMode = false;
  if (configByte & 0b10000000) {
    debugMode = true;
    cycleThroughAnimations = false;
    animationNumber = 2;  // set the interleaving waves animation
  }

#ifdef DEBUG_CONFIG

  if (debugMode) {
    Serial.println("Hardware Debug mode enabled");
  }
  Serial.print("installationVariant: ");
  Serial.println(installationVariant);
  Serial.print("number of motors: ");
  Serial.println(nMotors);
  Serial.print("Cycle through Animations: ");
  Serial.println(cycleThroughAnimations ? "true" : "false");
  if (!cycleThroughAnimations) {
    Serial.print("Animation to play: ");
    Serial.println(animationNumber);
  }
#endif

  // fixed settings:
  microStep_setting = MICROSTEP_LOWRES;  // resulution is set to a fixed
                                         // fidelity
  tmc220x_version = 9;                   // only use the tmc2209 drivers
}