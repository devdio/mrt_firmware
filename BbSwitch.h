#ifndef BBSWITCH_H
#define BBSWITCH_H

#include <Arduino.h>

#define SWB_PIN 41
#define SWA_PIN 40
#define SWITCH_ON 0
#define SWITCH_OFF 1


int BbSwitch_read(int pin) {

  // pinMode(_pin, INPUT_PULLUP);

  int val = digitalRead(pin);
  if (val == 0) {
    return SWITCH_OFF;
  } else {
    return SWITCH_ON;
  }
}

#endif
