#ifndef BBTOUCH_H
#define BBTOUCH_H

#include <Arduino.h>

#define THRESHOLD_VALUE 40000

int BbTouch_read(int pin) 
{
  int val = touchRead(pin);
  return val > THRESHOLD_VALUE ? 1 : 0;
}

int BbTouch_raw(int pin) 
{
    int val = touchRead(pin);
    Serial.println(val);
    val = int(val * 0.001);  // 왜그런지 값이 너무 크게 나옴...(32비트 값이라서..)
    return val;
}

#endif
