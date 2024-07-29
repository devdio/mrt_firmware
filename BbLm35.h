#ifndef BBLM35_H
#define BBLM35_H

#include <Arduino.h>

// 참고 : https://esp32io.com/tutorials/esp32-lm35-temperature-sensor
#define ADC_VREF_mV    3300.0 // in millivolt
#define ADC_RESOLUTION 1023.0


int BbLm35_get(int gpio) {
  // read the ADC value from the temperature sensor
  // int adcVal = analogRead(gpio);
  // convert the ADC value to voltage in millivolt
  // float milliVolt = adcVal * (ADC_VREF_mV / ADC_RESOLUTION);
  // convert the voltage to the temperature in °C
  // float tempC = milliVolt / 10;

  float tempC = (float(analogRead(gpio))*5/(1023))/0.01;
  return tempC;
}



#endif