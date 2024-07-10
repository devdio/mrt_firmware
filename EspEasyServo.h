#ifndef __EspEasyServo_H__
#define __EspEasyServo_H__

#include "Arduino.h"
#include <hal/ledc_types.h>

class EspEasyServo {
public:
  ledc_channel_t _ledc;
  gpio_num_t _gpio;
  uint16_t _frequency;
  uint8_t _bit;
  uint16_t _min;
  uint16_t _max;

  EspEasyServo(ledc_channel_t ledc, gpio_num_t gpio) {
    _ledc = ledc;
    _gpio = gpio;
    _frequency = 50;
    _bit = 10;
    _min = ((0.5 / 20) * (1 << _bit)) + 0.5;  // (0.5ms/20ms)*1024 + 0.5(round)
    _max = ((2.4 / 20) * (1 << _bit)) + 0.5;  // (2.4ms/20ms)*1024 + 0.5(round)

    // https://docs.espressif.com/projects/arduino-esp32/en/latest/api/ledc.html
    // ledcSetup(_ledc, _frequency, _bit);
    // ledcAttachPin(_gpio, _ledc);

    // ledcAttach(_gpio, _frequency, _bit);
    ledcAttachChannel(_gpio, _frequency, _bit, ledc);
  };

  void setServo(uint8_t degree) {
    if (180 < degree) {
      degree = 90;
    }
    ledcWrite(_gpio, _min + ((_max - _min) * degree / 180));
  }
};

#endif
