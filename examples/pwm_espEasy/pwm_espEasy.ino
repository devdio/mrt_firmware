// https://github.com/espressif/arduino-esp32/blob/master/libraries/ESP32/examples/AnalogRead/AnalogRead.ino
#include <BbBoard.h>
#include <EspEasyPWM.h>

EspEasyPWM pwn((ledc_channel_t)0, (gpio_num_t)P0);

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("PWM test");
}

void loop() {

  for(int i=0; i<10; i++) {
    // set 0%-100%
    pwn.setPWM(10*i);
    delay(1000);
  }   
}
