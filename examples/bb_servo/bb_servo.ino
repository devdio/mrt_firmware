#include "EspEasyServo.h"

EspEasyServo * servo = new EspEasyServo((ledc_channel_t)0, (gpio_num_t)16);

void setup() {
  Serial.begin(115200);

}

void loop() {
  servo->setServo(10);
  delay(1000);
  servo->setServo(90);
  delay(1000);
  servo->setServo(170);
  delay(1000);

  Serial.println("servo moving...");
}
