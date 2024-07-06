// https://github.com/tanakamasayuki/EspEasyUtils
#include <BbBoard.h>


static const int servoPin = P0;
BBServo servo(0, servoPin);  // channel 0 ~ 7

void setup() {
    Serial.begin(9600);
}

void loop() {
  // set 0(min)-90(center)-180(max)
  servo.write(90);
  delay(1000);

  servo.write(0);
  delay(1000);

  servo.write(180);
  delay(1000);

  for (int i = 0; i <= 180; i++) {
    servo.write(i);
    delay(30);
  }
}

