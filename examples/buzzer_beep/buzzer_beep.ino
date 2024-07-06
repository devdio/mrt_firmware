#include <BbBoard.h>

BbMatrix pixels(LED_COUNT, LED_PIN);
BbBuzzer buzzer(BUZZZER);
BbSwitch swA(SWITCH_A);
BbSwitch swB(SWITCH_B);

void setup() {
  pixels.begin();
  pixels.clear();
  buzzer.begin();
  swA.begin();
  swB.begin();

  Serial.begin(9600);
  delay(200);
}

void loop() {

  int a = swA.read();
  

  if (a) {
    pixels.displayColor(RGB(255, 0, 0));
    buzzer.beep();
    pixels.clear();
  } 
}
