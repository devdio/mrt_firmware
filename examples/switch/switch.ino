#include <BbBoard.h>

BbMatrix pixels(LED_COUNT, LED_PIN);
BbSwitch swA(SWITCH_A);
BbSwitch swB(SWITCH_B);

void setup() {
  pixels.begin();
  pixels.clear();

  swA.begin();
  swB.begin();
  
  Serial.begin(9600);
}

void loop() {

  int a = swA.read();
  int b = swB.read();

  if(a && b) {
    pixels.displayColor(RGB(255, 0, 0));
  } else if(a) {
    pixels.displayColor(RGB(0, 255, 0));
  } else if(b) {
    pixels.displayColor(RGB(0, 0, 255));
  } else {
    pixels.clear();
  }
  

}
