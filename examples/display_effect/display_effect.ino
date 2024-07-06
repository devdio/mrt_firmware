#include <BbBoard.h>
#include <BbMatrix.h>

BbMatrix pixels(LED_COUNT, LED_PIN);

void setup() {
  pixels.begin();
  pixels.clear();

  pixels.setBrightness(30);
}

void loop() {
  pixels.effect(0, 2);  // rainbow
  delay(500);
  pixels.effect(1, 2);  //chase rainbow
  delay(500);
  pixels.effect(2, 2); //color wipe
  delay(500);
  
}
