#include <BbBoard.h>
#include <BbMatrix.h>

BbMatrix pixels(LED_COUNT, LED_PIN);

void setup() {
  pixels.begin();
  pixels.clear();

  pixels.setBrightness(30);
}

void loop() {
  pixels.displayColor(RGB(255, 0, 0));
  delay(500);
  pixels.displayColor(RGB(255, 255, 0));
  delay(500);
  pixels.displayColor(RGB(255, 255, 255));
  delay(500);

  pixels.displayColor(RGB(0, 255, 255));
  delay(500);
  pixels.displayColor(RGB(0, 0, 255));
  delay(500);
  pixels.displayColor(RGB(0, 0, 0));
  delay(500);
  
}
