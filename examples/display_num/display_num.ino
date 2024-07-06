#include <BbBoard.h>
#include <BbMatrix.h>

BbMatrix pixels(LED_COUNT, LED_PIN);

void setup() {
  pixels.begin();
  pixels.clear();
}

void loop() {
  pixels.displayNum(0, RGB(255, 0, 0));
  delay(500);
  pixels.displayNum(1, RGB(0, 255, 0));
  delay(500);
  pixels.displayNum(2, RGB(0, 0, 255));
  delay(500);
  pixels.displayNum(3, RGB(255, 0, 0));
  delay(500);
  pixels.displayNum(4, RGB(0, 255, 0));
  delay(500);
  pixels.displayNum(5, RGB(0, 0, 255));
  delay(500);
  pixels.displayNum(6, RGB(255, 0, 0));
  delay(500);
  pixels.displayNum(7, RGB(0, 255, 0));
  delay(500);
  pixels.displayNum(8, RGB(0, 0, 255));
  delay(500);
  pixels.displayNum(9, RGB(255, 0, 0));
  delay(500);
  pixels.displayNum(10, RGB(0, 255, 0));
  delay(500);
  pixels.displayNum(10, RGB(0, 0, 255));
  delay(500);
  
}
