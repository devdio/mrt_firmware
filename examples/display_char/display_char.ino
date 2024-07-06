#include <BbBoard.h>
#include <BbMatrix.h>

BbMatrix pixels(LED_COUNT, LED_PIN);

void setup() {
  pixels.begin();
  pixels.clear();
}

void loop() {
  pixels.displayChar('A', RGB(255, 0, 0));
  delay(500);
  pixels.displayChar('B', RGB(0, 255, 0));
  delay(500);
  pixels.displayChar('C', RGB(0, 0, 255));
  delay(500);
  pixels.displayChar('D', RGB(255, 0, 0));
  delay(500);
  pixels.displayChar('E', RGB(0, 255, 0));
  delay(500);
  pixels.displayChar('F', RGB(0, 0, 255));
  delay(500);
  pixels.displayChar('G', RGB(255, 0, 0));
  delay(500);
  pixels.displayChar('H', RGB(0, 255, 0));
  delay(500);
  pixels.displayChar('I', RGB(0, 0, 255));
  delay(500);
  pixels.displayChar('J', RGB(255, 0, 0));
  delay(500);
  pixels.displayChar('K', RGB(0, 255, 0));
  delay(500);
  pixels.displayChar('L', RGB(0, 0, 255));
  delay(500);
  pixels.displayChar('M', RGB(0, 0, 255));
  delay(500);
  pixels.displayChar('N', RGB(0, 0, 255));
  delay(500);
  pixels.displayChar('O', RGB(0, 0, 255));
  delay(500);
  pixels.displayChar('P', RGB(0, 0, 255));
  delay(500);
  pixels.displayChar('Q', RGB(0, 0, 255));
  delay(500);
  pixels.displayChar('R', RGB(0, 0, 255));
  delay(500);
  pixels.displayChar('S', RGB(0, 0, 255));
  delay(500);
  pixels.displayChar('T', RGB(0, 0, 255));
  delay(500);
  pixels.displayChar('U', RGB(0, 0, 255));
  delay(500);
  pixels.displayChar('V', RGB(0, 0, 255));
  delay(500);
  pixels.displayChar('W', RGB(0, 0, 255));
  delay(500);
  pixels.displayChar('X', RGB(0, 0, 255));
  delay(500);
  pixels.displayChar('Y', RGB(0, 0, 255));
  delay(500);
  pixels.displayChar('Z', RGB(0, 0, 255));
  delay(500);
  
}
