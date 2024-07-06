#include <BbBoard.h>
#include <BbMatrix.h>

const uint8_t sym[5][5] = {
  {0, 1, 1, 1, 0},
  {0, 1, 0, 1, 0},
  {0, 1, 1, 1, 0},
  {0, 1, 0, 1, 0},
  {0, 1, 0, 1, 0}
};

BbMatrix pixels(LED_COUNT, LED_PIN);

void setup() {
  pixels.begin();
  pixels.clear();

  pixels.setBrightness(30);
}

void loop() {

  RGB c = RGB(random(0, 256), random(0, 256), random(0, 256));

  for(int i=0; i<5; i++) {
    for(int j=0; j<5; j++) {
      pixels.displayPixel(i, j, c);
      delay(500);
    }
  }

  c = RGB(random(0, 256), random(0, 256), random(0, 256));
  pixels.displaySymbol(sym, c);
  delay(500);
  pixels.clear();

}
