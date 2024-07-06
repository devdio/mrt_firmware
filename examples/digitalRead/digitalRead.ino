#include <BbBoard.h>

int PINS[] = {P0, P1, P2, P3, P4, P7, P11, P12};
int no = 0;

void setup() {
  // Set pin mode

  pinMode(P0, INPUT_PULLUP);
  Serial.begin(9600);
}
 
void loop() {
  int val = digitalRead(P0);
  Serial.println(val);
  delay(500);
 }
