// https://github.com/espressif/arduino-esp32/blob/master/libraries/ESP32/examples/AnalogRead/AnalogRead.ino
#include <BbBoard.h>


void setup() {
  Serial.begin(115200);
  pinMode(P16, OUTPUT);
}

void loop() {

  for(int i=0; i<250; i=i+10) {
    // set 0%-100%
    analogWrite(P16 , i);
    delay(1000);
  }   
}
