// https://github.com/espressif/arduino-esp32/blob/master/libraries/ESP32/examples/AnalogRead/AnalogRead.ino
#include <BbBoard.h>

void setup() {
  // Set pin mode
  Serial.begin(9600);

  //set the resolution to 12 bits (0-4095)
  analogReadResolution(12);
  pinMode(P12, INPUT);  
}
 
void loop() {
  int val = analogRead(P12);
  // print out the values you read:
  Serial.printf("ADC analog value = %d\n",val);
  delay(500);
 }

