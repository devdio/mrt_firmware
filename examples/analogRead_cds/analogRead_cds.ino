// https://github.com/espressif/arduino-esp32/blob/master/libraries/ESP32/examples/AnalogRead/AnalogRead.ino
#include <BbBoard.h>

int pin = P0;

void setup() {
  // Set pin mode
  Serial.begin(9600);

  //set the resolution to 12 bits (0-4095)
  analogReadResolution(12);
  // pinMode(P10, INPUT);  
}
 
void loop() {
  int val = analogRead(pin);
  int volts = analogReadMilliVolts(pin);

  // print out the values you read:
  Serial.printf("ADC analog value = %d\n",val);
  delay(500);
 }

