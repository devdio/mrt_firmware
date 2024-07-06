#include <BbBoard.h>

// 경보음 
int melody[] = {
     NOTE_C4,   0,        NOTE_C4,  0,        NOTE_C4,  
     NOTE_G4,   NOTE_C4,  0,        NOTE_C4,  0,  
     NOTE_C4,   NOTE_G4
};
int noteDuration[] = {
  8, 16, 8, 16, 8, 4, 8, 16, 8, 16, 8, 4
};


BbMatrix pixels(LED_COUNT, LED_PIN);
BbBuzzer buzzer(BUZZZER);
BbSwitch swA(SWITCH_A);
BbSwitch swB(SWITCH_B);

void setup() {
  pixels.begin();
  pixels.clear();
  buzzer.begin();
  swA.begin();
  swB.begin();

  Serial.begin(9600);
  delay(200);

  
}

void loop() {

  int a = swA.read();
  if (a) {
    for(int i=0; i<12; i++) {
      int dur = 1000 / noteDuration[i];
      buzzer.toneNote(melody[i], dur);
    }
  } 
}
