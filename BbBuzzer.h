#ifndef BBBUZZER_H
#define BBBUZZER_H

#include <Arduino.h>

#define NOTE_B0  31   
#define NOTE_C1  33   // 0 -> bb c0
#define NOTE_CS1 35   // 1
#define NOTE_D1  37   // 2
#define NOTE_DS1 39   // 3
#define NOTE_E1  41   // 4
#define NOTE_F1  44   // 5
#define NOTE_FS1 46   // 6
#define NOTE_G1  49   // 7
#define NOTE_GS1 52   // 8
#define NOTE_A1  55   // 9
#define NOTE_AS1 58   // 10
#define NOTE_B1  62   // 11

#define NOTE_C2  65
#define NOTE_CS2 69
#define NOTE_D2  73
#define NOTE_DS2 78
#define NOTE_E2  82
#define NOTE_F2  87
#define NOTE_FS2 93
#define NOTE_G2  98
#define NOTE_GS2 104
#define NOTE_A2  110
#define NOTE_AS2 117
#define NOTE_B2  123
#define NOTE_C3  131
#define NOTE_CS3 139
#define NOTE_D3  147
#define NOTE_DS3 156
#define NOTE_E3  165
#define NOTE_F3  175
#define NOTE_FS3 185
#define NOTE_G3  196
#define NOTE_GS3 208
#define NOTE_A3  220
#define NOTE_AS3 233
#define NOTE_B3  247
#define NOTE_C4  262    // 4옥타브 도 - 36
#define NOTE_CS4 277
#define NOTE_D4  294
#define NOTE_DS4 311
#define NOTE_E4  330
#define NOTE_F4  349
#define NOTE_FS4 370
#define NOTE_G4  392
#define NOTE_GS4 415
#define NOTE_A4  440   // 4옥타브 라 - A4
#define NOTE_AS4 466
#define NOTE_B4  494
#define NOTE_C5  523
#define NOTE_CS5 554
#define NOTE_D5  587
#define NOTE_DS5 622
#define NOTE_E5  659
#define NOTE_F5  698
#define NOTE_FS5 740
#define NOTE_G5  784
#define NOTE_GS5 831
#define NOTE_A5  880
#define NOTE_AS5 932
#define NOTE_B5  988
#define NOTE_C6  1047
#define NOTE_CS6 1109
#define NOTE_D6  1175
#define NOTE_DS6 1245
#define NOTE_E6  1319
#define NOTE_F6  1397
#define NOTE_FS6 1480
#define NOTE_G6  1568
#define NOTE_GS6 1661
#define NOTE_A6  1760
#define NOTE_AS6 1865
#define NOTE_B6  1976
#define NOTE_C7  2093
#define NOTE_CS7 2217
#define NOTE_D7  2349
#define NOTE_DS7 2489
#define NOTE_E7  2637
#define NOTE_F7  2794
#define NOTE_FS7 2960
#define NOTE_G7  3136
#define NOTE_GS7 3322
#define NOTE_A7  3520
#define NOTE_AS7 3729
#define NOTE_B7  3951
#define NOTE_C8  4186
#define NOTE_CS8 4435
#define NOTE_D8  4699
#define NOTE_DS8 4978


// 효과음
// https://github.com/hibit-dev/buzzer/tree/master/src

// 효과음 1
int builtinMelody1[] = { NOTE_C4, NOTE_G3, NOTE_G3, NOTE_A3, NOTE_G3, 
                         0,       NOTE_B3, NOTE_C4 };
int noteDurations1[] = {4, 8, 8, 4, 4, 4, 4, 4};

// 경보음 
int builtinMelody2[] = {
     NOTE_C4,   0,        NOTE_C4,  0,        NOTE_C4,  
     NOTE_G4,   NOTE_C4,  0,        NOTE_C4,  0,  
     NOTE_C4,   NOTE_G4
};
int noteDurations2[] = {
  8, 16, 8, 16, 8, 4, 8, 16, 8, 16, 8, 4
};

int titanicNote[] = {
    NOTE_E4, NOTE_B4, NOTE_E5, NOTE_E5, NOTE_E5, 
    NOTE_B4, NOTE_E4, NOTE_E4, NOTE_B4, NOTE_E5, 
    NOTE_E5, NOTE_E5, NOTE_B4, NOTE_E4, NOTE_E4, 
    NOTE_B4, NOTE_E5, NOTE_E5, NOTE_E5, NOTE_B4, 
};
int titanicDuration[] = {
  8,  8,  8,  8,  8,  
  8,  4,  8,  8,  8,  
  8,  8,  8,  4,  8,  
  8,  8,  8,  8,  8
};

int nyancatNote[] = {
  NOTE_DS5, NOTE_E5,  NOTE_FS5,   0,        NOTE_B5,  
  NOTE_E5,  NOTE_DS5, NOTE_E5,    NOTE_FS5, NOTE_B5,  
  NOTE_DS6, NOTE_E6,  NOTE_DS6,   NOTE_AS5, NOTE_B5,
  0,        NOTE_FS5, 0
};

int nyancatDurations[] = {
  16, 16, 16, 16, 16,
  16, 16, 16, 16, 16, 
  16, 16, 16, 16, 16,
  16, 16, 16
};

int nokiaNote[] = {
  NOTE_E5, NOTE_D5, NOTE_FS4, NOTE_GS4, 
  NOTE_CS5, NOTE_B4, NOTE_D4, NOTE_E4, 
  NOTE_B4, NOTE_A4, NOTE_CS4, NOTE_E4,
  NOTE_A4
};

int nokiaDurations[] = {
  8, 8, 4, 4,
  8, 8, 4, 4,
  8, 8, 4, 4,
  2
};


int melodySize[] = { 8, 12, 20, 18, 13 };
int* builtinMelody[] = {builtinMelody1, builtinMelody2, titanicNote, nyancatNote, nokiaNote };
int* noteDurations[] = {noteDurations1, noteDurations2, titanicDuration, nyancatDurations, nokiaDurations};

int notes[] = {
  NOTE_C1,    NOTE_CS1,   NOTE_D1,    NOTE_DS1,   NOTE_E1,    NOTE_F1,    NOTE_FS1,   NOTE_G1,    NOTE_GS1,   NOTE_A1,    NOTE_AS1,   NOTE_B1,    
  NOTE_C2,    NOTE_CS2,   NOTE_D2,    NOTE_DS2,   NOTE_E2,    NOTE_F2,    NOTE_FS2,   NOTE_G2,    NOTE_GS2,   NOTE_A2,    NOTE_AS2,   NOTE_B2,    
  NOTE_C3,    NOTE_CS3,   NOTE_D3,    NOTE_DS3,   NOTE_E3,    NOTE_F3,    NOTE_FS3,   NOTE_G3,    NOTE_GS3,   NOTE_A3,    NOTE_AS3,   NOTE_B3,    
  NOTE_C4,    NOTE_CS4,   NOTE_D4,    NOTE_DS4,   NOTE_E4,    NOTE_F4,    NOTE_FS4,   NOTE_G4,    NOTE_GS4,   NOTE_A4,    NOTE_AS4,   NOTE_B4,    
  NOTE_C5,    NOTE_CS5,   NOTE_D5,    NOTE_DS5,   NOTE_E5,    NOTE_F5,    NOTE_FS5,   NOTE_G5,    NOTE_GS5,   NOTE_A5,    NOTE_AS5,   NOTE_B5,
  NOTE_C6,    NOTE_CS6,   NOTE_D6,    NOTE_DS6,   NOTE_E6,    NOTE_F6,    NOTE_FS6,   NOTE_G6,    NOTE_GS6 ,  NOTE_A6,    NOTE_AS6,   NOTE_B6,    
  NOTE_C7,    NOTE_CS7,   NOTE_D7,    NOTE_DS7,   NOTE_E7,    NOTE_F7 ,   NOTE_FS7,   NOTE_G7,    NOTE_GS7 ,  NOTE_A7,    NOTE_AS7,   NOTE_B7,    
  NOTE_C8,    NOTE_CS8,   NOTE_DS8 
};

// #define BUZZZER_PIN 15


void BbBuzzer_melody(int channelNo, int idx) {
  for (int thisNote = 0; thisNote < melodySize[idx]; thisNote++) {
    int noteDuration = 1000 / noteDurations[idx][thisNote];
    // tone(gpio, builtinMelody[idx][thisNote], noteDuration);
    ledcWriteTone(channelNo, builtinMelody[idx][thisNote]);

    int pauseBetweenNotes = noteDuration * 1.08;
    delay(pauseBetweenNotes);
    // noTone(gpio);
    ledcWriteTone(channelNo, 0);
  }
}

void BbBuzzer_beep(int channelNo) {
  // tone(gpio, 1046 , 100);
  ledcWriteTone(channelNo, 1046);
  delay(100*1.08);
  // noTone(gpio);
  ledcWriteTone(channelNo, 0);
}

void BbBuzzer_toneNote(int channelNo, int note, int duration) {
  // tone(gpio, notes[note], duration);
  int n = note;
  if (n > 83) {
    n = 83;
  }
  // tone(gpio, notes[n], duration);
  ledcWriteTone(channelNo, notes[n]);
  delay(duration * 1.08);
  // noTone(gpio);
  ledcWriteTone(channelNo, 0);
}

void BbBuzzer_noToneNote(int channelNo){
  // noTone(gpio);
  ledcWriteTone(channelNo, 0);
}


#endif