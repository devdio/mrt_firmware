#ifndef BBBOARD_H
#define BBBOARD_H

// #include "BbMatrix.h"  
// #include "BbSwitch.h"
// #include "BbBuzzer.h"
// #include "BbTouch.h"
// #include "BBServo.h"
// 2024.04.18 P4, P11의 GPIO번호 바꿈 
//----------------------------------------
//      SENSOR        
//----------------------------------------
#define BUZZZER     15  // P번호 없슴
#define MPU_SDA     14  // P20
#define MPU_SCL     21  // P19
#define LED_PIN     38  // P8
#define SWITCH_A    40  // P5
#define SWITCH_B    41  // P6
#define TOUCH_0     10  // P0
#define TOUCH_1     4   // P1
#define TOUCH_2     8   // P2
#define MIC_IN      1   // P번호 없슴
#define LIGHT_1     9   // P번호 없슴
#define LIGHT_2     7   // P번호 없슴
#define SERVO       16  // P번호 없슴, 본체 서보모터 
//----------------------------------------
//  BB-mini   -> MICRO::BIT
//----------------------------------------

#define P0    10    //  TOUCH1    -> ANALOG IN
#define P1    4     //  TOUCH3    -> ANALOG IN
#define P2    8     //  TOUCH2    -> ANALOG IN
#define P3    2     //  ADC0      -> ANALOG IN 
// #define P4    9     //  LIGHT_1   -> ANALOG IN
#define P4    47
#define P5    40    //  SWITCH_A  -> BUTTON A
#define P6    41    //  SWITCH_B  ->
#define P7    39    //
#define P8    38    //  LED_PIN   ->
#define P9    5     //            -> ADC4, ANALOG IN 
#define P10   6     //            -> ADC3, ANALOG IN
// #define P11   7     //  LIGHT_1   -> ADC2
#define P11   48
#define P12   18    //
#define P13   12    //            -> SCK
#define P14   13    //            -> MISO
#define P15   11    //            -> MOSI
#define P16   46    //
#define P19   21    //            -> SCL
#define P20   14    //            -> SDA
#define P99   99    //  DUMMY PIN

#define LED_PIN   38
#define LED_COUNT 25

struct RGB {
  uint8_t r;
  uint8_t g;
  uint8_t b;

  RGB(uint8_t red, uint8_t green, uint8_t blue) {
    r = red;
    g = green;
    b = blue;
  }
};


#endif
