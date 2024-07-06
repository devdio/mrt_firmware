#ifndef NPROTOCOL_H
#define NPROTOCOL_H

// 2023-11-27
// 스크래치쪽의 CommandType과 아두이노쪽의 ACT_ 번호가 다른 것을 주의한다.
// bb-linker쪽에서 변환된다.

#define ACT_NOTHING     0x00
#define ACT_RESET_BOARD 0xC0 
#define ACT_MATRIX_LED  0xC1
#define ACT_BUTTON      0xC2
#define ACT_BUZZER      0xC3
#define ACT_MPU         0xC4
#define ACT_DIGITAL     0xC5
#define ACT_ANALOG      0xC6
#define ACT_ULTRASONIC  0xC7
#define ACT_SERVO       0xC8
#define ACT_TOUCH       0xC9
#define ACT_DCMOTOR     0xCA
#define ACT_TMPHUM      0xCB
#define ACT_LIGHT       0xCC
#define ACT_MIC         0xCD
#define ACT_RCCAR       0xCE    // add for rc-car
#define ACT_MAIN_SERVO  0xD0    
// #define ACT_BOARD_RESET 0xAA
#define ACT_OK          0xFE
#define ACT_ERROR       0xFF


#define BUZZER_BEEP     0x01
#define BUZZER_MELODY   0x02
#define BUZZER_NOTE     0x03

#define DISPLAY_NUM     0x01
#define DISPLAY_CHAR    0x02
#define DISPLAY_SYMBOL  0x03
#define DISPLAY_COLOR   0x04
#define DISPLAY_BRIGHT  0x05
#define DISPLAY_XY      0x06
#define DISPLAY_EFFECT  0x07
#define DISPLAY_ROW     0x08

#define TOUCH_INIT      0x01
#define TOUCH_VALUES    0x02

#define DIGITAL_OUTPUT  0x01
#define DIGITAL_INPUT   0x02
#define DIGITAL_PULLUP  0x03

#define ANALOG_OUTPUT   0x01
#define ANALOG_INPUT    0x02

#define RCCAR_FORWARD       0x01
#define RCCAR_BACKWARD      0x02
#define RCCAR_RLSPEED       0x03
#define RCCAR_STOP          0X04
#define RCCAR_DISTANCE      0x05
#define RCCAR_LINESENSOR    0x06
#define RCCAR_INITIALIZE    0x10

// #define RETURN_LENGTH 0x11
#define RETURN_LENGTH       0x14

#define ANYPIN    0xF0
#define CALLOK    0xF1
#define DEBUG     true


#endif
