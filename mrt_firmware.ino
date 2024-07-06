/*--------------------------------------------
// VERSION
// 20240328  ver 2.0.1
--------------------------------------------*/
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
// #include <Adafruit_NeoPixel.h>
// #define MPU6050_TOCKN
// #define BITBLOCK_DEBUG
#include "EspEasyServo.h"
#include "EspEasyPWM.h"
#include "Wire.h"

#ifdef MPU6050_TOCKN
  #include <MPU6050_tockn.h>
#else
  #include "MPU6050_light.h"
#endif

#include <DHT11.h>
// #include <NewPing.h>

#include "BbBoard.h"
#include "BbProtocol.h"
#include "BbMatrix.h"
#include "BbBuzzer.h"
#include "BbSwitch.h"
#include "BbTouch.h"

// Firmup 프로그램과 버전 맞춤
#define FIRMWARE_VERSION  "Ver 3.0.8"

// --------------------------------------------
// RC CAR START -------------------------------
// MOTOR --------------------------------------
#define LEFT_MOTOR_DIR_PIN    12
#define RIGHT_MOTOR_DIR_PIN   11

#define LEFT_MOTOR_PWM_PIN    13
#define RIGHT_MOTOR_PWM_PIN   46

#define LEFT_MOTOR_FREQ       1000
#define RIGHT_MOTOR_FREQ      1000

#define LEFT_MOTOR_PWM_CHANNEL   6      // 채널은 0 ~ 7
#define RIGHT_MOTOR_PWM_CHANNEL  7

#define LEFT_MOTOR_PWM_ROSOLUTION  8
#define RIGHT_MOTOR_PWM_ROSOLUTION 8
// Line Sensor ---------------------------------
#define LINE_LED_ONOFF_PIN        18

#define LEFT_LINE_SENSOR_PIN      8
#define CENTER_LINE_SENSOR_PIN    6
#define RIGHT_LINE_SENSOR_PIN     4
// Distance -----------------------------------
#define SONIC_ECHO_PIN    5
#define SONIC_TRIG_PIN    39
#define MAX_DISTANCE      200

// RC CAR END ----------------------------------
// --------------------------------------------
#define LED_INIT_BRIGHT     20
#define BB_PROTOCOL_LEN     20

const int ULTRASONIC_TIMEOUT_MICRO = 500000;  // 500 millis secs
bool rcInitialize = false;
bool touchInitialize = false;
bool notifyFlag = false;
// bool reportFlag = false;


// 버퍼 스크래치 -> ESP32로 보낸 데이터를 저장 
// char buffer[52];
char rxBuffer[BB_PROTOCOL_LEN];
// unsigned char prevc = 0;

// ESP32 -> 스크래치로 보낼 데이터 저장 
uint8_t returnBuffer[BB_PROTOCOL_LEN];

// ESP32 -> 스크래치로 각 핀의 리포터를 송신
uint8_t reportBuffer[BB_PROTOCOL_LEN];


// 20240227
// uint8_t AVAILABLE_PINS[] = {P0, P1, P2, P3, P4, P7, P11, P12};
// P0, P1, P2핀은 터치핀 전용
uint8_t AVAILABLE_PINS_LENGTH = 5;
uint8_t AVAILABLE_PINS[] = {P3, P4, P7, P11, P12};
// +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-

// SemaphoreHandle_t  serial_mutex;
unsigned long previousMillis = millis();

// val Union
union {
  byte byteVal[4];
  float floatVal;
  long longVal;
} val;

// valShort Union
union {
  byte byteVal[2];
  short shortVal;
} valShort;

uint8_t txValue[] = { 0xFF, 0x55, RETURN_LENGTH, 0, ACT_NOTHING, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0x0D, 0x0A };
uint8_t reportValue[] = { 0xFF, 0x66, RETURN_LENGTH, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0x0D, 0x0A };
// char txValue[] = {'A', 'B', 'C', 'D', 'E', 'F', 'G', 'H', 'I', 'J', 'K', 'L', 'M', 'N', 'O', 'P', 'Q', 'R', 'S', 'T'};
// BbMatrix pixels(LED_COUNT, LED_PIN);

int BbMatrix_nBrigtness = 30;
Adafruit_NeoPixel _pixels = Adafruit_NeoPixel(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);

MPU6050 _mpu(Wire);
TaskHandle_t mpuTaskHandle;
TaskHandle_t lightTaskHandle;
// TaskHandle_t dataTaskHandle;

int _tl = 0;
int _tr = 0;
int _tu = 0;
int _td = 0;

// 빛센서 이동평균
const int _numLightReadings = 10;
int _light_readIndex = 0;
int _light1_readings[_numLightReadings];
int _light2_readings[_numLightReadings];
int _light1_total = 0;
int _light2_total = 0;
int _light1_average = 0;
int _light2_average = 0;

// unsigned long timer = 0;
// float _angleX;
// float _angleY;

// RCCar

// NewPing * rcSonar = NULL;



byte idx = 0;
byte dataLen;

double lastTime = 0.0;
double currentTime = 0.0;

uint8_t cmd_idx = 0;
int action = 0;

boolean isStart = false;
boolean isUltrasonic = false;

unsigned int testVal = 0;

// ESP32 채널은 0 ~ 7
int get_channel(int pin) {
  if(pin == P3) {
    return LEDC_CHANNEL_0;
  } else if(pin == P4) {
    return LEDC_CHANNEL_1;
  } else if(pin == P7) {
    return LEDC_CHANNEL_2;
  } else if(pin == P11) {
    return LEDC_CHANNEL_3;
  } else if(pin == P12) {
    return LEDC_CHANNEL_4;
  } else if(pin == P16) {
    return LEDC_CHANNEL_5;
  } 
}

#define CHANNEL_LENGTH 6
int servo_pins[] = {0, 0, 0, 0, 0, 0};   // 핀번호 
EspEasyServo* reserved_servo[] = {NULL, NULL, NULL, NULL, NULL, NULL};
// EspEasyServo* _mainServo = new EspEasyServo((ledc_channel_t)LEDC_CHANNEL_6, (gpio_num_t)SERVO);
EspEasyServo* _mainServo = NULL;


// 채널 0, 1, 2, 3, 4, 5, 6, 7 채널은 총 8개가 지원되는 걸로 알고 있는데...  
// ledc_channel_t channels[] = {LEDC_CHANNEL_0, LEDC_CHANNEL_1, LEDC_CHANNEL_2, LEDC_CHANNEL_3, LEDC_CHANNEL_4, LEDC_CHANNEL_5, LEDC_CHANNEL_6, LEDC_CHANNEL_7};

EspEasyServo* findServo(int pin){
  for(int i = 0; i < CHANNEL_LENGTH; i++){
    if(servo_pins[i] == pin){
      return reserved_servo[i];
    }
  }

  for(int i = 0; i < CHANNEL_LENGTH; i++){
    if(servo_pins[i] == 0){
      servo_pins[i] = pin;
      int ch = get_channel(pin);
      reserved_servo[i] = new EspEasyServo((ledc_channel_t)ch, (gpio_num_t)pin);
      return reserved_servo[i];
    }
  }
  return NULL;
}


int dht_pins[] = {0, 0, 0, 0, 0, 0}; 
DHT11* reserved_dhts[] = {NULL, NULL, NULL, NULL, NULL, NULL};

DHT11 * findDHT11(int pin){
  for(int i = 0; i < CHANNEL_LENGTH; i++){
    if(dht_pins[i] == pin){
      return reserved_dhts[i];
    }
  }

  for(int i = 0; i < CHANNEL_LENGTH; i++){
    if(dht_pins[i] == 0){
      dht_pins[i] = pin;
      reserved_dhts[i] = new DHT11(pin);;
      return reserved_dhts[i];
    }
  }
  return NULL;
}

int pwm_pins[] = {0, 0, 0, 0, 0, 0}; 
EspEasyPWM * reserved_pwm[] = {NULL, NULL, NULL, NULL, NULL, NULL};

EspEasyPWM * findPWM(int pin){
  for(int i = 0; i < CHANNEL_LENGTH; i++){
    if(pwm_pins[i] == pin){
      return reserved_pwm[i];
    }
  }

  for(int i = 0; i < CHANNEL_LENGTH; i++){
    if(pwm_pins[i] == 0){
      pwm_pins[i] = pin;
      int ch = get_channel(pin);
      reserved_pwm[i] = new EspEasyPWM((ledc_channel_t)ch, (gpio_num_t)pin);
      return reserved_pwm[i];
    }
  }
  return NULL;
}


unsigned char readBuffer(int index){
  return rxBuffer[index];
}


void writeBuffer(int idx, unsigned char c){
  rxBuffer[idx] = c;
}

// for Serial
// void writeSerial(unsigned char c){
//   Serial.write(c);
// }

// for BLE
void writeSerial(int idx, unsigned char c){
  returnBuffer[idx] = (uint8_t) c;
}

// for BLE
void sendPacket() {
  memcpy(txValue, returnBuffer, BB_PROTOCOL_LEN);
  notifyFlag = true;
}



void callNoResponse(uint8_t action){
  // 응답을 보내지않음.
  // writeHead(0);  // ff, 55
  // writeSerial(2, RETURN_LENGTH);   // lenght
  // writeSerial(3, 0);               // cmd_idx을 0으로 보내면 스크래치에서 무시된다.
  // writeSerial(4, action);
  // writeSerial(5, 0x00);  
  // writeSerial(6, 0x00);
  // writeSerial(7, 0x00);
  // writeSerial(8, 0x00);
  // writeSerial(9, 0x00);
  // writeSerial(10, 0x00);
  // writeSerial(11, 0x00);
  // writeSerial(12, 0x00);
  // writeSerial(13, 0x00);
  // writeSerial(14, 0x00);
  // writeSerial(15, 0x00);
  // writeSerial(16, 0x00);
  // writeSerial(17, 0x00);
  // writeEnd(18);  // 13, 10
}

// 스크래치에 모든 데이터를 0으로 보낸다.
void callZERO(uint8_t action){
  
    writeHead(0);  // ff, 55
    writeSerial(2, RETURN_LENGTH);   // lenght
    writeSerial(3, cmd_idx);
    writeSerial(4, action);
    writeSerial(5, 0x00);  
    writeSerial(6, 0x00);
    writeSerial(7, 0x00);
    writeSerial(8, 0x00);
    writeSerial(9, 0x00);
    writeSerial(10, 0x00);
    writeSerial(11, 0x00);
    writeSerial(12, 0x00);
    writeSerial(13, 0x00);
    writeSerial(14, 0x00);
    writeSerial(15, 0x00);
    writeSerial(16, 0x00);
    writeSerial(17, 0x00);
    writeEnd(18);  // 13, 10
}

void callNG(uint8_t action) {
  
  writeHead(0);  // ff, 55
  writeSerial(2, RETURN_LENGTH);  // lenght
  writeSerial(3, cmd_idx);
  writeSerial(4, action);
  writeSerial(5, ACT_ERROR);  
  writeSerial(6, 0x00);
  writeSerial(7, 0x00);
  writeSerial(8, 0x00);
  writeSerial(9, 0x00);
  writeSerial(10, 0x00);
  writeSerial(11, 0x00);
  writeSerial(12, 0x00);
  writeSerial(13, 0x00);
  writeSerial(14, 0x00);
  writeSerial(15, 0x00);
  writeSerial(16, 0x00);
  writeSerial(17, 0x00);
  writeEnd(18);  // 13, 10
  sendPacket();
}

void callOK(uint8_t action) {
  switch(action) {
    case ACT_MATRIX_LED:
    case ACT_BUZZER:
    case ACT_ANALOG:
    case ACT_RESET_BOARD:
    case ACT_RCCAR:
    case ACT_TOUCH:
    default:
    {
      writeHead(0);  // ff, 55
      writeSerial(2, RETURN_LENGTH);  // lenght
      writeSerial(3, cmd_idx);
      writeSerial(4, action);
      writeSerial(5, ACT_OK);  
      writeSerial(6, 0x00);
      writeSerial(7, 0x00);
      writeSerial(8, 0x00);
      writeSerial(9, 0x00);
      writeSerial(10, 0x00);
      writeSerial(11, 0x00);
      writeSerial(12, 0x00);
      writeSerial(13, 0x00);
      writeSerial(14, 0x00);
      writeSerial(15, 0x00);
      writeSerial(16, 0x00);
      writeSerial(17, 0x00);
      writeEnd(18);  // 13, 10
      sendPacket();
    }
  }
}

void writeHead(int i){
  writeSerial(i, 0xff);
  writeSerial(i+1, 0x55);
}

void writeEnd(int i){
  // 13(0d), 10(0a)
  writeSerial(i, 0x0d);
  writeSerial(i+1, 0x0a);
}

void sendFloat(int i, float value){
  val.floatVal = value;
  writeSerial(i, val.byteVal[0]);
  writeSerial(i+1, val.byteVal[1]);
  writeSerial(i+2, val.byteVal[2]);
  writeSerial(i+3, val.byteVal[3]);
}

void sendShort(int i, double value){
  valShort.shortVal = value;
  writeSerial(i, valShort.byteVal[0]);    // low
  writeSerial(i+1, valShort.byteVal[1]);  // hight
}

// void writeShort(int i, short value){
//   byte lowerByte = value & 0xFF;
//   byte upperByte = (value >> 8) & 0xFF;
//   writeSerial(i, upperByte);
//   writeSerial(i+1, lowerByte);
//   Serial.println(value);
//   Serial.println(String(upperByte, HEX));
//   Serial.println(String(lowerByte, HEX));
// }

short readShort(int idx){
  valShort.byteVal[0] = readBuffer(idx);
  valShort.byteVal[1] = readBuffer(idx + 1);
  return valShort.shortVal;
}

float readFloat(int idx){
  val.byteVal[0] = readBuffer(idx);
  val.byteVal[1] = readBuffer(idx + 1);
  val.byteVal[2] = readBuffer(idx + 2);
  val.byteVal[3] = readBuffer(idx + 3);
  return val.floatVal;
}

long readLong(int idx){
  val.byteVal[0] = readBuffer(idx);
  val.byteVal[1] = readBuffer(idx + 1);
  val.byteVal[2] = readBuffer(idx + 2);
  val.byteVal[3] = readBuffer(idx + 3);
  return val.longVal;
}


// DISPLAY_NUM =    0x01      # 0 ~ 9
// DISPLAY_CHAR =   0x02     # A ~ Z
// DISPLAY_SYMBOL = 0x03   # 0b11111, 0b00000, 0b00000, 0b00000, 0b11111, 
// DISPLAY_COLOR =  0x04
// DISPLAY_BRIGHT = 0x05
// DISPLAY_XY =     0x06
// DISPLAY_EFFECT = 0x07
// DISPLAY_ROW = 0x08
void processMatrix() {
  int command = readBuffer(5);

  switch(command){
    case DISPLAY_NUM:
      {
        BbMatrix_clear(&_pixels);
        delay(10);

        char c = readBuffer(6);
        uint8_t R = (uint8_t)readBuffer(7);
        uint8_t G = (uint8_t)readBuffer(8);
        uint8_t B = (uint8_t)readBuffer(9);
        BbMatrix_displayNum(&_pixels, c, RGB(R, G, B));
        // pixels.displayNum(c, RGB(R, G, B));
        callOK(ACT_MATRIX_LED);
      }
      break;
    case DISPLAY_CHAR:
      {
        BbMatrix_clear(&_pixels);
        delay(10);

        char c = readBuffer(6);
        uint8_t R = (uint8_t)readBuffer(7);
        uint8_t G = (uint8_t)readBuffer(8);
        uint8_t B = (uint8_t)readBuffer(9);
        BbMatrix_displayChar(&_pixels, c, RGB(R, G, B));
        // pixels.displayChar(c, RGB(R, G, B));
        callOK(ACT_MATRIX_LED);
      }
      break;
    case DISPLAY_SYMBOL:
      {
        BbMatrix_clear(&_pixels);
        delay(10);

        char lines[] = {readBuffer(6), readBuffer(7), readBuffer(8), readBuffer(9), readBuffer(10)};
        uint8_t R = (uint8_t)readBuffer(11);
        uint8_t G = (uint8_t)readBuffer(12);
        uint8_t B = (uint8_t)readBuffer(13);
        BbMatrix_display(&_pixels, lines, RGB(R, G, B));
        // pixels.display(lines, RGB(R, G, B));
        callOK(ACT_MATRIX_LED);
      }
      break;
    case DISPLAY_ROW:
      {
        char lines[] = {readBuffer(6)};
        uint8_t R = (uint8_t)readBuffer(7);
        uint8_t G = (uint8_t)readBuffer(8);
        uint8_t B = (uint8_t)readBuffer(9);
        uint8_t row = (uint8_t)readBuffer(10);
        BbMatrix_displayRow(&_pixels, row, lines, RGB(R, G, B));
        // pixels.display(lines, RGB(R, G, B));
        callOK(ACT_MATRIX_LED);
      }
      break;
    case DISPLAY_COLOR:
      {
        BbMatrix_clear(&_pixels);
        delay(10);

        uint8_t R = (uint8_t)readBuffer(6);
        uint8_t G = (uint8_t)readBuffer(7);
        uint8_t B = (uint8_t)readBuffer(8);
        BbMatrix_displayColor(&_pixels, RGB(R, G, B));
        callOK(ACT_MATRIX_LED);
      }
      break;  
    case DISPLAY_BRIGHT:
      {
        uint8_t val = (uint8_t)readBuffer(6);
        BbMatrix_nBrigtness = val;
        BbMatrix_setBrightness(&_pixels, BbMatrix_nBrigtness);
        // pixels.setBrightness(val);
        callOK(ACT_MATRIX_LED);
      }
      break;
    case DISPLAY_XY:
      {
        uint8_t R = (uint8_t)readBuffer(6);
        uint8_t G = (uint8_t)readBuffer(7);
        uint8_t B = (uint8_t)readBuffer(8);

        uint8_t coordX = (uint8_t)readBuffer(9);
        uint8_t coordY = (uint8_t)readBuffer(10);
        BbMatrix_displayPixel(&_pixels, coordX, coordY, RGB(R, G, B));
        // pixels.displayPixel(coordX, coordY, RGB(R, G, B));
         callOK(ACT_MATRIX_LED);
      }
      break;
    case DISPLAY_EFFECT:
      {
        BbMatrix_clear(&_pixels);
        delay(10);
        
        uint8_t idx = (uint8_t)readBuffer(6);
        uint8_t wait = (uint8_t)readBuffer(7);
        BbMatrix_effect(&_pixels, idx, wait);
        // pixels.effect(idx, wait);
        callNoResponse(ACT_MATRIX_LED);
      }
      break;
    derfault:
      break;
  }
}

// Ultra echo  P9, 5
// Ultra trig  P7, 39
// #define SONIC_ECHO_PIN    5
// #define SONIC_TRIG_PIN    39
// #define MAX_DISTANCE      200

void rccar_ultrasonic() {
  // int distance = rcSonar->ping_cm();

  digitalWrite(SONIC_TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(SONIC_TRIG_PIN, LOW);
  
  long duration = pulseIn(SONIC_ECHO_PIN, HIGH, ULTRASONIC_TIMEOUT_MICRO);
  int distance = duration * 0.017;
  distance = constrain(distance, 0, 255);  // 0 ~ 255cm

  writeHead(0);  // ff, 55
  writeSerial(2, RETURN_LENGTH);  // lenght 16 -> 0x10
  writeSerial(3, cmd_idx);
  writeSerial(4, ACT_RCCAR);
  writeSerial(5, RCCAR_DISTANCE); 
  writeSerial(6, distance);  
  writeSerial(7, 0x00);
  writeSerial(8, 0x00);
  writeSerial(9, 0x00);
  writeSerial(10, 0x00);
  writeSerial(11, 0x00);
  writeSerial(12, 0x00);
  writeSerial(13, 0x00);
  writeSerial(14, 0x00);
  writeSerial(15, 0x00);
  writeSerial(16, 0x00);
  writeSerial(17, 0x00);
  writeEnd(18);  // 13, 10
  sendPacket();
}

// Line On/Off P12, 18
// Left Line P2, 8
// Center Line P10, 6
// Right Line  P1, 4
void rccar_linesensor() {
  
  // 좌우를 바꿈 
  int rv = analogRead(LEFT_LINE_SENSOR_PIN);
  int cv = analogRead(CENTER_LINE_SENSOR_PIN);
  int lv = analogRead(RIGHT_LINE_SENSOR_PIN);

  writeHead(0);  // ff, 55
  writeSerial(2, RETURN_LENGTH);  // lenght 16 -> 0x10
  writeSerial(3, cmd_idx);
  writeSerial(4, ACT_RCCAR);
  writeSerial(5, RCCAR_LINESENSOR);
  sendShort(6, lv);   // 6, 7     0~1023
  sendShort(8, cv);   // 8, 9     0~1023
  sendShort(10, rv);  // 10, 11   0~1023
  writeSerial(12, 0x00);
  writeSerial(13, 0x00);
  writeSerial(14, 0x00);
  writeSerial(15, 0x00);
  writeSerial(16, 0x00);
  writeSerial(17, 0x00);
  writeEnd(18);  // 13, 10
  sendPacket();
}

void rccar_initialize() {
  // ***** Motor Initialize
  pinMode(LEFT_MOTOR_DIR_PIN, OUTPUT);
  pinMode(RIGHT_MOTOR_DIR_PIN, OUTPUT);

  // pinMode(LEFT_MOTOR_PWM_PIN, OUTPUT);
  // pinMode(RIGHT_MOTOR_PWM_PIN, OUTPUT);

  ledcSetup(LEFT_MOTOR_PWM_CHANNEL, LEFT_MOTOR_FREQ, LEFT_MOTOR_PWM_ROSOLUTION);
  ledcSetup(RIGHT_MOTOR_PWM_CHANNEL, RIGHT_MOTOR_FREQ, RIGHT_MOTOR_PWM_ROSOLUTION);

  ledcAttachPin(LEFT_MOTOR_PWM_PIN, LEFT_MOTOR_PWM_CHANNEL);
  ledcAttachPin(RIGHT_MOTOR_PWM_PIN, RIGHT_MOTOR_PWM_CHANNEL);

  ledcWrite(LEFT_MOTOR_PWM_CHANNEL, 0);
  ledcWrite(RIGHT_MOTOR_PWM_CHANNEL, 0);

  // ***** Line Sensor
  pinMode(LEFT_LINE_SENSOR_PIN, INPUT);
  pinMode(CENTER_LINE_SENSOR_PIN, INPUT);
  pinMode(RIGHT_LINE_SENSOR_PIN, INPUT);
  
  pinMode(LINE_LED_ONOFF_PIN, OUTPUT);
  digitalWrite(LINE_LED_ONOFF_PIN, HIGH);  // ON

  // ***** ultrasonic
  pinMode(SONIC_TRIG_PIN, OUTPUT);
  pinMode(SONIC_ECHO_PIN, INPUT);

  // rcSonar = new NewPing(SONIC_TRIG_PIN, SONIC_ECHO_PIN, MAX_DISTANCE);

  rcInitialize = true;
  touchInitialize = false;
}

void motor_speed_set(uint8_t lspd, uint8_t rspd) {
  ledcWrite(LEFT_MOTOR_PWM_CHANNEL, lspd);
  ledcWrite(RIGHT_MOTOR_PWM_CHANNEL, rspd);
}

void spin_motor(uint8_t ldir, uint8_t rdir) {
  digitalWrite(LEFT_MOTOR_DIR_PIN, ldir);
  digitalWrite(RIGHT_MOTOR_DIR_PIN, rdir);
}

// R PWM P16, 46
// R DIR P15, 11
// L PWM P14, 13
// L DIR P13, 12
void rccar_rlspeed(int dir_l, int speed_l, int dir_r, int speed_r) {
  motor_speed_set(speed_l, speed_r); 
  spin_motor(dir_l, dir_r);
}


// #define RCCAR_FORWARD       0x01
// #define RCCAR_BACKWARD      0x02
// #define RCCAR_RLSPEED       0x03
// #define RCCAR_STOP          0X04
// #define RCCAR_DISTANCE      0x05
// #define RCCAR_LINESENSOR    0x06
// #define RCCAR_INITIALIZE    0x10

void processRCCar() {
  int command = (int)readBuffer(5);

  switch(command) {
    case RCCAR_INITIALIZE:
    {
      rccar_initialize();
      callOK(ACT_RCCAR);
      break;
    }
    case RCCAR_RLSPEED:
        {
            if (rcInitialize) {
                int dir_l = (int)readBuffer(6);
                int speed_l = (int)readBuffer(7);
                int dir_r = (int)readBuffer(8);
                int speed_r = (int)readBuffer(9);

                rccar_rlspeed(dir_l, speed_l, dir_r, speed_r);
                callOK(ACT_RCCAR);
            } else {
                callNG(ACT_RCCAR);
            }
        break;
      }
    case RCCAR_STOP:
      rccar_rlspeed(0, 0, 0, 0);
      callOK(ACT_RCCAR);
      break;
    case RCCAR_DISTANCE:
      {
        if (rcInitialize) {
            rccar_ultrasonic();
        } else {
            callNG(ACT_RCCAR);
        }
      }
      break;
    case RCCAR_LINESENSOR:
    {
        if (rcInitialize) {
            rccar_linesensor();
        } else {
            callNG(ACT_RCCAR);
        }
      break;
    }
  }
}

// BUZZER_BEEP = 0x01
// BUZZER_MELODY = 0x02
// BUZZER_NOTE = 0x03
void processBuzzer() {
  int command = (int)readBuffer(5);

  switch(command) {
    case BUZZER_BEEP:
      BbBuzzer_beep();
      callOK(ACT_BUZZER);
      break;
    case BUZZER_MELODY:
      {
        int idx = (int)readBuffer(6); 
        callNoResponse(ACT_BUZZER);
        BbBuzzer_melody(idx);
      }
      break;
    case BUZZER_NOTE:
      {
        int note = (int)readBuffer(6);
        int ah = (int)readBuffer(7);
        int al = (int)readBuffer(8);
        int duration = (ah << 8) | al;
        callNoResponse(ACT_BUZZER);
        BbBuzzer_toneNote(note, duration);
      }
      break;
  }
}

void processButton()
{
  writeHead(0);  // ff, 55
  writeSerial(2, RETURN_LENGTH);  // lenght
  writeSerial(3, cmd_idx);
  writeSerial(4, ACT_BUTTON);
  writeSerial(5, BbSwitch_read(SWITCH_A));
  writeSerial(6, BbSwitch_read(SWITCH_B));  
  writeSerial(7, 0x00);
  writeSerial(8, 0x00);
  writeSerial(9, 0x00);
  writeSerial(10, 0x00);
  writeSerial(11, 0x00);
  writeSerial(12, 0x00);
  writeSerial(13, 0x00);
  writeSerial(14, 0x00);
  writeSerial(15, 0x00);
  writeSerial(16, 0x00);
  writeSerial(17, 0x00);
  writeEnd(18);  // 13, 10
  sendPacket();
}

// DIGITAL_OUTPUT = 0x01
// DIGITAL_INPUT = 0x02
// DIGITAL_PULLUP = 0x03
void processDigital(void) {
  int dir = readBuffer(5);
  int pin = readBuffer(6);

  switch(dir){
    case DIGITAL_OUTPUT: 
      {
        int val = readBuffer(7);
        pinMode(pin, OUTPUT);
        delay(5);
        digitalWrite(pin, val);
        callOK(ACT_DIGITAL);
      }
      break;
    case DIGITAL_INPUT:
      {
        pinMode(pin, INPUT);
        delay(10);
        int val = digitalRead(pin);
        delay(10);
        writeHead(0);  // ff, 55
        writeSerial(2, RETURN_LENGTH);
        writeSerial(3, cmd_idx);
        writeSerial(4, DIGITAL_INPUT);
        writeSerial(5, val);  
        writeSerial(6, 0x00);  
        writeSerial(7, 0x00);
        writeSerial(8, 0x00);
        writeSerial(9, 0x00);
        writeSerial(10, 0x00);
        writeSerial(11, 0x00);
        writeSerial(12, 0x00);
        writeSerial(13, 0x00);
        writeSerial(14, 0x00);
        writeSerial(15, 0x00);
        writeSerial(16, 0x00);
        writeSerial(17, 0x00);
        writeEnd(18);  // 13, 10
        sendPacket();
      } 
      break;
    case DIGITAL_PULLUP:
      {
        pinMode(pin, INPUT_PULLUP);
        delay(10);
        int val = digitalRead(pin);
        delay(10);
        writeHead(0);  // ff, 55
        writeSerial(2, RETURN_LENGTH);
        writeSerial(3, cmd_idx);
        writeSerial(4, DIGITAL_PULLUP);
        writeSerial(5, val);  
        writeSerial(6, 0x00);  
        writeSerial(7, 0x00);
        writeSerial(8, 0x00);
        writeSerial(9, 0x00);
        writeSerial(10, 0x00);
        writeSerial(11, 0x00);
        writeSerial(12, 0x00);
        writeSerial(13, 0x00);
        writeSerial(14, 0x00);
        writeSerial(15, 0x00);
        writeSerial(16, 0x00);
        writeSerial(17, 0x00);
        writeEnd(18);  // 13, 10
        sendPacket();
      } 
      break;
  }
}

// ANALOG_OUTPUT 0x01
// ANALOG_INPUT  0x02
void processAnalog(void) {
  int dir = readBuffer(5);
  int pin = readBuffer(6);

  switch(dir){
    case ANALOG_OUTPUT: 
      {
        int val = readShort(7); //0 ~ 1023
        // ledcAttachPin(pin, pin);  // pin번호를 채널번호로 사용한다.
        // ledcSetup(pin, 5000, 8);
        // delay(5);
        // val = (int)map(val, 0, 100, 0, 255);   // 입력은 0에서 100사이 값으로 받는다.
        // ledcWrite(pin, val);

        EspEasyPWM * pwm = findPWM(pin);
        pwm->setPWM(map(val, 0, 255, 0, 100));  //setPWM(백분율) 로 지정하기 때문에 0 ~ 255 백분율로 바꾸어서 지정 
        delay(5);
        callOK(ACT_ANALOG);
      }
      break;
    case ANALOG_INPUT:
      {
        pinMode(pin, INPUT);
        delay(5);
        int analogValue = analogRead(pin);
        delay(5);

        // 하위 8비트와 상위 2비트로 분리
        // byte lowerByte = analogValue & 0xFF;
        // byte upperByte = (analogValue >> 8) & 0x03;
        writeHead(0);  // ff, 55
        writeSerial(2, RETURN_LENGTH);
        writeSerial(3, cmd_idx);
        writeSerial(4, ACT_ANALOG);

        // writeSerial(lowerByte);
        // writeSerial(upperByte);

        // writeSerial(analogValue);
        // writeSerial(0x00);
        sendShort(5, analogValue); // 5, 6  0~1023

        writeSerial(7, 0x00);
        writeSerial(8, 0x00);
        writeSerial(9, 0x00);
        writeSerial(10, 0x00);
        writeSerial(11, 0x00);
        writeSerial(12, 0x00);
        writeSerial(13, 0x00);
        writeSerial(14, 0x00);
        writeSerial(15, 0x00);
        writeSerial(16, 0x00);
        writeSerial(17, 0x00);
        writeEnd(18);  // 13, 10
        sendPacket();
      } 
      break;
  }
}

void processLight(void) {
  // int pin = readBuffer(5);  // light1 or light2
  // int val0 = analogRead(9);
  // int val1 = analogRead(7);
    
  writeHead(0);  // ff, 55
  writeSerial(2, RETURN_LENGTH);
  writeSerial(3, cmd_idx);
  writeSerial(4, ACT_LIGHT);
  sendShort(5, int(_light1_average)); // 5, 6
  sendShort(7, int(_light2_average)); // 7, 8
  writeSerial(9, 0x00);
  writeSerial(10, 0x00);
  writeSerial(11, 0x00);
  writeSerial(12, 0x00);
  writeSerial(13, 0x00);
  writeSerial(14, 0x00);
  writeSerial(15, 0x00);
  writeSerial(16, 0x00);
  writeSerial(17, 0x00);
  writeEnd(18);  // 13, 10
  sendPacket();

}


void processMic(void) {
  int val = analogRead(MIC_IN);
 
  writeHead(0);  // ff, 55
  writeSerial(2, RETURN_LENGTH);
  writeSerial(3, cmd_idx);
  writeSerial(4, ACT_MIC);
  sendShort(5, val); // 5, 6
  writeSerial(7, 0x00);
  writeSerial(8, 0x00);
  writeSerial(9, 0x00);
  writeSerial(10, 0x00);
  writeSerial(11, 0x00);
  writeSerial(12, 0x00);
  writeSerial(13, 0x00);
  writeSerial(14, 0x00);
  writeSerial(15, 0x00);
  writeSerial(16, 0x00);
  writeSerial(17, 0x00);
  writeEnd(18);  // 13, 10
  sendPacket();

}

void processMPU()
{
  writeHead(0);  // ff, 55
  writeSerial(2, RETURN_LENGTH);  // lenght 16 -> 0x10
  writeSerial(3, cmd_idx);
  writeSerial(4, ACT_MPU);
  
  sendFloat(5, 0); // 5, 6, 7, 8
  sendFloat(9, 0); // 9, 10, 11, 12
  //sendFloat(13, az); // 13, 14, 15, 16
  writeSerial(13, _tl);
  writeSerial(14, _tr);
  writeSerial(15, _tu);
  writeSerial(16, _td);
  writeSerial(17, 0x00);
  writeEnd(18);  // 13, 10
  sendPacket();
}

void processUltrasonic() {
  int trig = readBuffer(5);
  int echo = readBuffer(6);

  pinMode(trig, OUTPUT);
  pinMode(echo, INPUT);
  delay(20);

  // digitalWrite(trig, LOW);
  // delayMicroseconds(2);
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);

    long duration = pulseIn(echo, HIGH, ULTRASONIC_TIMEOUT_MICRO);
  int distance = duration * 0.017;
  distance = constrain(distance, 0, 250);  // 0 ~ 250cm

  writeHead(0);  // ff, 55
  writeSerial(2, RETURN_LENGTH);  // lenght 16 -> 0x10
  writeSerial(3, cmd_idx);
  writeSerial(4, ACT_ULTRASONIC);
  writeSerial(5, distance); 
  writeSerial(6, 0x00);  
  writeSerial(7, 0x00);
  writeSerial(8, 0x00);
  writeSerial(9, 0x00);
  writeSerial(10, 0x00);
  writeSerial(11, 0x00);
  writeSerial(12, 0x00);
  writeSerial(13, 0x00);
  writeSerial(14, 0x00);
  writeSerial(15, 0x00);
  writeSerial(16, 0x00);
  writeSerial(17, 0x00);
  writeEnd(18);  // 13, 10
  sendPacket();
}

void processServo() {
  int pin = readBuffer(5);
  int val = readBuffer(6);
  
  if(val >= 0 && val <= 180){
    EspEasyServo * servo = findServo(pin);
    
    if(servo == NULL) {
      // error
      callNG(ACT_SERVO);
      return;
    }
    servo->setServo(val);
    delay(20);
    callOK(ACT_SERVO);
  }
}

void processMainServo() {
  int pin = readBuffer(5);
  int val = readBuffer(6);
  
  if(val >= 0 && val <= 180){
    if(_mainServo == NULL) {
      _mainServo = new EspEasyServo((ledc_channel_t)LEDC_CHANNEL_6, (gpio_num_t)SERVO);
    }
    _mainServo->setServo(val);
    delay(20);
    callOK(ACT_MAIN_SERVO);
  }
}

void processTouch() {
  // pin 사용하지 않은 
  int command = readBuffer(5);

  switch(command){
    case TOUCH_INIT:
      {
        touchInitialize = true;
        callOK(ACT_TOUCH);
      }
      break;
    case TOUCH_VALUES:
    {
        int val0 = BbTouch_read(TOUCH_0);
        int val1 = BbTouch_read(TOUCH_1);
        int val2 = BbTouch_read(TOUCH_2);

        writeHead(0);  // ff, 55
        writeSerial(2, RETURN_LENGTH);
        writeSerial(3, cmd_idx);
        writeSerial(4, ACT_TOUCH);
        sendShort(5, val0); // 5, 6
        sendShort(7, val1); // 7, 8
        sendShort(9, val2); // 9, 10
        writeSerial(11, 0x00);
        writeSerial(12, 0x00);
        writeSerial(13, 0x00);
        writeSerial(14, 0x00);
        writeSerial(15, 0x00);
        writeSerial(16, 0x00);
        writeSerial(17, 0x00);
        writeEnd(18);  // 13, 10
        sendPacket();
    }
  }  
}   // processTouch 

void processDHT() {
  int pin = readBuffer(5);
  float temp = 0.0;
  float hum = 0.0;
  
  DHT11 * dht = findDHT11(pin);
  if(dht == NULL) {
    callZERO(ACT_TMPHUM);
    return;
  }

  hum = dht->readHumidity();
  temp = dht->readTemperature();
  if (hum == -1) { hum = 0.0; }
  if (temp == -1) { temp = 0.0; }

  writeHead(0);  // ff, 55
  writeSerial(2, RETURN_LENGTH);  // lenght
  writeSerial(3, cmd_idx);
  writeSerial(4, ACT_TMPHUM);
  writeSerial(5, int(temp));  
  writeSerial(6, int(hum));  
  writeSerial(7, 0x00);
  writeSerial(8, 0x00);
  writeSerial(9, 0x00);
  writeSerial(10, 0x00);
  writeSerial(11, 0x00);
  writeSerial(12, 0x00);
  writeSerial(13, 0x00);
  writeSerial(14, 0x00);
  writeSerial(15, 0x00);
  writeSerial(16, 0x00);
  writeSerial(17, 0x00);
  writeEnd(18);  // 13, 10
  sendPacket();
}

void processReset() {
    if(rcInitialize) {
      // 본체 서보모터 16 초키화?
      BbMatrix_displayColor(&_pixels, RGB(0, 0, 0));
      if(_mainServo != NULL) {
        _mainServo->setServo(0);
      }
      rccar_initialize();
    } else {
      // 본체 P16번 모터핀
      // pinMode(P16, OUTPUT);
      digitalWrite(P16, LOW);
      // 디스플레이 
      BbMatrix_displayColor(&_pixels, RGB(0, 0, 0));

      // Servo, DHT
      for(int i = 0; i < CHANNEL_LENGTH; i++) {
        servo_pins[i] = 0;
        reserved_servo[i] = NULL;
        dht_pins[i] = 0;
        reserved_dhts[i] = NULL;
        pwm_pins[i] = 0 ;
        reserved_pwm[i] = NULL;
      }

      // 터치핀은 아날로그 입력 전용으로 사용하고, 터치핀을 제외한 핀 초기화, 
      // 터치핀은 한번 터치로 정해지면 모드를 변경할 수 없다.
      for(int i=0; i<AVAILABLE_PINS_LENGTH; i++) {
        pinMode(AVAILABLE_PINS[i], OUTPUT);
      }
      for(int i=0; i<AVAILABLE_PINS_LENGTH; i++) {
        digitalWrite(AVAILABLE_PINS[i], LOW);
      }

      if(_mainServo != NULL) {
        _mainServo->setServo(0);
      }

    }
}


//ff 55 len idx action data0 data1 data2 data3 data4 data5 end  
//ff 55 0b  01  02     01    03    01    00    00    00    00
void parseData() {
  isStart = false;
  int idx = readBuffer(3);
  cmd_idx = (uint8_t)idx;
  action = readBuffer(4);

  switch(action){
    case ACT_BUZZER:
    {
      processBuzzer();
    }
    break;
    case ACT_MATRIX_LED: // C1
    {
      processMatrix();
    }
    break;
    case ACT_BUTTON:  
    {
      processButton();
    }
      break;
    case ACT_MPU:
      processMPU();
      break;
    case ACT_DIGITAL:
      processDigital();
      break;  
    case ACT_ANALOG:
      processAnalog();
      break;  
    case ACT_ULTRASONIC:
      processUltrasonic();
      break;
    case ACT_SERVO:
      processServo();
      break;
    case ACT_MAIN_SERVO:
      processMainServo();
      break;
    case ACT_TOUCH:
        {
            processTouch();
        }
        break;
    case ACT_TMPHUM:
      processDHT();
      break;
    case ACT_LIGHT:
      processLight();
      break;  
    case ACT_MIC:
      processMic();
      break;  
    case ACT_RCCAR:
      processRCCar();
      break;  
    case ACT_RESET_BOARD:
    {
      processReset();
      // 패드에는 리턴패킷은 안보는 것으로 코드가 만들어져 있지만
      // PC버전은 구조상 리턴을 보내야 된다.
      callOK(ACT_RESET_BOARD);
    }
      break;  
    default:
      callNG(ACT_ERROR);
      break;
  }
}

// +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-
// +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-
// +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-
// BLE
#define HWID "0000000000"
/**
 * Bluetooth TX power level(index), it's just a index corresponding to power(dbm).
 * * ESP_PWR_LVL_N12 (-12 dbm)
 * * ESP_PWR_LVL_N9  (-9 dbm)
 * * ESP_PWR_LVL_N6  (-6 dbm)
 * * ESP_PWR_LVL_N3  (-3 dbm)
 * * ESP_PWR_LVL_N0  ( 0 dbm)
 * * ESP_PWR_LVL_P3  (+3 dbm)
 * * ESP_PWR_LVL_P6  (+6 dbm)
 * * ESP_PWR_LVL_P9  (+9 dbm)
 */

// BLE Service Name
const char* DEVICE_NAME = "BitBlock MINI";
//const char* DEVICE_NAME = "KamiBot Pi";


BLEServer* pServer = NULL;
BLECharacteristic* pTxCharacteristic;
bool deviceConnected = false;
bool oldDeviceConnected = false;
int DISPLAY_INTERVAL = 250;
// See the following for generating UUIDs:
// https://www.uuidgenerator.net/

// F4:12:FA:8E:9E:BD  bbmini board mac - OK
#define SERVICE_UUID "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"  // UART service UUID
#define CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

// 데스크탑용으로 임시적으로 
//48fbccae-933a-47cb-8482-b4ace8c2801f
//b131d6e1-8bed-4a28-9efb-98749cb79aec
//e85bf096-800a-4b75-a439-199900e629ad
//5dd67aec-c585-4eae-ae78-8682f533b63b
//#define SERVICE_UUID "48fbccae-933a-47cb-8482-b4ace8c2801f"  // UART service UUID
//#define CHARACTERISTIC_UUID_RX "e85bf096-800a-4b75-a439-199900e629ad"
//#define CHARACTERISTIC_UUID_TX "5dd67aec-c585-4eae-ae78-8682f533b63b"

int idx_led = 0;

class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) {
    deviceConnected = true;
    //BbMatrix_displayColor(&_pixels, RGB(0, 0, 0));
    //delay(10);
    BbMatrix_displayColor(&_pixels, RGB(0, 0, 255));
    BbBuzzer_beep();
    BbMatrix_displayColor(&_pixels, RGB(0, 0, 0));
  };

  void onDisconnect(BLEServer* pServer) {
    deviceConnected = false;
    // 모든핀 리셋
    processReset();
    //BbMatrix_displayColor(&_pixels, RGB(0, 0, 0));
    //delay(10);
    BbMatrix_displayColor(&_pixels, RGB(255, 0, 0));
    BbBuzzer_beep();
    BbMatrix_displayColor(&_pixels, RGB(0, 0, 0));
  }
};

//void printHexData(std::string rxValue) {
  // if (rxValue.length() > 0) {
    //   Serial.println("{BLE} ### Received Value: ###");
    //   Serial.printf("Received Length: %3d\n", rxValue.length());

    //   for (int i = 0; i < rxValue.length(); i++) {
    //     Serial.print(String(rxValue[i], HEX));
    //     Serial.print(" , ");
    //   }
    //   Serial.println();
    // }
//}

// ref. https://github.com/espressif/arduino-esp32/blob/master/libraries/BLE/src/BLECharacteristic.h
class MyCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* pCharacteristic) {
    std::string rxValue = pCharacteristic->getValue();
    if (rxValue.length() == BB_PROTOCOL_LEN) {
    // 스크래치에서 온 데이터를 ESP32에서 사용하는 버퍼로 복사한다.
    // str을 char*로 바꾸어서 복사한다.
    // char* chValue = const_cast<char*>(rxValue.c_str());
      memcpy(rxBuffer, const_cast<char*>(rxValue.c_str()), BB_PROTOCOL_LEN);
      parseData();
    } else {
      // Serial.printf("XXX Received Length is not 20 : %3d\n", rxValue.length()); 
      // Serial.println();
    }

    // // +-+_+_+_+_ DEBUG +-+_+_+_+_
    // if (rxValue.length() == 20) {
    //   Serial.println("{BLE} +++ Received Value +++");
    //   Serial.printf("Received Length: %3d\n", rxValue.length());

    //   for (int i = 0; i < rxValue.length(); i++) {
    //     Serial.print(String(rxValue[i], HEX));
    //     Serial.print(" , ");
    //   }
    //   Serial.println();

    //   // copy data
    //   int receivedData[20] = {0, };
    //   for (int i = 0; i < rxValue.length(); i++) {
    //     receivedData[i] = int(rxValue[i]);
    //   }
    // } else {
    //   Serial.printf("XXX Received Length is not 20 : %3d\n", rxValue.length()); 
    //   Serial.println();
    // }
  }
};

void mpuTaskCode( void * parameter) {
  for(;;) {
    _mpu.update();
    _tl = _mpu.tiltLeft();
    _tr = _mpu.tiltRight();
    _tu = _mpu.tiltUp();
    _td = _mpu.tiltDown();
    vTaskDelay(10 / portTICK_PERIOD_MS);
  } //for
}

void lightTaskCode(void * parameter) {
  for(;;){
    // subtract the last reading:
    _light1_total = _light1_total - _light1_readings[_light_readIndex];
    _light2_total = _light2_total - _light2_readings[_light_readIndex];
    // read from the sensor:
    _light1_readings[_light_readIndex] = analogRead(9);
    _light2_readings[_light_readIndex] = analogRead(7);

    // add the reading to the total:
    _light1_total = _light1_total + _light1_readings[_light_readIndex];
    _light2_total = _light2_total + _light2_readings[_light_readIndex];
    // advance to the next position in the array:
    _light_readIndex = _light_readIndex + 1;

    // if we're at the end of the array...
    if (_light_readIndex >= _numLightReadings) {
      // ...wrap around to the beginning:
      _light_readIndex = 0;
    }

    // calculate the average:
    _light1_average = _light1_total / _numLightReadings;
    _light2_average = _light2_total / _numLightReadings;
    vTaskDelay(10 / portTICK_PERIOD_MS);
  } // for
}


void callOKTaskCode( void * parameter) {
    int idx = (int) parameter; 
    // if (xSemaphoreTake(serial_mutex, portMAX_DELAY)) {
    writeHead(0);  // ff, 55
    writeSerial(2, RETURN_LENGTH);  // lenght
    writeSerial(3, idx);
    writeSerial(4, ACT_BUZZER);
    writeSerial(5, 0x00);  
    writeSerial(6, 0x00);
    writeSerial(7, 0x00);
    writeSerial(8, 0x00);
    writeSerial(9, 0x00);
    writeSerial(10, 0x00);
    writeSerial(11, 0x00);
    writeSerial(12, 0x00);
    writeSerial(13, 0x00);
    writeSerial(14, 0x00);
    writeSerial(15, 0x00);
    writeSerial(16, 0x00);
    writeSerial(17, 0x00);
    writeEnd(18);  // 13, 10
    sendPacket();
    vTaskDelay(10 / portTICK_PERIOD_MS);
    // xSemaphoreGive(serial_mutex);
    // }
    // vTaskDelete(NULL);
}


// -+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
// -+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
// -+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
// -+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
// -+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
// core 0 -> wifi, ble task
// core 1 -> arduino task

void setup() {
  Serial.begin(115200);
  // --- MPU 초기화 ---
  Wire.begin(MPU_SDA, MPU_SCL);
  // serial_mutex = xSemaphoreCreateMutex();

#ifdef MPU6050_TOCKN
  _mpu.begin();
#else
  byte status = _mpu.begin();
  while(status != 0){ 
    delay(1);
  }
  // delay(10);
  // _mpu.upsideDownMounting = true; // uncomment this line if the MPU6050 is mounted upside-down/
#endif

  // Create the BLE Device
  // BLEDevice::init("BitBlock");
  BLEDevice::init(DEVICE_NAME);

  // --- 블투 파워업? ---
  esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_DEFAULT, ESP_PWR_LVL_P9);
  esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_ADV, ESP_PWR_LVL_P9);
  // esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_SCAN ,ESP_PWR_LVL_P9);
  esp_ble_tx_power_set( ESP_BLE_PWR_TYPE_CONN_HDL0, ESP_PWR_LVL_P9 );

  // --- 블투 초기화 ---
  // Create the BLE Server
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // Create the BLE Service
  BLEService* pService = pServer->createService(SERVICE_UUID);

  // Create a BLE Characteristic
  pTxCharacteristic = pService->createCharacteristic(CHARACTERISTIC_UUID_TX, BLECharacteristic::PROPERTY_NOTIFY);
  pTxCharacteristic->addDescriptor(new BLE2902());

  BLECharacteristic* pRxCharacteristic = pService->createCharacteristic(CHARACTERISTIC_UUID_RX, BLECharacteristic::PROPERTY_WRITE);
  pRxCharacteristic->setCallbacks(new MyCallbacks());
  // Start the service
  pService->start();

  // --
  //pServer->getAdvertising()->start();
  
  // BLEAdvertising *pAdvertising = pServer->getAdvertising();  // this still is working for backward compatibility
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  // 9
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  //pAdvertising->setMinPreferred(0x06);  // functions that help with iPhone connections issue
  //pAdvertising->setMinPreferred(0x12);

  delay(10);
  BLEDevice::startAdvertising();
  // BbBoard Initialize
  _pixels.setBrightness(LED_INIT_BRIGHT);

  //set the resolution to 12 bits (0-4095)
  //set the resolution to 10 bits (0-1023)
  analogReadResolution(10);
  // delay(200);

  // 빛센서 이동평균을 위한 코드 
  for (int i = 0; i < _numLightReadings; i++) {
    _light1_readings[i] = 0;
    _light2_readings[i] = 0;
  }


  // switch A, B
  // pinMode(SWITCH_A, INPUT);
  // pinMode(SWITCH_B, INPUT);

 xTaskCreatePinnedToCore(
      mpuTaskCode, /* Function to implement the task */
      "mpuTask", /* Name of the task */
      4096,  /* Stack size in words */
      NULL,  /* Task input parameter */
      0,  /* Priority of the task */
      &mpuTaskHandle,  /* Task handle. */
      0); /* Core where the task should run */


xTaskCreatePinnedToCore(
      lightTaskCode, /* Function to implement the task */
      "lightTask", /* Name of the task */
      10000,  /* Stack size in words */
      NULL,  /* Task input parameter */
      0,  /* Priority of the task */
      &lightTaskHandle,  /* Task handle. */
      1); /* Core where the task should run */

// xTaskCreatePinnedToCore(
//       reportTaskCode, /* Function to implement the task */
//       "reportTask", /* Name of the task */
//       4096,  /* Stack size in words */
//       NULL,  /* Task input parameter */
//       0,  /* Priority of the task */
//       NULL,  /* Task handle. */
//      0); 

    touchInitialize = false;
    rcInitialize = false;

}; // end of setup()
// -+-+-+-+-+-+ END OF SETUP FUNCTION -+-+-+-+-+-+  

void reportSensor() {
    reportBuffer[0] = (uint8_t) 0xff;
    reportBuffer[1] = (uint8_t) 0x66;
    reportBuffer[2] = (uint8_t) BbSwitch_read(SWITCH_A);
    reportBuffer[3] = (uint8_t) BbSwitch_read(SWITCH_B);

    valShort.shortVal = double(_light1_average);
    reportBuffer[4] = (uint8_t) valShort.byteVal[0];
    reportBuffer[5] = (uint8_t) valShort.byteVal[1];

    valShort.shortVal = double(_light2_average);
    reportBuffer[6] = (uint8_t) valShort.byteVal[0];
    reportBuffer[7] = (uint8_t) valShort.byteVal[1];

    // 2024.04.03 비비카 연결한 상태에서 터치센서로 전류가 나오는 현상 ...
    if (touchInitialize) {
        valShort.shortVal = double(BbTouch_read(TOUCH_0));
        reportBuffer[8] = (uint8_t) valShort.byteVal[0];
        reportBuffer[9] = (uint8_t) valShort.byteVal[1];

        valShort.shortVal = double(BbTouch_read(TOUCH_1));
        reportBuffer[10] = (uint8_t) valShort.byteVal[0];
        reportBuffer[11] = (uint8_t) valShort.byteVal[1];

        valShort.shortVal = double(BbTouch_read(TOUCH_2));
        reportBuffer[12] = (uint8_t) valShort.byteVal[0];
        reportBuffer[13] = (uint8_t) valShort.byteVal[1];
    } else if (rcInitialize) {
        // RC카 초기화이면, 터치센서를 사용하지 않으므로 라인센서값을 실어서 보내자!!!
        // 20240415
        int rv = analogRead(LEFT_LINE_SENSOR_PIN);
        int cv = analogRead(CENTER_LINE_SENSOR_PIN);
        int lv = analogRead(RIGHT_LINE_SENSOR_PIN);

        valShort.shortVal = lv;
        reportBuffer[8] = (uint8_t) valShort.byteVal[0];
        reportBuffer[9] = (uint8_t) valShort.byteVal[1];

        valShort.shortVal = cv;
        reportBuffer[10] = (uint8_t) valShort.byteVal[0];
        reportBuffer[11] = (uint8_t) valShort.byteVal[1];

        valShort.shortVal = rv;
        reportBuffer[12] = (uint8_t) valShort.byteVal[0];
        reportBuffer[13] = (uint8_t) valShort.byteVal[1];
        // Serial.println(String(lv) + "," + String(cv)+ ","+String("rv"));

    } else {
        valShort.shortVal = double(0);
        reportBuffer[8] = (uint8_t) valShort.byteVal[0];
        reportBuffer[9] = (uint8_t) valShort.byteVal[1];

        valShort.shortVal = double(0);
        reportBuffer[10] = (uint8_t) valShort.byteVal[0];
        reportBuffer[11] = (uint8_t) valShort.byteVal[1];

        valShort.shortVal = double(0);
        reportBuffer[12] = (uint8_t) valShort.byteVal[0];
        reportBuffer[13] = (uint8_t) valShort.byteVal[1];
    }
    _mpu.update();
    _tl = _mpu.tiltLeft();
    _tr = _mpu.tiltRight();
    _tu = _mpu.tiltUp();
    _td = _mpu.tiltDown();
    reportBuffer[14] = (uint8_t)((_tl << 3) | (_tr << 2) | (_tu << 1) | _td);

    valShort.shortVal = double(analogRead(MIC_IN));
    reportBuffer[15] = (uint8_t) valShort.byteVal[0];
    reportBuffer[16] = (uint8_t) valShort.byteVal[1];
    reportBuffer[17] = (uint8_t) 0;
    reportBuffer[18] = (uint8_t) 0x0d;
    reportBuffer[19] = (uint8_t) 0x0a;

    pTxCharacteristic->setValue(reportBuffer, BB_PROTOCOL_LEN);
    pTxCharacteristic->notify();
}

// 센서데이터값을 0 ~ 255사이의 값으로 변환해서 보낸다.
void reportSensor_2() {
  
    reportBuffer[0] = (uint8_t) 0xff;
    reportBuffer[1] = (uint8_t) 0x66;
    reportBuffer[2] = (uint8_t) BbSwitch_read(SWITCH_A);
    reportBuffer[3] = (uint8_t) BbSwitch_read(SWITCH_B);

    reportBuffer[4] = (uint8_t) map(_light1_average, 0, 1023, 0, 255);
    reportBuffer[5] = (uint8_t) map(_light2_average, 0, 1023, 0, 255);
    
    reportBuffer[6] = (uint8_t) map(BbTouch_read(TOUCH_0), 0, 1023, 0, 255);
    reportBuffer[7] = (uint8_t) map(BbTouch_read(TOUCH_1), 0, 1023, 0, 255);
    reportBuffer[8] = (uint8_t) map(BbTouch_read(TOUCH_2), 0, 1023, 0, 255);
    
    _mpu.update();
    _tl = _mpu.tiltLeft();
    _tr = _mpu.tiltRight();
    _tu = _mpu.tiltUp();
    _td = _mpu.tiltDown();

    // 각도 
    valShort.shortVal = double(_mpu.getBBAngleX());
    reportBuffer[9] = (uint8_t) valShort.byteVal[0];
    reportBuffer[10] = (uint8_t) valShort.byteVal[1];

    valShort.shortVal = double(_mpu.getBBAngleY());
    reportBuffer[11] = (uint8_t) valShort.byteVal[0];
    reportBuffer[12] = (uint8_t) valShort.byteVal[1];

    reportBuffer[13] = (uint8_t)((_tl << 3) | (_tr << 2) | (_tu << 1) | _td);

    valShort.shortVal = double(analogRead(MIC_IN));
    reportBuffer[14] = (uint8_t) valShort.byteVal[0];
    reportBuffer[15] = (uint8_t) valShort.byteVal[1];
    reportBuffer[16] = (uint8_t) 0;
    reportBuffer[17] = (uint8_t) 0;
    reportBuffer[18] = (uint8_t) 0x0d;
    reportBuffer[19] = (uint8_t) 0x0a;
   
    pTxCharacteristic->setValue(reportBuffer, BB_PROTOCOL_LEN);
    pTxCharacteristic->notify();
}

void loop() {
  if (deviceConnected) {
    if(notifyFlag) {
      pTxCharacteristic->setValue(txValue, BB_PROTOCOL_LEN);
      pTxCharacteristic->notify();
      notifyFlag = false;
    } else {
      if (millis() - previousMillis >= 100) {
        reportSensor();
        previousMillis = millis();
      }
    }
    delay(10);  // bluetooth stack will go into congestion, if too many packets are sent
  }

  // ble connect gauge
  if (!deviceConnected) {
    // blue rgb(19,196,233), green rgb(80, 233, 19), purple rgb(161, 36, 245), yellow(226, 233, 19)
    // 2024.04.04 2.0.2 purple rgb(161, 36, 245)
    // 2024.04.15 2.1.0 green rgb(80, 233, 19)
    BbMatrix_displayPixel(&_pixels, idx_led++, RGB(80, 233, 19));
    delay(DISPLAY_INTERVAL);
    if(idx_led > 24) {
      BbMatrix_clear(&_pixels);
      idx_led = 0;
    }
  }

  // disconnecting
  if (!deviceConnected && oldDeviceConnected) {
    delay(10);                   // give the bluetooth stack the chance to get things ready
    pServer->startAdvertising();  // restart advertising
    oldDeviceConnected = deviceConnected;
    _pixels.setBrightness(LED_INIT_BRIGHT);
    idx_led = 0;
  }

  // connecting
  if (deviceConnected && !oldDeviceConnected) {
    // do stuff here on connecting
    oldDeviceConnected = deviceConnected;
  }
}

// END OF FILE ...!!!
// +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-
// +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-
// +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-
