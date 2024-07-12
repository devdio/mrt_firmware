/*--------------------------------------------
// VERSION
// 20240701  ver 1.0.1
--------------------------------------------*/
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// #include "Wire.h"
// #include <DHT11.h>
// #include <NewPing.h>
// #include "EspEasyServo.h"

// NODE용 헤더
#include "NProtocol.h"

// Firmup 프로그램과 버전 맞춤
#define FIRMWARE_VERSION  "Ver 1.0.1"


// --------------------------------------------
// PIN MAP
// --------------------------------------------
// 입력핀
#define INPUT_PIN_01 33     // ADC1_CH5
#define INPUT_PIN_02 34     // ADC1_CH6
#define INPUT_PIN_03 35     // ADC1_CH7
#define INPUT_PIN_04 36     // SENSOR_VP  ADC1_CH0
#define INPUT_PIN_05 39     // SENSOR_VN  ADC1_CH3

int INPUT_PINS[5] = {INPUT_PIN_01, INPUT_PIN_02, INPUT_PIN_03, INPUT_PIN_04, INPUT_PIN_05};

// 출력핀
#define OUTPUT_PIN_01 23
#define OUTPUT_PIN_02 25
#define OUTPUT_PIN_03 26
#define OUTPUT_PIN_04 27
#define OUTPUT_PIN_05 32

int OUTPUT_PINS[5] = {OUTPUT_PIN_01, OUTPUT_PIN_02, OUTPUT_PIN_03, OUTPUT_PIN_04, OUTPUT_PIN_05};

// 모터핀
#define MOT_R1_1 18
#define MOT_R1_2 19
#define MOT_R2_1 16
#define MOT_R2_2 17
#define MOT_L1_1 15
#define MOT_L1_2 2
#define MOT_L2_1 12
#define MOT_L2_2 14

// 초음파 ???
// VCC
#define ULTRA_TRIG 4
#define UlTRA_ECHO 5
// GND

// 서보모터
#define SERVO_PIN 13

// 자이로 ???
#define GYRO_PIN_1 22
#define GYRO_PIN_2 21

// LCD ???
#define LCD_PIN_1 22
#define LCD_PIN_2 21


// ESP32 API  https://docs.espressif.com/projects/arduino-esp32/en/latest/api/ledc.html
// EspEasyServo servo(LEDC_CHANNEL_0, (gpio_num_t)13);


// --------------------------------------------------------------------
// 모터 드라이버 사용
// https://wiki.dfrobot.com/HR8833_Dual_DC_Motor_Driver__SKU_DIR0040_
// https://lastminuteengineers.com/drv8833-arduino-tutorial/
void set_motor_pwm(int pwm, int IN1_PIN, int IN2_PIN)
{
  if (pwm < 0) {  // reverse speeds
    analogWrite(IN1_PIN, -pwm);
    digitalWrite(IN2_PIN, LOW);

  } else { // stop or forward
    digitalWrite(IN1_PIN, LOW);
    analogWrite(IN2_PIN, pwm);
  }
}
// set_motor_pwm(80, MOT_L2_1, MOT_L2_2);       // CCW
// set_motor_pwm(-80, MOT_L2_1, MOT_L2_2);      // CW
// set_motor_pwm(100, MOT_L1_1, MOT_L1_2);      // CCW
// set_motor_pwm(-100, MOT_L1_1, MOT_L1_2);     // CW
// set_motor_pwm(100, MOT_R1_1, MOT_R1_2);      // CCW
// set_motor_pwm(-100, MOT_R1_1, MOT_R1_2);     // CW
// set_motor_pwm(100, MOT_R2_1, MOT_R2_2);      // CCW
// set_motor_pwm(-100, MOT_R2_1, MOT_R2_2);     // CW


const int ULTRASONIC_TIMEOUT_MICRO = 500000;  // 500 millis secs
bool rcInitialize = false;
bool touchInitialize = false;
bool notifyFlag = false;
// bool reportFlag = false;

// --------------------------------------------
// PROTOCOL
// --------------------------------------------
#define PROTOCOL_PACKET_LEN 20

// 버퍼 스크래치 -> ESP32로 보낸 데이터를 저장
// ESP32보드 수신한 데이터 
char rxBuffer[PROTOCOL_PACKET_LEN];
// unsigned char prevc = 0;

// ESP32 -> 스크래치로 보낼 데이터 저장 
// ESP32가 송신할 데이터
uint8_t returnBuffer[PROTOCOL_PACKET_LEN];

// ESP32 -> 스크래치로 각 핀의 리포터를 송신
uint8_t reportBuffer[PROTOCOL_PACKET_LEN];


uint8_t txValue[] = { 0xFF, 0x55, RETURN_LENGTH, 0, ACT_NOTHING, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0x0D, 0x0A };
uint8_t txTestValue[] = { 'H', 'H', 'H', 'H', 'H', 'H', 'H', 'H', 'H', 'H', 'H', 'H', 'H', 'H', 'H', 'H', 'H', 'H', 0x0D, 0x0A };

uint8_t reportValue[] = { 0xFF, 0x66, RETURN_LENGTH, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0x0D, 0x0A };

// 센서값 리포터의 송신을 위한 타이밍
unsigned long previousMillis = millis();
double lastTime = 0.0;
double currentTime = 0.0;

// 받은 패킷을 파싱할 때 사용하는 변수
uint8_t cmd_idx = 0;
int action = 0;

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

// 입력(스크래치->ESP32) 패킷에서 한개의 값 읽기
unsigned char readBuffer(int index){
  return rxBuffer[index];
}

// void writeBuffer(int idx, unsigned char c){
//   rxBuffer[idx] = c;
// }

// for Serial
// void writeSerial(unsigned char c){
//   Serial.write(c);
// }

// for BLE
// 센서값을 스크래치에 보내기위한 버퍼에 넣어둔다.
void writeSerial(int idx, unsigned char c){
  returnBuffer[idx] = (uint8_t) c;
}

// for BLE 
// BLE의 캐릭터리스틱에 센서값을 복사한다. 
void sendPacket() {
  memcpy(txValue, returnBuffer, PROTOCOL_PACKET_LEN);
  notifyFlag = true;
}

void callNoResponse(uint8_t action){
  // 응답을 보내지않음.
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

void writeFloat(int i, float value){
  val.floatVal = value;
  writeSerial(i, val.byteVal[0]);
  writeSerial(i+1, val.byteVal[1]);
  writeSerial(i+2, val.byteVal[2]);
  writeSerial(i+3, val.byteVal[3]);
}

void writeShort(int i, double value){
  valShort.shortVal = value;
  writeSerial(i, valShort.byteVal[0]);    // low
  writeSerial(i+1, valShort.byteVal[1]);  // hight
}

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


// DIGITAL_OUTPUT = 0x01
// DIGITAL_INPUT = 0x02
// DIGITAL_PULLUP = 0x03
void processDigital(void) {
  int dir = readBuffer(5);
 
  Serial.print("processDigital dir : ");
  Serial.println(dir);

  switch(dir){
    case DIGITAL_OUTPUT: 
      {
        int pin = readBuffer(6);
        int _gpio = OUTPUT_PINS[pin-1];
        
        Serial.print("gpio : ");
        Serial.println(_gpio);
     
        int val = readBuffer(7);
        Serial.print("value : ");
        Serial.println(val);
        
        pinMode(_gpio, OUTPUT);
        delay(5);
        digitalWrite(_gpio, val);
        callOK(ACT_DIGITAL);
      }
      break;
    case DIGITAL_INPUT:
      {
        int pin = readBuffer(6);
        int _gpio = INPUT_PINS[pin-1];
        
        Serial.println("DIGITAL INPUT");
        Serial.print("_gpio : ");
        Serial.println(_gpio);

        pinMode(_gpio, INPUT);
        delay(10);

        int digitalVal = digitalRead(_gpio);
        delay(10);

        Serial.print("value : ");
        Serial.println(digitalVal);

        writeHead(0);  // ff, 55
        writeSerial(2, RETURN_LENGTH);
        writeSerial(3, cmd_idx);
        writeSerial(4, DIGITAL_INPUT);
        writeSerial(5, digitalVal);  
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
        int pin = readBuffer(6);
        pin = INPUT_PINS[pin-1];
        
        Serial.print("pin : ");
        Serial.println(pin);

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
  Serial.print("processAnalog :");
  Serial.println(dir);

  switch(dir){
    case ANALOG_OUTPUT: 
      {
        int pin = readBuffer(6);
        int _gpio = OUTPUT_PINS[pin-1];
        int channelNo = pin;                     // pin번호(1 ~ 5)를 채널번호(1 ~ 5)로 사용한다. 0번 채널은 서보모터에 사용

        Serial.print("gpio : ");
        Serial.println(_gpio);

        Serial.print("channel : ");
        Serial.println(channelNo);

        int val = readShort(7); //0 ~ 1023
        ledcAttachPin(_gpio, channelNo);        // (핀번호, 채널번호)
        ledcSetup(channelNo, 5000, 8);          // (채널번호, 주파수, 분해능)
        delay(5);
        val = (int)map(val, 0, 100, 0, 255);    // 입력은 0에서 100사이 값으로 받는다.
        Serial.print("Value : ");
        Serial.println(val);
        ledcWrite(channelNo, val);              // (채널번호, 튜티)
        delay(5);
        callOK(ACT_ANALOG);
      }
      break;
    case ANALOG_INPUT:
      {
        Serial.println("*** Analog Input");

        int pin = readBuffer(6);
        int _gpio = INPUT_PINS[pin-1];

        Serial.print("gpio : ");
        Serial.println(_gpio);

        // 20240712 핀모드는 없어도 현재는 잘 동작한다. 참고 https://randomnerdtutorials.com/esp32-adc-analog-read-arduino-ide/
        // pinMode(_gpio, INPUT);
        // delay(5);

        int analogValue = analogRead(_gpio);
        delay(5);
        Serial.print("Analog Value: ");
        Serial.println(analogValue);

        writeHead(0);  // ff, 55
        writeSerial(2, RETURN_LENGTH);
        writeSerial(3, cmd_idx);
        writeSerial(4, ACT_ANALOG);
        writeShort(5, analogValue); // 5, 6  0~1023
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

void processDCMotor() {

  // run_dcmotor(motorNo, motorDir, motorSpeed)
  int mode = readBuffer(5);
  int motorNo = readBuffer(6);
  
  switch(mode){
    case DCMOTOR_RUN: {
      int motorDir = (readBuffer(7) == 0) ? -1 : 1;   // 0: CW  1: CCW (Counter Clock Wise)
      int motorSpeed = readBuffer(8);

      Serial.println('**************************');
      Serial.println(motorNo);
      Serial.println(motorSpeed * motorDir);

      if (motorNo == 0) {
        // R1
        set_motor_pwm(motorSpeed * motorDir, MOT_R1_1, MOT_R1_2);
      } else if(motorNo == 1) {
        // R2
        set_motor_pwm(motorSpeed * motorDir, MOT_R2_1, MOT_R2_2);
      } else if(motorNo == 2) {
        // L1
        set_motor_pwm(motorSpeed * motorDir, MOT_L1_1, MOT_L1_2);
      } else if (motorNo == 3) {
        // L2
        set_motor_pwm(motorSpeed * motorDir, MOT_L2_1, MOT_L2_2);
      }
      break;
    }
    case DCMOTOR_STOP: {
      if (motorNo == 0) {
        // R1
        analogWrite(MOT_R1_1, 0);
        analogWrite(MOT_R1_2, 0);
      } else if(motorNo == 1) {
        // R2
        analogWrite(MOT_R2_1, 0);
        analogWrite(MOT_R2_2, 0);
      } else if(motorNo == 2) {
        // L1
        analogWrite(MOT_L1_1, 0);
        analogWrite(MOT_L1_2, 0);
      } else if (motorNo == 3) {
        // L2
        analogWrite(MOT_L2_1, 0);
        analogWrite(MOT_L2_2, 0);
      }
      break;
    }
    default:{
      break;
    }
  }
  delay(5);
}

void processServo() 
{
  int pin = readBuffer(5); // 현재는 필요없지만, 
  int angle = readBuffer(6);
  // servo.setServo(angle);
  // delay(200);
  callOK(ACT_SERVO);
}

void processReset() {
  // 모터 멈춤
  Serial.println("STOP------------");
  // analogWrite(MOT_R1_1, LOW);
  // analogWrite(MOT_R1_2, LOW);
  // analogWrite(MOT_R2_1, LOW);
  // analogWrite(MOT_R2_2, LOW);
  // analogWrite(MOT_L1_1, LOW);
  // analogWrite(MOT_L1_2, LOW);
  // analogWrite(MOT_L2_1, LOW);
  // analogWrite(MOT_L2_2, LOW);
  
}


bool isStart = false;


//ff 55 len idx action data0 data1 data2 data3 data4 data5 end  
//ff 55 0b  01  02     01    03    01    00    00    00    00
void parseData() {
  isStart = false;
  int idx = readBuffer(3);
  cmd_idx = (uint8_t)idx;
  action = readBuffer(4);

  
  switch(action){
    case ACT_DIGITAL:
    {
      processDigital();
      break;  
    }
    case ACT_ANALOG: 
    {
      processAnalog();
      break;
    }
    case ACT_DCMOTOR:
    {
      processDCMotor();
      callOK(ACT_DCMOTOR);
      break;
    }  
    case ACT_SERVO:
    {
      processServo();
      break;
    }
    case ACT_RESET_BOARD:
    {
      Serial.println("RESET@");
      processReset();
      callOK(ACT_RESET_BOARD);
      break;  
    }
    default:
    {
      callNG(ACT_ERROR);
      break;
    }
  }
}

// +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-
// BLE
// +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-
// #define HWID "0000000000"
// #define MRT_NODE_NAME "MRT-NODE"
// BLE Service Name
// const char* DEVICE_NAME = "MRT-NODE";
const char* DEVICE_NAME = "BitBlock MINI";

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


class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) {
    deviceConnected = true;
    // take some time
    // delay(10);
    Serial.println("ON-CONNECT CALLBACK");
  };

  void onDisconnect(BLEServer* pServer) {
    deviceConnected = false;
    // 모든핀 리셋
    processReset();
    Serial.println("ON-DIS-CONNECT CALLBACK");
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
// ref. https://github.com/espressif/arduino-esp32/tree/master/libraries/BLE/src

class MyCallbacks : public BLECharacteristicCallbacks {

  // 스크래치가 BLE에 메세지를 write하면 실행되는 콜백.
  void onWrite(BLECharacteristic* pCharacteristic) {
    // esp32 board : 2.0.17
    std::string rxValue = pCharacteristic->getValue();
    // esp32 board : 3.0.2
    // String rxValue = pCharacteristic->getValue();
    // Serial.println("***SomthingWrite***");
    if (rxValue.length() == PROTOCOL_PACKET_LEN) {
      memcpy(rxBuffer, const_cast<char*>(rxValue.c_str()), PROTOCOL_PACKET_LEN);
      parseData();
    } else {
      // Serial.println("Under 20");
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

// -+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
// -+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
// core 0 -> wifi, ble task
// core 1 -> arduino task

void setup() {
  Serial.begin(115200);
  // Create the BLE Device
  BLEDevice::init(DEVICE_NAME);
  
  // --- 블투 초기화 ---
  // Create the BLE Server
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // Create the BLE Service
  BLEService* pService = pServer->createService(SERVICE_UUID);

  // Create a BLE Characteristic
  pTxCharacteristic = pService->createCharacteristic(CHARACTERISTIC_UUID_TX, 
                                                    BLECharacteristic::PROPERTY_NOTIFY);
  pTxCharacteristic->addDescriptor(new BLE2902());

  BLECharacteristic* pRxCharacteristic = pService->createCharacteristic(CHARACTERISTIC_UUID_RX, 
                                                                        BLECharacteristic::PROPERTY_WRITE);
  pRxCharacteristic->setCallbacks(new MyCallbacks());
  // Start the service
  pService->start();

  // --
  // pServer->getAdvertising()->start();
  // BLEAdvertising *pAdvertising = pServer->getAdvertising();  // this still is working for backward compatibility
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  // pAdvertising->setMinPreferred(0x06);  // functions that help with iPhone connections issue
  // pAdvertising->setMinPreferred(0x12);

  delay(10);
  BLEDevice::startAdvertising();
  
  // ---------------------------------------
  // -- NODE Board Initialize
  // ---------------------------------------
  //set the resolution to 12 bits (0-4095)
  //set the resolution to 10 bits (0-1023)
  analogReadResolution(10);
  // delay(200);

  // ----------------------------------------
  // 모터핀 초기화 
  //-----------------------------------------
  pinMode(MOT_L1_1, OUTPUT);
  pinMode(MOT_L1_2, OUTPUT);
  pinMode(MOT_L2_1, OUTPUT);
  pinMode(MOT_L2_2, OUTPUT);
  pinMode(MOT_R1_1, OUTPUT);
  pinMode(MOT_R1_2, OUTPUT);
  pinMode(MOT_R2_1, OUTPUT);
  pinMode(MOT_R2_2, OUTPUT);

  // 모터 모두 멈춤
  analogWrite(MOT_R1_1, 0);
  analogWrite(MOT_R1_2, 0);
  analogWrite(MOT_R2_1, 0);
  analogWrite(MOT_R2_2, 0);
  analogWrite(MOT_L1_1, 0);
  analogWrite(MOT_L1_2, 0);
  analogWrite(MOT_L2_1, 0);
  analogWrite(MOT_L2_2, 0);

}; // *** end of setup()

void reportSensor() { /* something */ }

void loop() {

  if (deviceConnected) {
    // 스크래치에서 명령을 받으면 notifyFlag가 true가 되어서 리턴을 보낸다.
    if(notifyFlag) {
      pTxCharacteristic->setValue(txValue, PROTOCOL_PACKET_LEN);
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
    delay(DISPLAY_INTERVAL);
  }

  // disconnecting
  if (!deviceConnected && oldDeviceConnected) {
    delay(10);                   // give the bluetooth stack the chance to get things ready
    pServer->startAdvertising();  // restart advertising
    oldDeviceConnected = deviceConnected;
  }

  // connecting
  if (deviceConnected && !oldDeviceConnected) {
    // do stuff here on connecting
    oldDeviceConnected = deviceConnected;
  }
} // *** END OF LOOP

// END OF FILE ...!!!
// +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-
// +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-
// +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-
