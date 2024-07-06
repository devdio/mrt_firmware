/*--------------------------------------------
// VERSION
// 20240701  ver 1.0.1
--------------------------------------------*/
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
// #include <Adafruit_NeoPixel.h>
// #define MPU6050_TOCKN
// #define BITBLOCK_DEBUG
// #include "EspEasyServo.h"
// #include "EspEasyPWM.h"
#include "Wire.h"
#include <DHT11.h>
// #include <NewPing.h>

// NODE용 헤더
#include "NProtocol.h"

// Firmup 프로그램과 버전 맞춤
#define FIRMWARE_VERSION  "Ver 1.0.1"


// --------------------------------------------
// PIN MAP
// --------------------------------------------
#define INPUT_PIN_01    12
#define INPUT_PIN_01    12
#define INPUT_PIN_01    12
#define INPUT_PIN_01    12
#define INPUT_PIN_01    12

#define OUTPUT_PIN_01    12
#define OUTPUT_PIN_01    12
#define OUTPUT_PIN_01    12
#define OUTPUT_PIN_01    12
#define OUTPUT_PIN_01    12

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


// 20240227
// uint8_t AVAILABLE_PINS[] = {P0, P1, P2, P3, P4, P7, P11, P12};
// P0, P1, P2핀은 터치핀 전용
// uint8_t AVAILABLE_PINS_LENGTH = 5;
// uint8_t AVAILABLE_PINS[] = {P3, P4, P7, P11, P12};
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

byte idx = 0;
byte dataLen;

double lastTime = 0.0;
double currentTime = 0.0;

uint8_t cmd_idx = 0;
int action = 0;





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
  memcpy(txValue, returnBuffer, PROTOCOL_PACKET_LEN);
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

void processReset() {
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
      processDigital();
      break;  
    case ACT_ANALOG:
      processAnalog();
      break;  
    case ACT_RESET_BOARD:
    {
      processReset();
      callOK(ACT_RESET_BOARD);
    }
    break;  
    default:
      callNG(ACT_ERROR);
      break;
  }
}

// +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-
// BLE
// +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-
#define HWID "0000000000"
#define MRT_NODE_NAME "MRT-NODE"
// BLE Service Name
const char* DEVICE_NAME = "MRT-NODE";

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
    // std::string rxValue = pCharacteristic->getValue();
    String rxValue = pCharacteristic->getValue();

    if (rxValue.length() == PROTOCOL_PACKET_LEN) {
    // 스크래치에서 온 데이터를 ESP32에서 사용하는 버퍼로 복사한다.
    // str을 char*로 바꾸어서 복사한다.
    // char* chValue = const_cast<char*>(rxValue.c_str());
      memcpy(rxBuffer, const_cast<char*>(rxValue.c_str()), PROTOCOL_PACKET_LEN);
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

}; // *** end of setup()

void reportSensor() { /* something */ }

void loop() {
  if (deviceConnected) {
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
    Serial.println('>');
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
