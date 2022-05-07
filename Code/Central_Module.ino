#include <ArduinoBLE.h>
#include <SPI.h>
#include <SD.h>   //SD crad library
#include "DFRobot_RGBLCD1602.h"

#define ECGIDENT 0
#define EMGIDENT 1
#define BREATHIDENT 2
#define SOUNDIDENT 3
#define IMUIDENT 4

#define BLE_UUID_DATA_SERVICE                  "180F"
#define BLE_UUID_DATA_LEVEL                    "2A19"

#define BLE_MAX_PERIPHERALS 5
#define BLE_SCAN_INTERVAL 7000
#define BLE_IDENT_INTERVAL 3000000 //in microseconds

#define SAMPLING_DELAY 40000 //in microseconds

#define BUTTON_HOLD_LIMIT 2000 //in milliseconds

#define BATTERY_PIN A0
//LED Pins
#define LEDR_SD 2
#define LEDG_SD 3
#define LEDB_SD 5
#define LEDR_POWER 6
#define LEDG_POWER A2
#define LEDB_POWER 9
#define LED_SENSOR 4
#define LED_BLE 7
//Button Pins
#define BUTTON_REF A3
#define BUTTON_BLE A1
#define BUTTON_SENSOR A6
#define BUTTON_SD A7
//LCD I2C Pins
#define SCL A5
#define SDA A4
//SD Card Pins
#define CS 10
#define MOSI 11
#define MISO 12
#define SCK 13

//LCD RGB settings
#define LCDR 255
#define LCDG 255
#define LCDB 255

BLEDevice peripherals[BLE_MAX_PERIPHERALS];
BLECharacteristic dataCharacteristics[BLE_MAX_PERIPHERALS];

DFRobot_RGBLCD1602 lcd(/*lcdCols*/16,/*lcdRows*/2);  //16 characters and 2 lines of show

int peripheralsConnected = 0;
bool bleConnected = false;
word datas[BLE_MAX_PERIPHERALS];
word data;
String sensorList[] = {"ECG","EMG","Breathing","Sound","IMU"};
String unitList[] = {"V (10bit 3.3V ADC)", "V (10bit 3.3V ADC)", "V (10bit 3.3V ADC)", "V (10bit 3.3V ADC)", "mg"};
String displayUnitList[] = {"BPM", "V", "BPM", "V", "mg"};
int thresholdList[] = {580,100, 650, 40, 1400};
String connectedSensors[BLE_MAX_PERIPHERALS];
String connectedUnits[BLE_MAX_PERIPHERALS];
String connectedDisplayUnits[BLE_MAX_PERIPHERALS];
int connectedThresholds[BLE_MAX_PERIPHERALS];
float rates[] = {0,0}; //rates are only calculated for ECG (index 0) and breathing(index 1) (in BPM)
unsigned long ratePrevTimes[] = {0,0};
int ratePrevVals[] = {0,0};
bool increasing[] = {false, false};
unsigned long rateInterval;
unsigned long startTime = 0;

bool batteryState = true;

int dataFileNum = 1;
bool countFileUpdated = false;
char fileName[] = "data";

//Button variables
bool sdState = false;
int sensorCount = 0;
bool bleState = false;
long prevSdPress = 0;
long prevSensorPress = 0;
long prevBlePress = 0;

//SD module variables
bool sdBegin = false;

void setup() {
  pinMode(LED_BLE, OUTPUT);
  Serial.begin(9600);
  // begin initialization
  if (SD.begin(CS)) {
    sdBegin = true;
  }
  lcd.init();
  lcd.setRGB(LCDR,LCDG,LCDB);
}

unsigned long loopCurrent = 0;
unsigned long loopPrev = 0;
unsigned long timeStamp = 0;
unsigned long firstTime = 0;
bool loopFunction = false;
bool newDataPrint;
bool identified = false;
bool lcdTransition = false;
unsigned long lcdPrev = millis();

float displayData = 0;

void loop() {
  File dataFile;
  detectPress();
  batteryVoltage();
  sdCheck();
  bleCheck();
  batteryLED();
  loopCurrent = micros();
  if (loopCurrent - loopPrev < SAMPLING_DELAY){    
    loopFunction= false;
  } else {
    loopFunction= true;
  }
  if (loopFunction) {
    loopPrev = loopCurrent;
    loopFunction = false;
    newDataPrint = false;
    LCDFunc();
    for(int j = 0; j < peripheralsConnected; j++) {
      Serial.println(connectedSensors[j]);
    }
    if(bleState) {
      if(sdState && sdBegin) {
        countFile();
        dataFile = SD.open(fileName, FILE_WRITE);
      }
      if(!bleConnected) {
        BLEConnect();
      }
      if(identified && micros() - startTime > BLE_IDENT_INTERVAL + BLE_SCAN_INTERVAL) {
        for ( int i = 0; i < peripheralsConnected; i++ ) {
          if (dataCharacteristics[i].valueUpdated()) {
            newDataPrint = true;
            dataCharacteristics[i].readValue( data );
            datas[i] = data;
            if (connectedSensors[i] == "ECG") {
              if(data > ratePrevVals[0] && data >= connectedThresholds[i]) {
                if (ratePrevVals[0] == 0 ) {
                  ratePrevVals[0] = data;
                } else {
                  rateInterval = micros() - ratePrevTimes[0];
                  if (!increasing[0]) {
                    //conditional to avoid detecting threshold crossing due to noise ~3 samples
                    rates[0] = 60000000/rateInterval;
                    Serial.print("interval:");
                    Serial.println(rateInterval);
                    ratePrevTimes[0] = micros();
                    increasing[0] = true;
                  }
                }
              } else if (data < connectedThresholds[i] && increasing[0]) {
                increasing[0] = false;
              }
              ratePrevVals[0] = data;
            } else if (connectedSensors[i] == "Breathing") {
              if(data > ratePrevVals[1] && data >= connectedThresholds[i]) {
                if (ratePrevVals[1] == 0 ) {
                  ratePrevVals[1] = data;
                } else {
                  rateInterval = micros() - ratePrevTimes[1];
                  if (!increasing[1]) {
                    rates[1] = 60000000/rateInterval;
                    Serial.print("interval:");
                    Serial.println(rateInterval);
                    ratePrevTimes[1] = micros();
                    increasing[1] = true;
                  }
                }
              } else if (data < connectedThresholds[i] && increasing[1]) {
                increasing[1] = false;
              }
              ratePrevVals[1] = data;
            }
          }
        }
        if (newDataPrint) {
          firstTime = firstTime > 0? micros(): firstTime;
          for ( int i = 0; i < peripheralsConnected; i++ ) {
            timeStamp = micros() - firstTime;
            Serial.print(timeStamp);
            Serial.print(",");
            Serial.print(datas[i]);
            if ( i < peripheralsConnected - 1 ) {
              Serial.print(",");
            }
            if(sdState && sdBegin) {
              dataFile.print(timeStamp);
              dataFile.print( "," );
              dataFile.print(datas[i]);
              if ( i < peripheralsConnected - 1 ) {
                dataFile.print(",");
              }
            }
          }
          Serial.print( "\n" );
          if(sdState && sdBegin) {
            dataFile.print("\n");
          }
        }
      } else {
        for ( int i = 0; i < peripheralsConnected; i++ ) {
          if (dataCharacteristics[i].valueUpdated()) {
            dataCharacteristics[i].readValue( data );
            connectedSensors[i] = sensorList[data];
            connectedUnits[i] = unitList[data];
            connectedDisplayUnits[i] = displayUnitList[data];
            connectedThresholds[i] = thresholdList[data];
            if(sdState && sdBegin) {
              dataFile.print("Time,");
              dataFile.print(sensorList[data]);
              dataFile.print("(" + unitList[data] + ")");
            }
          }
        }
        if(sdState && sdBegin) {
          dataFile.print("\n");
        }
        identified = true;
      }
      dataFile.close();
      sensorLED();
    } else if(bleConnected) {
      BLE.disconnect();
      bleConnected = false;
      for (int i = 0; i < peripheralsConnected; i++) {
        connectedSensors[i] = "";
        connectedDisplayUnits[i] = "";
        connectedUnits[i] = "";
        connectedThresholds[i] = 0;
        identified = false;
      }
    }
  }
}

void detectPress() {
  analogWrite(BUTTON_REF, 0);
  if (analogRead(BUTTON_SD) < 30) {
    if (millis() - prevSdPress >= BUTTON_HOLD_LIMIT) {
      sdState = !sdState;
      Serial.println("Pressed SD");
      prevSdPress = millis();
    }
  }
  if (analogRead(BUTTON_BLE) < 30) {
    if (millis() - prevBlePress >= BUTTON_HOLD_LIMIT) {
      bleState = !bleState;
      Serial.println("Pressed ble");
      prevBlePress = millis();
    }
  }
  if (analogRead(BUTTON_SENSOR) < 30) {
    if (millis() - prevSensorPress >= BUTTON_HOLD_LIMIT) {
      sensorCount++;
      if(sensorCount >= peripheralsConnected) {
        sensorCount -= peripheralsConnected;
      }
      Serial.println("Pressed sensor");
      prevSensorPress = millis();
      lcd.clear();
    }
  }
}

void batteryVoltage() {
  if(analogRead(BATTERY_PIN) < 600) {
    batteryState = false;
  }
}

void batteryLED() {
  if(batteryState) {
    powerLED(0,255,0);
  } else {
    powerLED(255,0,0);
  }
}

void sdCheck() {
  if(sdState){
    sdLED(0,255,0);
  } else {
    sdLED(0,0,0);
  }
}

void bleCheck(){
  if(bleState) {
    digitalWrite(LED_BLE, HIGH);
  } else {
    digitalWrite(LED_BLE, LOW);
    countFileUpdated = false;
  }
}

void LCDFunc() {
  if(bleState && identified && peripheralsConnected > 0) {
    if(lcdTransition) {
      lcd.clear();
      lcdTransition = false;
    }
    lcd.setCursor(0,0);
    lcd.print(connectedSensors[sensorCount]);
    lcd.setCursor(0,1);
    if (millis() - lcdPrev > 500) {
      lcd.setCursor(0,1);
      lcd.print("                ");
      lcd.setCursor(0,1);
      if (connectedSensors[sensorCount] == "ECG") {
        displayData = rates[0];
      } else if (connectedSensors[sensorCount] == "EMG") {
        displayData = datas[sensorCount]*3.3/1023.0;
      } else if (connectedSensors[sensorCount] == "Breathing") {
        displayData = rates[1];
      } else if (connectedSensors[sensorCount] == "Sound") {
        displayData = datas[sensorCount]*3.3/1023.0;
      } else if (connectedSensors[sensorCount] == "IMU") {
        displayData = datas[sensorCount];
      }
      lcdPrev = millis();
    }
    lcd.print(displayData, 1);
    lcd.print(connectedDisplayUnits[sensorCount]);
  } else {
    lcd.clear();
  }
}

void sensorLED() {
  int threshold  = connectedThresholds[sensorCount];
  if(datas[sensorCount] > threshold) {
    digitalWrite(LED_SENSOR, HIGH);
  } else {
    digitalWrite(LED_SENSOR, LOW);
  }
}

void powerLED(int R, int G, int B) {
  analogWrite(LEDR_POWER, R);
  analogWrite(LEDG_POWER, G);
  analogWrite(LEDB_POWER, B);
}

void sdLED(int R, int G, int B) {
  analogWrite(LEDR_SD, R);
  analogWrite(LEDG_SD, G);
  analogWrite(LEDB_SD, B);
}

void BLEConnect() {
  //Serial.println("BLE Central - Receiver");
  //Serial.println("Make sure to turn on the device.");
  // start scanning for peripheral
  BLE.begin();
  BLE.scan();
  BLE.scanForUuid( BLE_UUID_DATA_SERVICE );
  int peripheralCounter = 0;
  unsigned long startMillis = millis();
  lcd.print("Scanning...");
  while ( millis() - startMillis < BLE_SCAN_INTERVAL && peripheralCounter != BLE_MAX_PERIPHERALS ) {
    BLEDevice peripheral = BLE.available();
    if ( peripheral && peripheral.localName() == "GDPsensor" ) {
      boolean peripheralAlreadyFound = false;
      for ( int i = 0; i < peripheralCounter; i++ ) {
        if ( peripheral.address() == peripherals[i].address() ) {
            peripheralAlreadyFound = true;
          }
      }
      if ( !peripheralAlreadyFound ) {
          peripherals[peripheralCounter] = peripheral;
          peripheralCounter++;
      }
      //Serial.print("One connected!");
    }
  }
  //Serial.print("All connected!");
  BLE.stopScan();
  lcd.clear();
  lcd.print("SensorsConnected");
  startTime = micros();
  for ( int i = 0; i < peripheralCounter; i++ ) {
    peripherals[i].connect();
    peripherals[i].discoverAttributes();
    BLECharacteristic dataCharacteristic = peripherals[i].characteristic( BLE_UUID_DATA_LEVEL );
    if ( dataCharacteristic ) {
      dataCharacteristics[i] = dataCharacteristic;
      dataCharacteristics[i].subscribe();
    }
  }
  peripheralsConnected = peripheralCounter;
  bleConnected = true;
  lcdTransition = true;
}

void countFile() {
  if(!countFileUpdated) {
    if(SD.exists("filenumcounter.txt")) {
      File counterFile = SD.open("filenumcounter.txt", FILE_READ);
      dataFileNum = counterFile.parseInt() + 1;
      counterFile.close();
      SD.remove("filenumcounter.txt");
    }
    File counterFile = SD.open("filenumcounter.txt", FILE_WRITE);
    counterFile.print(dataFileNum);
    counterFile.close();
    countFileUpdated = true;
    char dataFileNumStr[2];
    itoa(dataFileNum, dataFileNumStr, 10);
    strcpy(fileName, "data");
    strcat(fileName, dataFileNumStr);
    strcat(fileName, ".txt");
  }
}
