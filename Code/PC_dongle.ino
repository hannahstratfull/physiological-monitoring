#include <ArduinoBLE.h>
//#include <SPI.h>
//#include <SD.h>   //SD crad library


#define BLE_UUID_DATA_SERVICE                  "180F"
#define BLE_UUID_DATA_LEVEL                    "2A19"

#define BLE_MAX_PERIPHERALS 5
#define BLE_SCAN_INTERVAL 5000
#define BLE_IDENT_INTERVAL 5000000 //in microseconds

#define SAMPLING_DELAY 15000 //in microseconds

BLEDevice peripherals[BLE_MAX_PERIPHERALS];
BLECharacteristic dataCharacteristics[BLE_MAX_PERIPHERALS];

int peripheralsConnected = 0;
bool bleConnected = false;
word datas[BLE_MAX_PERIPHERALS];
word data;
String sensorList[] = {"ECG","EMG","Breathing","Sound","IMU"};
String connectedSensors[BLE_MAX_PERIPHERALS];
String requestCode;
bool Sending = false;
unsigned long firstTime = 0;

void setup() {
  Serial.begin(9600);
  // begin initialization
  BLE.begin();
  BLEConnect();
}

unsigned long loopCurrent = 0;
unsigned long loopPrev = 0;
bool loopFunction = false;
bool newDataPrint;
bool identified = false;

void loop() {
  HandleRequest();
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
    if (identified && micros() > BLE_IDENT_INTERVAL) {
      for ( int i = 0; i < peripheralsConnected; i++ ) {
          if ( dataCharacteristics[i].valueUpdated() ) {
          newDataPrint = true;
          dataCharacteristics[i].readValue( data );
          datas[i] = data;
        }
      }
      if ( newDataPrint && Sending) {
        for ( int i = 0; i < peripheralsConnected; i++ ) {
          Serial.print( micros()-firstTime );
          Serial.print( "," );
          Serial.print( datas[i] );
          if ( i < peripheralsConnected - 1 ) {
            Serial.print( "," ); 
          }  
        }
        Serial.print( "\n" );
        newDataPrint = false;
      }
    } else {
      for (int i = 0; i < peripheralsConnected; i++ ) {
        dataCharacteristics[i].readValue( data );
        connectedSensors[i] = sensorList[data];
      }
      identified = true;
    }
  }
}

void HandleRequest() {
  if(Serial.available()){
    requestCode = Serial.readString();
  }
  if (requestCode == "Get list" && identified) {
    for (int i = 0; i < peripheralsConnected; i++){
      Serial.print(connectedSensors[i]);
      if(i < peripheralsConnected -1) {
        Serial.print(',');
      }
    }
    Serial.println();
    requestCode = "";
  } else if (requestCode == "Start plotting") {
    requestCode = "";
    Sending = true;
    firstTime = micros();
  }
}

void BLEConnect() {
  //Serial.println("BLE Central - Receiver");
  //Serial.println("Make sure to turn on the device.");
  // start scanning for peripheral
  BLE.scan();
  BLE.scanForUuid( BLE_UUID_DATA_SERVICE );
  int peripheralCounter = 0;
  unsigned long startMillis = millis();
  while ( millis() - startMillis < BLE_SCAN_INTERVAL && peripheralCounter < BLE_MAX_PERIPHERALS ) {
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
}
