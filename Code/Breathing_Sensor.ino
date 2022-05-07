/* prevoid - add any presetup needed for your sensor and include 
 *  the library needed!
*/

#include <ArduinoBLE.h>                         //bluetooth library
#include <Filters.h>
#include <Filters/Butterworth.hpp>

#define sensorPin A0
#define batteryPin A1
#define RPin 2
#define GPin 3
#define BPin 5
#define ECGIDENT 0
#define EMGIDENT 1
#define BREATHIDENT 2
#define SOUNDIDENT 3
#define IMUIDENT 4

#define SAMPLING_DELAY 1000 //in microseconds
#define IDENT_INTERVAL 4000 //in microseconds

boolean didMyOneTimeAction = false;             //identifier check

 // BLE IMU Service - make sure the tags match files
BLEService GDPsensor("180F");

// BLE Orientation Characteristic

BLECharacteristic sensordata("2A19", BLERead | BLENotify, 512);
int connect_millis;

void setup() {
  Serial.begin(9600);

  // begin BLE initialization
  if (!BLE.begin()) {
    Serial.println("starting BLE failed!");

    while (1);
  }
  
  /* Set a local name for the BLE device
     This name will appear in advertising packets
     and can be used by remote devices to identify this BLE device
     The name can be changed but maybe be truncated based on space left in advertisement packet
  */
  BLE.setLocalName("GDPsensor");
  BLE.setAdvertisedService(GDPsensor);     // add the service UUID
  GDPsensor.addCharacteristic(sensordata); // add the IMU characteristic
  BLE.addService(GDPsensor);               // Add the IMU service
  sensordata.writeValue("HELLO");          // set initial value for this characteristic
  
  /* Start advertising BLE.  It will start continuously transmitting BLE
     advertising packets and will be visible to remote BLE central devices
     until it receives a new connection */

  // start advertising
  BLE.advertise();

  Serial.println("Bluetooth device active, waiting for connections...");  
  String address = BLE.address();
  Serial.print("Local address is: ");
  Serial.println(address);

}

unsigned long loopCurrent = 0;
unsigned long loopPrev = 0;
bool loopFunction = false;

// Sampling frequency
const double f_s = 100; // Hz
// Cut-off frequency (-3 dB)
const double f_c = 2; // Hz
// Normalized cut-off frequency
const double f_n = 2 * f_c / f_s;

float cf = 19.5;
 
// Sixth-order Butterworth filter
auto filter = butter<6>(f_n);

void loop() {
  BatteryLED();
  // wait for a BLE central
  BLEDevice central = BLE.central();
  connect_millis = millis();

  // if a central is connected to the peripheral:
  if (central) {
    Serial.print("Connected to central: ");
    // print the central's BT address:
    Serial.println(central.address());

    // check the sensor every period
    // while the central is connected:
    while (central.connected()) {
      loopCurrent = micros();
      if (loopCurrent - loopPrev < SAMPLING_DELAY){    
        loopFunction= false;
      } else {
        loopFunction= true;
      }
      word data = 0;
      if (loopFunction == true) {
        loopPrev = loopCurrent;
        loopFunction = false;
        if (millis() - connect_millis < IDENT_INTERVAL) {
          word ident  = BREATHIDENT;
          sensordata.writeValue(ident);
          Serial.println(ident);
        }
        //fill in your sensor code here! whatever you need to get the data output
        //put the answer into the data word
        else {
          //writes the imu word value into the characteristic
          int cur_sample = 0;
          data = filter(analogRead(A0));
          float vout = (cur_sample * 5.0) / 1023.0;
          //data = vout * cf ;
          sensordata.writeValue(data);
          Serial.println(data);
        }
      }      
    }
        
    // when the central disconnects,
    Serial.print("Disconnected from central: ");
    Serial.println(central.address());

}
}

void BatteryLED() {
  if(analogRead(batteryPin) > 600) {
    BatteryLED(0,255,0);
  } else {
    BatteryLED(255,0,0);
  }
}

void BatteryLED(int R, int G, int B) {
  analogWrite(RPin, R);
  analogWrite(GPin, G);
  analogWrite(BPin, B);
}
