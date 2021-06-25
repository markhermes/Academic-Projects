//measurements are very sensitive -> jiggling wires causes disturbances
//leg and bolts are 12.5 grams

#include "HX711.h"
#include <Arduino_LSM9DS1.h>
#include <ArduinoBLE.h>

HX711 LC1;
HX711 LC2;
HX711 LC3;

BLEService arduinoControl("3000");
BLECharacteristic loadCells("3100", BLERead | BLEWrite | BLENotify, 20);
BLECharacteristic imu("3200", BLERead | BLEWrite | BLENotify, 40);
const int ledPin = LED_BUILTIN;

#define DOUT1  12
#define CLK1  11
#define DOUT2  10
#define CLK2  9
#define DOUT3  8
#define CLK3  7

float calibration_factor1 = -1950; //for number 1
float calibration_factor2 = -2040; //for number 2
float calibration_factor3 = -2008; //for number 3

void setup() {
  //
  pinMode(ledPin, OUTPUT); // use the LED pin as an output
  Serial.begin(57600);
  //
  if (!IMU.begin()) {
    Serial.println("starting IMU failed!");
  }
  // 
  startupLoadCells();
  startupBLE();
}

void loop() {
  //
  LC1.set_scale(calibration_factor1); //Adjust to this calibration factor
  LC2.set_scale(calibration_factor2);
  LC3.set_scale(calibration_factor3); //Adjust to this calibration factor
  //
  BLEDevice central = BLE.central();
  if (central)
  {
    Serial.print("Connected to central: ");
    Serial.println(central.address());
    digitalWrite(LED_BUILTIN, HIGH);

    while (central.connected()) {
      //
      writeLoadCellData();
      writeIMUData();
      //
    }
  }
}
void writeIMUData() {
  float x, y, z;
  char imuChar[40];
  String accdata = "0"; String gyrodata = "0"; String magdata = "0";
  while (!IMU.accelerationAvailable());
  IMU.readAcceleration(x, y, z);
  x = x + 1; y = y + 1; z = z + 1; //add 1 to remove negative numbers
  accdata = String(x) + "," + String(y) + "," + String(z);

  while (!IMU.gyroscopeAvailable()) ;
  IMU.readGyroscope(x, y, z); // only need z component
  gyrodata = String(z);

  while (!IMU.magneticFieldAvailable());
  IMU.readMagneticField(x, y, z);
  magdata = String(x) + "," + String(y) + "," + String(z);

  String imudata = accdata + "," + gyrodata + "," + magdata;
  Serial.println(imudata);
  imudata.toCharArray(imuChar, 40);
  imu.setValue(imuChar);
}

void writeLoadCellData() {  //Write loadcell data to the LC characteristic
  //
  String LCString = String(LC1.get_units(), 2) + "," + String(LC2.get_units(), 2) + "," + String(LC3.get_units(), 2);
  char LCChar[20];
  LCString.toCharArray(LCChar, 20);
  loadCells.setValue(LCChar);
  //
}
void startupLoadCells() {
  LC1.begin(DOUT1, CLK1);
  LC1.set_scale();
  LC1.tare(); //Reset the LC1 to 0
  LC2.begin(DOUT2, CLK2);
  LC2.set_scale();
  LC2.tare(); //Reset the LC1 to 0
  LC3.begin(DOUT3, CLK3);//discovered the error to come from the third amplifier... so for now just set the third instance as the second load cell
  LC3.set_scale();
  LC3.tare(); //Reset the LC1 to 0

}
void startupBLE() {
  if (!BLE.begin()) {
    Serial.println("starting BLE failed!");
    while (1); 
  }
  BLE.setEventHandler(BLEConnected, blePeripheralConnectHandler);
  BLE.setEventHandler(BLEDisconnected, blePeripheralDisconnectHandler);
  BLE.setLocalName("ArduinoSensors");
  BLE.setAdvertisedService(arduinoControl);
  arduinoControl.addCharacteristic(loadCells);
  arduinoControl.addCharacteristic(imu);
  BLE.addService(arduinoControl);
  loadCells.setValue(0);  // set an initial value for the characteristic
  imu.setValue(0);
  // start advertising
  BLE.advertise();
  Serial.println(("Bluetooth device active, waiting for connections..."));
}

// NOT IMPORTANT -- maybe important idk im tired of debugging
/////////////////////////////////////////////////////////////////////////////////////////////////
void blePeripheralConnectHandler(BLEDevice central) {
  // central connected event handler
  Serial.print("Connected event, central: ");
  Serial.println(central.address());
}
void blePeripheralDisconnectHandler(BLEDevice central) {
  // central disconnected event handler
  Serial.print("Disconnected event, central: ");
  Serial.println(central.address());
}
void switchCharacteristicWritten(BLEDevice central, BLECharacteristic characteristic) {
  // central wrote new value to characteristic, update LED
  Serial.print("Characteristic event, written: ");

  if (loadCells.value()) {
    Serial.println("LED on");
    digitalWrite(ledPin, HIGH);
  } else {
    Serial.println("LED off");
    digitalWrite(ledPin, LOW);
  }
}
/////////////////////////////////////////////////////////////////////////////////////////////////

//  loadCells.setEventHandler(BLEWritten, switchCharacteristicWritten);   // assign event handlers for characteristic

// SKIP connection check and just broadcast

//  BLEDevice central = BLE.central();
//  if (central) {
//    Serial.print("Connected to central: ");
//    Serial.println(central.address());
//    digitalWrite(LED_BUILTIN, HIGH);
//    while (central.connected()) {
//
