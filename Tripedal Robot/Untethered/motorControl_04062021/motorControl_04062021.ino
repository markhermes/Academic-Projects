//Author: Mark Hermes
//Date: Sept 2020
//Contact: markherm@usc.edu
//
//Description: This script recieves test sweep command from MATLAB and implements for two cycles,
//      and doesnt move until it recives the next command

// OPERATE MOTORS LESS THAN 6V
#include <Servo.h>
#include <ArduinoBLE.h>

BLEService motorControl("1900");
BLECharacteristic motorArray("2010", BLERead | BLEWrite | BLENotify, 9);
BLECharacteristic motorState("2030", BLERead | BLEWrite | BLENotify, 1);

Servo myServos[3];

// Use Define to save memory space and reduce precision errors.
#define COUNT_DOWN -1
#define COUNT_UP +1
#define INCREMENT 10                  // move in steps of 10 milliseconds
#define DELAY 7
#define SWEEP_DEGREES 30
#define PHASE_DEGREES 30
#define ZERO 1500
#define SPAN SWEEP_DEGREES*475/45     //Calibration is 45 / 475 [deg]/[microS]
#define CONVERT2PWM 475/45
#define PHASE Pi*PHASE_DEGREES/180
#define NUMCYCLES 2

const float Pi = 3.1415;
volatile double TIME = 0;             //Going to change this to float -> less memory than double
volatile float ARG = 0;               //Also want to test what happens when I remove volatile signifier...
volatile float testAngle = 0;         //needs to be float for PWM calculation
float FREQ = 1 * 0.001;  //frequency is defined in terms of cycles per millisecond,
float Period = 1 / FREQ ;       //Period is milliseconds

int nPulseWidth1 = ZERO ;             // 1500 center pos of servo motors
int nPulseWidth2 = ZERO ;
int nPulseWidth3 = ZERO ;
int testAngles[3];
int temp[3];
const int ledPin = LED_BUILTIN;
char updatePin[1];

void setup()
{
  Serial.begin(57600);


  int servoNum[3] = {8, 9, 10};
  // attach the servos
  for (int nServo = 0; nServo < 3; nServo++)
  {
    myServos[nServo].attach(servoNum[nServo]);
  }

  TIME = 0;
  myServos[0].writeMicroseconds(ZERO);  //First go to center position
  myServos[1].writeMicroseconds(ZERO);
  myServos[2].writeMicroseconds(ZERO);
  delay(500);                            //Give the motors a sec to get to the position

  startupBLE();

}
void loop()
{
  BLEDevice central = BLE.central();
  if (central)
  {
    Serial.print("Connected to central: ");
    Serial.println(central.address());
    digitalWrite(LED_BUILTIN, HIGH);

    while (central.connected()) {
      if (motorArray.written())
      {
        saveMotorArray(&testAngles[0]);
        actuateBody(&testAngles[0]);
        updatePin[0] = (char)0x01;
        motorState.setValue(updatePin);
      }
    }
  }
}

bool saveMotorArray(int *testAngles) { //Update MotorArray value based on what MATLAB sends
  //
  char motorVals[6];
  char tempChar[2];
  motorArray.readValue(motorVals, 6);
  for (int i = 0; i <= 2; i++)                //for loop for splitting the char
  {
    tempChar[0] = motorVals[i * 2]; tempChar[1] = motorVals[i * 2 + 1];
    testAngles[i] = atoi(tempChar) - 30;          //Easier to subtract to not worry about negatives
  }
}
void actuateBody(int *testAngles) {
  int cycle = 0;

  while (cycle < NUMCYCLES)                     //run function for two cycles.
  {
    float Begin = millis();               //Reset clock after each period30
    TIME = 0;
    while (TIME <= Period)
    {
      TIME = (millis() - Begin);
      ARG = 2 * Pi * FREQ * TIME;

      double T0 = ZERO   +  (testAngles[2] * CONVERT2PWM  * sin(ARG));
      double T1 = ZERO   +  (testAngles[1] * CONVERT2PWM  * sin(ARG));
      double T2 = ZERO   +  (testAngles[0] * CONVERT2PWM  * sin(ARG));

//      updatePin[0] = (char)0x00;
//      motorState.setValue(updatePin);
      myServos[0].writeMicroseconds(T0);  //4pi/3 leg
    
      myServos[1].writeMicroseconds(T1);  //2pi/3 leg
    
      myServos[2].writeMicroseconds(T2);  //lead leg (red cable not green)

    }
    Serial.println(String(testAngles[0]) + "," + String(testAngles[1]) + "," + String(testAngles[2]));
    cycle++;

  }
}
void startupBLE() {
  if (!BLE.begin()) {
    Serial.println("starting BLE failed!");
  }
  BLE.setEventHandler(BLEConnected, blePeripheralConnectHandler);
  BLE.setEventHandler(BLEDisconnected, blePeripheralDisconnectHandler);

  BLE.setLocalName("MotorArduino");
  BLE.setAdvertisedService(motorControl);
  motorControl.addCharacteristic(motorArray);
  motorControl.addCharacteristic(motorState);
  BLE.addService(motorControl);
  motorArray.setValue(0);
  motorState.setValue(0);
  // start advertising
  BLE.advertise();
  Serial.println(("Bluetooth device active, waiting for connections..."));
}

// NOT IMPORTANT
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
}
/////////////////////////////////////////////////////////////////////////////////////////////////
