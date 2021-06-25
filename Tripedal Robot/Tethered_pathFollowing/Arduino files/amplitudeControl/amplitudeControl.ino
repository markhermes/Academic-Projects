//Author: Mark Hermes
//Date: Sept 2020
//Contact: markherm@usc.edu
//
//Description: This script recieves test sweep command from MATLAB and implements for two cycles,
//      and doesnt move until it recives the next command

#include <Servo.h>
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
const float FREQ = 1.0 * 1 / 1000;    //frequency is defined in terms of cycles per millisecond,
const float Period = 1 / FREQ ;       //Period is milliseconds

int nPulseWidth1 = ZERO ;             // 1500 center pos of servo motors
int nPulseWidth2 = ZERO ;
int nPulseWidth3 = ZERO ;


void setup()
{
  int servoNum[3] = {9, 10, 11};

  // attach the servos
  for (int nServo = 0; nServo < 2; nServo++)
  {
    myServos[nServo].attach(servoNum[nServo]);
    myServos[nServo].writeMicroseconds(DEFAULT_PULSE_WIDTH);
  }

  //ZERO and SPAN are empirical values corresponding to motor pulse width and output angle
  TIME = 0;

  myServos[0].writeMicroseconds(ZERO);  //First go to center position
  myServos[1].writeMicroseconds(ZERO);
  myServos[2].writeMicroseconds(ZERO);

  delay(500);                            //Give the motors a sec to get to the position
  Serial.begin(9600);

}
void loop()
{
  if (Serial.available() > 0)             //Wait until TESTSPAN is sent
  {
    int testAngle = Serial.parseInt();
    if (testAngle > 0 )                   //Apparently ParseInt recognized 'NL/CR' as a 0
    {
      actuateBody(testAngle);
      Serial.flush();
    }
  }
}

void testFunc(int testAngle)
{
  Serial.println(testAngle);
}

void actuateBody(int testAngle)
{
  int cycle = 0;
  while (cycle < NUMCYCLES)                     //run function for two cycles.
  {
    float Begin = millis();               //Reset clock after each period
    TIME = 0;
    while (TIME <= Period)
    {
      TIME = (millis() - Begin);
      ARG = 2 * Pi * FREQ * TIME;

      int T0 = ZERO   -  (SPAN  * sin(ARG));
      int T1 = ZERO   +  (SPAN  * sin(ARG));
      int T2 = ZERO   +  (testAngle * CONVERT2PWM  * sin(ARG));

      myServos[0].writeMicroseconds(T0);
      myServos[1].writeMicroseconds(T1);
      myServos[2].writeMicroseconds(T2);
    }
    cycle++;
  }
  Serial.println(0);                      //Tell MATLAB we finished the cycle
}
