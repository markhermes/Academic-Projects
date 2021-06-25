//y direction aligns with 45 deg off
//x is directly perpendicular to flow

//Rot Servo
//Delay is necessary
//Vin<5V is necessary
//0 is 800 microseconds
//90 is 1685 microseconds
//High is 2200 microseconds approx 140 deg
//y is milliseconds, x is degrees
//positive is CC

//Trial 1: 65 degrees is approximate offset

//Low is 1000 microseconds
//High is 2000 microseconds
//y = 252.74x + 991.15
//y is milliseconds, x is inches
//1 in/ 25.4 mm
//1100 seems to be zero - jk. maybe even 24 mm from this
//x hat is perp to flow around thetapluss =44, or zero position

#include <Servo.h>
//Rot
float off = 71;
float thetaPlusOld = 0;
float thetaPlusNew = 0;
float theta = 0;
float theta2sec = 0;
float increment = thetaPlusOld;

//Linear
float zero = 1100;
//23offset
float mm = 0;//48
float pos = 252.74 * mm * 0.03937 + 991.15;
float z = 5;

Servo rotServo;
Servo linServo;

void setup() {
  //  linServo.attach(11);
    rotServo.attach(11);
  Serial.begin(9600);
}

void loop() {

  linServo.writeMicroseconds(round(pos));
  delay(15);


  while (increment != thetaPlusNew) {
    if (increment > thetaPlusNew) {
      increment = increment - 1;
    }
    else {
      increment = increment + 1;
    }
    theta = off + increment;
    theta2sec = theta * (1685 - 800) / 91 + 800;
    rotServo.writeMicroseconds(round(theta2sec));
    delay(500);
  }

  if (z == 0) {
    Serial.println("Finished");
    z = 1;
  }
}
