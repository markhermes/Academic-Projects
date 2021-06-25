
byte arm1_Ena = 22;
byte arm2_Ena = 28;
byte arm3_Ena = 34;
byte arm4_Ena = 46;
byte arm5_Ena = 47;
byte arm6_Ena = 4;
byte arm7_Ena = 10;

byte arm1_Pulse = 24;
byte arm2_Pulse = 30;
byte arm3_Pulse = 36;
byte arm4_Pulse = 48;
byte arm5_Pulse = 49;
byte arm6_Pulse = 2;
byte arm7_Pulse = 8;

byte arm1_Dir = 26;
byte arm2_Dir = 33;
byte arm3_Dir = 38;
byte arm4_Dir = 50;
byte arm5_Dir = 51;
byte arm6_Dir = 3;
byte arm7_Dir = 9;

unsigned int steps;

String cmd;
String directio;
String pump;
String stepsStr;

int timeDelay = 100; //started this code at 5000 -> 0.05 secs, was skipping steps
                  //motors 7 and 6 worked well with 500, but small motors did not

void setup() {

  driveAllLow();

  pinMode (arm1_Dir, OUTPUT);
  pinMode (arm2_Dir, OUTPUT);
  pinMode (arm3_Dir, OUTPUT);
  pinMode (arm4_Dir, OUTPUT);
  pinMode (arm5_Dir, OUTPUT);
  pinMode (arm6_Dir, OUTPUT);
  pinMode (arm7_Dir, OUTPUT);

  pinMode (arm1_Pulse, OUTPUT);
  pinMode (arm2_Pulse, OUTPUT);
  pinMode (arm3_Pulse, OUTPUT);
  pinMode (arm4_Pulse, OUTPUT);
  pinMode (arm5_Pulse, OUTPUT);
  pinMode (arm6_Pulse, OUTPUT);
  pinMode (arm7_Pulse, OUTPUT);

  pinMode (arm1_Ena, OUTPUT);
  pinMode (arm2_Ena, OUTPUT);
  pinMode (arm3_Ena, OUTPUT);
  pinMode (arm4_Ena, OUTPUT);
  pinMode (arm5_Ena, OUTPUT);
  pinMode (arm6_Ena, OUTPUT);
  pinMode (arm7_Ena, OUTPUT);


  Serial.begin(9600);
}


void loop() {

  driveAllLow();

  while (Serial.available()) {
    cmd = Serial.readString(); // read the incoming data as string

    //Format : "Pump Num" + "Direction F-> away from motor" + "numSteps"
    pump = cmd.substring(0, 1);
    directio = cmd.substring(1, 2);
    stepsStr = cmd.substring(2, cmd.length());
    steps = stepsStr.toInt();

    //Manually type in if then logic... may be a better way?
    if (pump.equals("1")) {

      digitalWrite(arm1_Ena, LOW);

      if (directio.equals("F")) {
        for (int i = 0; i < steps; i++) {

          Serial.println(pump);
          Serial.println(directio);
          Serial.println(steps);

          digitalWrite(arm1_Dir, HIGH);
          digitalWrite(arm1_Pulse, HIGH);
          delayMicroseconds(timeDelay);
          digitalWrite(arm1_Pulse, LOW);
          delayMicroseconds(timeDelay);
        }
      }
      else if (directio.equals("R")) {
        for (int i = 0; i < steps; i++) {

          Serial.println(pump);
          Serial.println(directio);
          Serial.println(steps);

          digitalWrite(arm1_Dir, LOW);
          digitalWrite(arm1_Pulse, HIGH);
          delayMicroseconds(timeDelay);
          digitalWrite(arm1_Pulse, LOW);
          delayMicroseconds(timeDelay);
        }

      }
    }
    if (pump.equals("2")) {

      digitalWrite(arm2_Ena, LOW);

      if (directio.equals("F")) {
        for (int i = 0; i < steps; i++) {

          Serial.println(pump);
          Serial.println(directio);
          Serial.println(steps);

          digitalWrite(arm2_Dir, HIGH);
          digitalWrite(arm2_Pulse, HIGH);
          delayMicroseconds(timeDelay);
          digitalWrite(arm2_Pulse, LOW);
          delayMicroseconds(timeDelay);
        }
      }
      else if (directio.equals("R")) {
        for (int i = 0; i < steps; i++) {

          Serial.println(pump);
          Serial.println(directio);
          Serial.println(steps);

          digitalWrite(arm2_Dir, LOW);
          digitalWrite(arm2_Pulse, HIGH);
          delayMicroseconds(timeDelay);
          digitalWrite(arm2_Pulse, LOW);
          delayMicroseconds(timeDelay);
        }

      }
    }
    if (pump.equals("3")) {

      digitalWrite(arm3_Ena, LOW);

      if (directio.equals("F")) {
        for (int i = 0; i < steps; i++) {

          Serial.println(pump);
          Serial.println(directio);
          Serial.println(steps);

          digitalWrite(arm3_Dir, HIGH);
          digitalWrite(arm3_Pulse, HIGH);
          delayMicroseconds(timeDelay);
          digitalWrite(arm3_Pulse, LOW);
          delayMicroseconds(timeDelay);
        }
      }
      else if (directio.equals("R")) {
        for (int i = 0; i < steps; i++) {

          Serial.println(pump);
          Serial.println(directio);
          Serial.println(steps);

          digitalWrite(arm3_Dir, LOW);
          digitalWrite(arm3_Pulse, HIGH);
          delayMicroseconds(timeDelay);
          digitalWrite(arm3_Pulse, LOW);
          delayMicroseconds(timeDelay);

        }
      }
    }
    if (pump.equals("4")) {

      digitalWrite(arm4_Ena, LOW);

      if (directio.equals("F")) {
        for (int i = 0; i < steps; i++) {

          Serial.println(pump);
          Serial.println(directio);
          Serial.println(steps);

          digitalWrite(arm4_Dir, HIGH);
          digitalWrite(arm4_Pulse, HIGH);
          delayMicroseconds(timeDelay);
          digitalWrite(arm4_Pulse, LOW);
          delayMicroseconds(timeDelay);
        }
      }
      else if (directio.equals("R")) {
        for (int i = 0; i < steps; i++) {

          Serial.println(pump);
          Serial.println(directio);
          Serial.println(steps);

          digitalWrite(arm4_Dir, LOW);
          digitalWrite(arm4_Pulse, HIGH);
          delayMicroseconds(timeDelay);
          digitalWrite(arm4_Pulse, LOW);
          delayMicroseconds(timeDelay);

        }
      }
    }
    if (pump.equals("5")) {

      digitalWrite(arm5_Ena, LOW);

      if (directio.equals("F")) {
        for (int i = 0; i < steps; i++) {

          Serial.println(pump);
          Serial.println(directio);
          Serial.println(steps);

          digitalWrite(arm5_Dir, HIGH);
          digitalWrite(arm5_Pulse, HIGH);
          delayMicroseconds(timeDelay);
          digitalWrite(arm5_Pulse, LOW);
          delayMicroseconds(timeDelay);
        }
      }
      else if (directio.equals("R")) {
        for (int i = 0; i < steps; i++) {

          Serial.println(pump);
          Serial.println(directio);
          Serial.println(steps);

          digitalWrite(arm5_Dir, LOW);
          digitalWrite(arm5_Pulse, HIGH);
          delayMicroseconds(timeDelay);
          digitalWrite(arm5_Pulse, LOW);
          delayMicroseconds(timeDelay);

        }
      }
    }
    if (pump.equals("6")) {

      digitalWrite(arm6_Ena, HIGH);

      if (directio.equals("F")) {
        for (int i = 0; i < steps; i++) {

          Serial.println(pump);
          Serial.println(directio);
          Serial.println(steps);

          digitalWrite(arm6_Dir, HIGH);
          digitalWrite(arm6_Pulse, HIGH);
          delayMicroseconds(timeDelay);
          digitalWrite(arm6_Pulse, LOW);
          delayMicroseconds(timeDelay);
        }
      }
      else if (directio.equals("R")) {
        for (int i = 0; i < steps; i++) {

          Serial.println(pump);
          Serial.println(directio);
          Serial.println(steps);

          digitalWrite(arm6_Dir, LOW);
          digitalWrite(arm6_Pulse, HIGH);
          delayMicroseconds(timeDelay);
          digitalWrite(arm6_Pulse, LOW);
          delayMicroseconds(timeDelay);

        }
      }
    }
    if (pump.equals("7")) {

      digitalWrite(arm7_Ena, HIGH);

      if (directio.equals("F")) {
        for (int i = 0; i < steps; i++) {

          Serial.println(pump);
          Serial.println(directio);
          Serial.println(steps);

          digitalWrite(arm7_Dir, HIGH);
          digitalWrite(arm7_Pulse, HIGH);
          delayMicroseconds(timeDelay);
          digitalWrite(arm7_Pulse, LOW);
          delayMicroseconds(timeDelay);
        }
      }
      else if (directio.equals("R")) {
        for (int i = 0; i < steps; i++) {

          Serial.println(pump);
          Serial.println(directio);
          Serial.println(steps);

          digitalWrite(arm7_Dir, LOW);
          digitalWrite(arm7_Pulse, HIGH);
          delayMicroseconds(timeDelay);
          digitalWrite(arm7_Pulse, LOW);
          delayMicroseconds(timeDelay);

        }
      }
    }
  }
}


void driveAllLow() {

  digitalWrite(arm1_Ena, HIGH);
  digitalWrite(arm2_Ena, HIGH);
  digitalWrite(arm3_Ena, HIGH);
  digitalWrite(arm4_Ena, HIGH);
  digitalWrite(arm5_Ena, HIGH);
  digitalWrite(arm6_Ena, LOW);
  digitalWrite(arm7_Ena, LOW);


  digitalWrite(arm1_Pulse, LOW);
  digitalWrite(arm2_Pulse, LOW);
  digitalWrite(arm3_Pulse, LOW);
  digitalWrite(arm4_Pulse, LOW);
  digitalWrite(arm5_Pulse, LOW);
  digitalWrite(arm5_Pulse, LOW);

  digitalWrite(arm1_Dir, LOW);
  digitalWrite(arm2_Dir, LOW);
  digitalWrite(arm3_Dir, LOW);
  digitalWrite(arm4_Dir, LOW);
  digitalWrite(arm5_Dir, LOW);



}
