
/***************************************************************************

            ARDUINO ARM  v. 1.0.0 - IK8YFW - 2017

This is a project that define a base firmware for mearm project.

All text above, must be included in any redistribution

****************************************************************************/
#include <Servo.h>

Servo myservo3;
Servo myservo0;  // create servo object to control a servo
Servo myservo1;
Servo myservo2;


const int SERVO0 = 6; // PWM
const int SERVO0_P = 2; // right upper button
const int SERVO0_M = 3; // right down button

const int SERVO1 = 9; // PWM
const int SERVO1_P = 4; // left upper button 
const int SERVO1_M = 5; // left down   button

const int SERVO2 = 10; // PWM
const int SERVO2_P = 7; // down left button
const int SERVO2_M = 8; // down right button

const int SERVO3 = 11; // PWM
const int SERVO3_P = 14; // upper left button
const int SERVO3_M = 15; // upper right button

// HAEAD/BACK (RIGHT BUTTONS)
int SERVO0_INIT = 45;
int SERVO0_MIN = 35;
int SERVO0_MAX = 115;

// UP/DWN     (LEFT BUTTONS)
int SERVO1_INIT = 90;
int SERVO1_MIN = 35;
int SERVO1_MAX = 105;

// BASE       (DOWN BUTTONS)
int SERVO2_INIT = 90;
int SERVO2_MIN = 0;
int SERVO2_MAX = 180;

// CLAMP      (UPPER BUTTONS)
int SERVO3_INIT = 60;
int SERVO3_MIN = 55;
int SERVO3_MAX = 85;

int pos0 = SERVO0_INIT;    // variable to store the servo position
int pos1 = SERVO1_INIT;    // variable to store the servo position
int pos2 = SERVO2_INIT;    // variable to store the servo position
int pos3 = SERVO3_INIT;    // variable to store the servo position

int isteresys = 80;

void setup() {
  //Serial.begin(9600);
   
  pinMode(SERVO0_P, INPUT_PULLUP);
  pinMode(SERVO0_M, INPUT_PULLUP);

  pinMode(SERVO1_P, INPUT_PULLUP);
  pinMode(SERVO1_M, INPUT_PULLUP);

  pinMode(SERVO2_P, INPUT_PULLUP);
  pinMode(SERVO2_M, INPUT_PULLUP);

  pinMode(SERVO3_P, INPUT_PULLUP);
  pinMode(SERVO3_M, INPUT_PULLUP);
  
  myservo0.attach(SERVO0);  // attaches the servo on pin 
  myservo1.attach(SERVO1);  // attaches the servo on pin 
  myservo2.attach(SERVO2);  // attaches the servo on pin 
  myservo3.attach(SERVO3);  // attaches the servo on pin
home();

}


void home (){
  
  myservo0.write(pos0);
  delay(50);

  myservo1.write(pos1);
  delay(50);

  myservo2.write(pos2);
  delay(50);

  myservo3.write(pos3);
  delay(50);
}

void action(char cmd) {
  
   if (digitalRead(SERVO0_P) == LOW || cmd =='a') {

      if (pos0<SERVO0_MAX){
        //pos0 ++;
        pos0 = pos0 + 2;
        myservo0.write(pos0);
        delay(isteresys);
      }
      
    }

    if (digitalRead(SERVO0_M) == LOW || cmd =='b') {

      if (pos0>SERVO0_MIN){
        //pos0 --;
        pos0 = pos0 - 2;
        myservo0.write(pos0);
        delay(isteresys);
      }
      
    }

// ************************ SERVO1
    if (digitalRead(SERVO1_P) == LOW || cmd =='c') {

      if (pos1<SERVO1_MAX){
        //pos1 ++;
        pos1 = pos1 + 2;
        myservo1.write(pos1);
        delay(isteresys);
      }
      
    }

    if (digitalRead(SERVO1_M) == LOW || cmd =='d') {

      if (pos1>SERVO1_MIN){
        //pos1 --;
        pos1 = pos1 - 2;
        myservo1.write(pos1);
        delay(isteresys);
      }
      
    }
// ************************ SERVO2
    if (digitalRead(SERVO2_P) == LOW || cmd =='e') {

      if (pos2<SERVO2_MAX){
        //pos2 ++;
        pos2 = pos2 + 2;
        myservo2.write(pos2);
        delay(isteresys);
      }
      
    }

    if (digitalRead(SERVO2_M) == LOW || cmd =='f') {

      if (pos2>SERVO2_MIN){
        //pos2 --;
        pos2 = pos2 - 2;
        myservo2.write(pos2);
        delay(isteresys);
      }
      
    }
// ************************ SERVO3
    if (digitalRead(SERVO3_P) == LOW || cmd =='g') {
      if (pos3<SERVO3_MAX){
        //pos3 ++;
        pos3 = pos3 + 4;
        myservo3.write(pos3);
        delay(isteresys);
      }
      
    }

    if (digitalRead(SERVO3_M) == LOW || cmd =='h') {
      if (pos3>SERVO3_MIN){
        //pos3 --;
        pos3 = pos3 - 4;
        myservo3.write(pos3);
        delay(isteresys);
      }
      
    }

   if (cmd =='p') {
        delay(100);
    }                 
     //Serial.println(pos0);
     //Serial.println(pos1);
     //Serial.println(pos2);
     //Serial.println(pos3);

  delay(25);
}

/************  COMMANDS REMOTE OR MEMORY *******
 *   
// HAEAD/BACK (RIGHT BUTTONS)
a SERVO0_P right upper button
b SERVO0_M right down button

// UP/DWN     (LEFT BUTTONS)
c SERVO1_P left upper button 
d SERVO1_M left down   button

// BASE       (DOWN BUTTONS)
e SERVO2_P down left button
f SERVO2_M down right button

// CLAMP      (UPPER BUTTONS)
g SERVO3_P upper left button
h SERVO3_M upper right button

p PAUSE 200 ms
************************************************/

String cmsS ="ggggg hhhhh pp eeeeeeeeee bbbbbbbbb ccccccccccc  pp ffffffffff aaaaaaaaaaa dddddddd   "; 

void loop() {
  char cmd;

  /* SOLO DEMO
  for (int i = 0; i < cmsS.length() ; i ++){
      cmd = cmsS.charAt(i); 
      action(cmd);
  }

   delay(100);

   home();
   */
   cmd=' ';
   action(cmd);

}






