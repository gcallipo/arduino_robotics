

/***************************************************************************

            ARDUINO ROVER ROBOT v. 1.0.0 - IK8YFW - 2017

This is a project that define a rover robot firmware.

All text above, must be included in any redistribution

****************************************************************************/

#include <PS2X_lib.h>

/*  
Wiring:
IN1,IN2--Motor A(right wheel)
IN3,IN4--Motor B(left wheel)

L298N    Arduino    Code
 IN1-------11-------pinRF
 IN2-------10-------pinRB
 IN3-------9--------pinLF
 OLD  IN4-------6--------pinLB
  IN4-------8--------pinLB
  
  ENA A --> 6
  ENA B --> 5
 
IN1 IN2 
1   0    Motor A counterclockwise
0   1    Motor A clockwise
1   1    stop

IN3 IN4
0   1    Motor B clockwise
1   0    Motor B counterclockwise
1   1    stop

pinRF  pinRB  pinLF  pinLB 	
IN1    IN2    IN3    IN4   Car
1      0      0      1     advance
0      1      1      0     back
1      1      1      1     stop
1      0      1      1     Single wheel turn left
1      1      0      1     Single wheel turn right
1      0      1      0     Double wheel turn left
0      1      0      1     Double wheel turn right

Ultrasonic wave modules
A0:Echo
A1:Triger
servo : 3

    L = left
    R = right
    F = advance
    B = back
*/
#include <Servo.h> 

 //#define test_serial 1

#define enaA 6     // left back
#define enaB 5     // left front
#define pinLB 8     // left back
#define pinLF 9     // left front
#define pinRB 10    // right back
#define pinRF 11    // Right front
#define inputPin  A0  // ultrasonic echo
#define outputPin A1  // ultrasonic trig
#define SERVO_PIN  3 // Define the servo motor output pin3 (PWM)
#define PIR_PIN  4
#define VOICE_PIN 2

#define PS2_DAT A5
#define PS2_SEL A3
#define PS2_CMD A4
#define PS2_CLK A2

#define RS_ANGLE 5
#define LS_ANGLE 130  //178-48
#define FS_ANGLE 60   //90-30

#define MIN_DISTANCE 20

PS2X ps2x; // create PS2 Controller Class

Servo myservo;        //  myservo

int Fspeedd = 0;      // front distance
int Rspeedd = 0;      // right distance
int Lspeedd = 0;      // left distance
int directionn = 0;   // Determine the direction of car turns
int delay_time = 500; // Stable steering servo motor

int Fgo = 8;         // advance
int Rgo = 6;         // turn right
int Lgo = 4;         // turn left
int Bgo = 2;         // back

unsigned long lastTime = 0; // timer Speed gear
unsigned long lastTimeGo = 0; // timer Lock
unsigned long lastTimeVoice = 0; // timer Voice

int error = 1;
int type = 0;
boolean controller = false;
boolean status_alarm = true;

void setup()
{
  
  // Setup Controller wireless
  delay (500);
  error= ps2x.config_gamepad(PS2_CLK, PS2_CMD, PS2_SEL, PS2_DAT, false, false);
  
  #ifdef test_serial
  Serial.begin(9600);
  #endif
  
  pinMode(pinLB,OUTPUT); // pin 8 
  pinMode(pinLF,OUTPUT); // pin 9 (PWM)
  pinMode(pinRB,OUTPUT); // pin 10 (PWM) 
  pinMode(pinRF,OUTPUT); // pin 11 (PWM)
  
  pinMode(enaA,OUTPUT); // pin 6
  pinMode(enaB,OUTPUT); // pin 5
  
  pinMode(inputPin, INPUT);    // Define ultrasound input pin
  pinMode(outputPin, OUTPUT);  // Define ultrasound output pin 

  analogWrite(enaA, 110);
  analogWrite(enaB, 100);

  myservo.attach(SERVO_PIN);    
  
  delay(delay_time);      // Waiting for servo motor stable    
  myservo.write(FS_ANGLE); //measure distance in the front
  delay(delay_time);      // Waiting for servo motor stable    

  pinMode(PIR_PIN, INPUT); // pir pin 4
  digitalWrite (PIR_PIN, LOW);
  
  pinMode(VOICE_PIN, OUTPUT); // voice pin 2
  digitalWrite (VOICE_PIN, LOW);
  
  resetTime();
  resetTimeGo();
  resetTimeVoice();
  
  randomSeed(analogRead(A5));
}

void advance(int a)     // advance
    {                   //In the mid-point of the two wheels as a reference
     digitalWrite(pinRB,LOW);  //right wheel advance
     digitalWrite(pinRF,HIGH);
     digitalWrite(pinLB,HIGH);  //left wheel advance
     digitalWrite(pinLF,LOW);
     delay(a);  
    }
void right(int b)        //turn right (single wheel)
    {
     digitalWrite(pinRB,HIGH);   //right stop
     digitalWrite(pinRF,HIGH);
     digitalWrite(pinLB,HIGH);  //left advance
     digitalWrite(pinLF,LOW);
     delay(b);  
    }
void left(int c)         //turn left(single wheel)
    {
     digitalWrite(pinRB,LOW); //righ wheel advance
     digitalWrite(pinRF,HIGH);
     digitalWrite(pinLB,HIGH);   //left stop
     digitalWrite(pinLF,HIGH);
     delay(c);
    }
void turnR(int d)        //turn right(double wheels)
    {
     digitalWrite(pinRB,HIGH);  //right wheel back
     digitalWrite(pinRF,LOW);
     digitalWrite(pinLB,HIGH); //left wheel advance
     digitalWrite(pinLF,LOW);  
     delay(d);
    }
void turnL(int e)        //turn left (double wheels)
    {
     digitalWrite(pinRB,LOW);   //right wheel advance
     digitalWrite(pinRF,HIGH);   
     digitalWrite(pinLB,LOW);   //left wheel back
     digitalWrite(pinLF,HIGH);
     delay(e);
    }    
void stopp(int f)         //stop
    {
     digitalWrite(pinRB,HIGH);
     digitalWrite(pinRF,HIGH);
     digitalWrite(pinLB,HIGH);
     digitalWrite(pinLF,HIGH);
     delay(f);
     
    }
void back(int g)          //back
    {
     digitalWrite(pinRB,HIGH);  //right wheel back
     digitalWrite(pinRF,LOW);
     digitalWrite(pinLB,LOW);  //left wheel back
     digitalWrite(pinLF,HIGH);
     delay(g); 
    }    
    
void detection()        //Measuring three angles(2.90.178)
      { 
   
      ask_pin_F();            // Read the distance of front
         if(Fspeedd <= MIN_DISTANCE)         // If the distance is less than 20cm in front
         {
           
           stopp(1);
           slow();
            
           back(1000);
           stopp(1);
           // clear output ,motor stop
           myservo.write(LS_ANGLE);      //measure left distance
           delay(delay_time);     
           ask_pin_L();
         
           delay(delay_time);     
           myservo.write(RS_ANGLE);     //measure right distance
           delay(delay_time);     
           ask_pin_R();   
        
           if(Lspeedd > Rspeedd)   //compare distance of right and left
           {
            //directionn = Lgo;      //turn left
            directionn = Rgo;
            resetTime ();
          
           }    
           if(Lspeedd <= Rspeedd)   //if the distance is less than or equal to the distance at the right
           {
             //directionn = Rgo;      //turn right
             directionn = Lgo;
             resetTime ();
           }

           myservo.write(FS_ANGLE); //measure distance in the front
           delay(delay_time);     
        }
        else
        {
         directionn = Fgo;
         if (checkTime() > 3000 && checkTime() <= 5900){
           fast();
         }else if (checkTime() >= 6000){
           resetTime();
         }
         else{
           slow();
         } 
        } 
    }
    
unsigned long checkTime(){
     
 unsigned long ret = millis() - lastTime;
     
 #ifdef test_serial
      Serial.print("ret "); Serial.println(ret); 
 #endif

 return ret;
} 
   
void resetTime (){
 lastTime = millis();
}
   
unsigned long checkTimeGo(){
 unsigned long ret = millis() - lastTimeGo;
 return ret;
} 
   
void resetTimeGo (){
 lastTimeGo = millis();
}

unsigned long checkTimeVoice(){
 unsigned long ret = millis() - lastTimeVoice;
 return ret;
} 
   
void resetTimeVoice (){
 lastTimeVoice = millis();
}

void ask_pin_F()   // Measure the distance in front
{
  Fspeedd = myping();//sonar.ping_cm();
  #ifdef test_serial
  Serial.print("Fspedd "); Serial.println(Fspeedd);
  #endif
}

void ask_pin_L()    
{
  Lspeedd = myping();
  #ifdef test_serial
   Serial.print("Lspedd "); Serial.println(Lspeedd);
  #endif
 }

void ask_pin_R()    
{
   Rspeedd = myping();
  #ifdef test_serial
   Serial.print("Rspeedd "); Serial.println(Rspeedd);
  #endif
}
 
int myping(){
      digitalWrite(outputPin, LOW);   //
      delayMicroseconds(2);
      digitalWrite(outputPin, HIGH);  //
      delayMicroseconds(11);
      digitalWrite(outputPin, LOW);    // 
      float distance = pulseIn(inputPin, HIGH);  //
      distance= distance/5.8/10;       // 
      return distance;              //
}

void veryfast(){
 #ifdef test_serial
   Serial.println("VERYFAST ");
 #endif

 analogWrite(enaA, 200);
 analogWrite(enaB, 190);
}


void fast(){
  #ifdef test_serial
   Serial.println("FAST ");
  #endif
  analogWrite(enaA, 170);
  analogWrite(enaB, 140);
}

void slow(){
  #ifdef test_serial
   Serial.println("SLOW ");
  #endif
  analogWrite(enaA, 130);
  analogWrite(enaB, 100);
}
        
void loop()
{
  checkHuman();
   
 if (controller){
    remoteControll();
 }else{
   
  detection();
  if(directionn == 2)
  {
    back(600);
    resetTime ();
    resetTimeGo();
  }
  else if(directionn == 6)
  {
    resetTimeGo();
    turnR(350);
    
    stopp(1);
    resetTime ();
  }
  else if(directionn == 4)
  {
    resetTimeGo();
    turnL(350);
    
    stopp(1);
    resetTime ();
  }
  else if(directionn == 8)
  {
    advance(1);
    // after15 seconds of go ...
    // avoid lock
    if (checkTimeGo() > 15000){
      tryEvasion(1);
      resetTimeGo();
    }
    
    // after 7 secs try to change direction
    if (checkTimeGo() > 7000 && checkTimeGo() < 8000){
      tryEvasion(2);
    }
    
    if (checkTimeGo() > 3000 && checkTimeGo() < 4000){
      readMode();
    }
  }
 }
}
 
void tryEvasion(int mode){
  
  if(mode == 1){
     slow();
     stopp(200);
     turnL(random(200,500));
     stopp(200);
     back(random(300,900));
     stopp(200);
     turnR(random(200,500));
     stopp(200);
  }
  if(mode == 2){
       #ifdef test_serial
         Serial.println("TRY EVASION 2 ");
      #endif

    int rr = random(0,10);
    if (rr > 7){
       #ifdef test_serial
         Serial.println("DO EVASION 2 ");
      #endif
      
      int ran = random(0,150);
      turnR(ran);
    }else if(rr > 4 && rr < 7){
      int ran = random(0,150);
      turnL(ran);
    }
    
    advance(1);
    
  }
  return;
}

void readMode(){
  
     ps2x.read_gamepad();          //read controller and set large motor to spin at 'vibrate' speed
     if (ps2x.NewButtonState())               //will be TRUE if any button changes state (on to off, or off to on)
     {
       if(ps2x.Button(PSB_RED)) {
        slow(); 
        controller = true;
        status_alarm=false;
        delay(100);
       }
      
       if(ps2x.Button(PSB_PINK)) {
        slow(); 
        controller = false;
        status_alarm=true;
        delay(100);
       } 
     }
   return;
}


void remoteControll(){

  ps2x.read_gamepad();          
  if(ps2x.Button(PSB_PAD_UP)) {         //will be TRUE as long as button is pressed
    advance(10);
  }else
  if(ps2x.Button(PSB_PAD_RIGHT)){
    turnL(80);
    stopp(1);
  }else
  if(ps2x.Button(PSB_PAD_LEFT)){
    turnR(80);
    stopp(1);
  }else
  if(ps2x.Button(PSB_PAD_DOWN)){
    
    back(200);
    stopp(1);
    
  }else if(ps2x.Button(PSB_BLUE)){
   slow();
  }
  else if(ps2x.Button(PSB_GREEN)){
   fast();
  }
  else if(ps2x.Button(PSB_L1)){
   onAlarm();
   doAlarm();
  }
  else if(ps2x.Button(PSB_R1)){
   offAlarm();
  }
  else if(ps2x.Button(PSB_L2) || ps2x.Button(PSB_R2)){
   doAlarm();
  }
  else
  {
    stopp(1);
    readMode();
  }  
  
  delay(50);
}

void onAlarm(){
  status_alarm = true;
}

void offAlarm(){
  status_alarm = false;
}

void doAlarm(){
     digitalWrite(VOICE_PIN, HIGH);
     delay(50);
     digitalWrite(VOICE_PIN, LOW);
}



void checkHuman(){
    if (checkTimeVoice() > 10000){
      if (status_alarm == true){
         if (digitalRead(PIR_PIN) == HIGH){
            doAlarm();
         }
      };
      resetTimeVoice();
    }
}


 
 
 
 
 
 
