/*****************************************************
IDP  Feedback and control of servo motor (V20210915)
Potentiometer controlled servo
*****************************************************/

int pos2=90;
int pos3=90;
int pos4=90;
int pos5=90;
int incr2=1;
int incr3=2;
int incr4=3;
int incr5=4;
#include <Servo.h>

Servo myServo2;  //Create servo object to control a servo
Servo myServo3;
Servo myServo4;
Servo myServo5;
void setup() {
  //pinMode(A0, INPUT); //Pin mode definition
  myServo2.attach(2);  //Attaches the servo on pin 2 to the servo object
  myServo3.attach(3);
  myServo4.attach(4);
  myServo5.attach(5);
  Serial.begin(115200); //Start serial connection
  //myServo.write(0); //Set position
  delay(1000);
  
}

void loop() {
  if(pos2>=120){
    
    incr2=-1;
  }
  if(pos2<=60){
    
    incr2=1;
  }
  pos2=pos2+incr2;


  if(pos3>=120){
    
    incr3=-2;
  }
  if(pos3<=60){
    
    incr3=2;
  }
  pos3=pos3+incr3;

if(pos4>=120){
    
    incr4=-3;
  }
  if(pos4<=60){
    
    incr4=3;
  }
  pos4=pos4+incr4;
  if(pos5>=120){
    
    incr5=-4;
  }
  if(pos5<=60){
    
    incr5=4;
  }
  pos5=pos5+incr5;
  //int inputSignal = analogRead(A0);
  //int pos = map(inputSignal, 0, 1023, 0, 180); //Converting the reading range to 180 degrees
  myServo2.write(90);
  //myServo3.write(pos3);
  myServo3.write(90);
  myServo4.write(pos4);
  //myServo4.write(180);
  myServo5.write(90);

  Serial.print("  ; Writes angle : ");
  Serial.println(pos2);
  delay(30);
}
