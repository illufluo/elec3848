/*****************************************************
IDP  Feedback and control of servo motor (V20210915)
Potentiometer controlled servo
*****************************************************/

int pos2=40;
int pos3=40;
int pos4=80;
int pos5=1200;

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
  Serial.begin(9600); //Start serial connection
  //myServo.write(0); //Set position
  delay(1000);
  
}

void loop() {
  readserial();
  myServo2.write(pos2);
  myServo3.write(pos3);

  myServo4.write(pos4);

  myServo5.write(pos5);

  
  delay(30);
}
void readserial(){
  
  if (Serial.available() > 0) {
    String data = Serial.readStringUntil('\n');
    pos2=data.substring(0,3).toInt();
    pos3=data.substring(3,6).toInt();
    pos4=data.substring(6,9).toInt();
    pos5=data.substring(9,12).toInt();
    
  }

  
}