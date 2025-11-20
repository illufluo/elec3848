/*****************************************************
IDP  Feedback and control of servo motor (V20210915)
Potentiometer controlled servo
*****************************************************/


#define default2 95
#define default3 35
#define rise4 135
#define drop4 80

#define release5 90
#define clip5 55
int pos2=default2;
int pos3=default3;
int pos4=rise4;

int pos5=release5;
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
  myServo2.write(pos2);
  myServo3.write(pos3);

  myServo4.write(pos4);
  
  myServo5.write(pos5);

  
  delay(30);
  delay(1000);
  
}

void loop() {
  if (Serial.available() > 0){
  String data = Serial.readStringUntil('\n');
  if(data=="go"){
    approach();
    delay(1000);
    clip();
    delay(1000);
    rise();
  }else{
    release(); 
  }
  }
  
  
  
  
  
}
void approach(){
  
  while(pos4>drop4){
    pos4-=1;
    writeall();
    delay(10);
  }
  
}
void clip(){
  pos5=clip5;
  writeall();
}
void rise(){
  while(pos4<rise4){
    pos4+=1;
    writeall();
    delay(10);
  }
}
void release(){
  pos5=release5;
  writeall();
}
void serialmove(){
  readserial();
  writeall();

  
  delay(30);
}
void writeall(){
  myServo2.write(pos2);
  myServo3.write(pos3);

  myServo4.write(pos4);
  
  myServo5.write(pos5);

  
  delay(30);
}
void readserial(){
  if (Serial.available() > 0){
  String data = Serial.readStringUntil('\n');
  pos3=data.substring(0,3).toInt();
  pos4=data.substring(3,6).toInt();
  }
  
}
void precisectrl(){
  if (Serial.available() > 0) {
    String data = Serial.readStringUntil('\n');
    pos2=data.substring(0,3).toInt();
    pos3=data.substring(3,6).toInt();
    pos4=data.substring(6,9).toInt();
    pos5=data.substring(9,12).toInt();
    
  }
}