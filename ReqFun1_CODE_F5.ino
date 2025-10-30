
#include <Arduino.h>

#define ULTRA_L_TRIG  28  
#define ULTRA_L_ECHO  29 
#define ULTRA_R_TRIG  32  
#define ULTRA_R_ECHO  33  

#define TARGET_DIST_CM   8.0    
#define TARGET_EPS_CM    1.0    
#define ALIGN_EPS_CM     1.5   
#define MIN_SAFE_CM      5.0   
#define MAX_VALID_CM     350.0  


#define SPD_FAST  60
#define SPD_SLOW  30
#define SPD_TURN  40


#define PWMA 12    // Motor A PWM
#define DIRA1 34
#define DIRA2 35   // Motor A Direction
#define PWMB 8     // Motor B PWM
#define DIRB1 37
#define DIRB2 36   // Motor B Direction
#define PWMC 9     // Motor C PWM
#define DIRC1 43
#define DIRC2 42   // Motor C Direction
#define PWMD 5     // Motor D PWM
#define DIRD1 A4
#define DIRD2 A5   // Motor D Direction


#define MOTORA_FORWARD(pwm)    do{digitalWrite(DIRA1,LOW); digitalWrite(DIRA2,HIGH);analogWrite(PWMA,pwm);}while(0)
#define MOTORA_STOP(x)         do{digitalWrite(DIRA1,LOW); digitalWrite(DIRA2,LOW); analogWrite(PWMA,0);}while(0)
#define MOTORA_BACKOFF(pwm)    do{digitalWrite(DIRA1,HIGH);digitalWrite(DIRA2,LOW); analogWrite(PWMA,pwm);}while(0)

#define MOTORB_FORWARD(pwm)    do{digitalWrite(DIRB1,LOW); digitalWrite(DIRB2,HIGH);analogWrite(PWMB,pwm);}while(0)
#define MOTORB_STOP(x)         do{digitalWrite(DIRB1,LOW); digitalWrite(DIRB2,LOW); analogWrite(PWMB,0);}while(0)
#define MOTORB_BACKOFF(pwm)    do{digitalWrite(DIRB1,HIGH);digitalWrite(DIRB2,LOW); analogWrite(PWMB,pwm);}while(0)

#define MOTORC_FORWARD(pwm)    do{digitalWrite(DIRC1,LOW); digitalWrite(DIRC2,HIGH);analogWrite(PWMC,pwm);}while(0)
#define MOTORC_STOP(x)         do{digitalWrite(DIRC1,LOW); digitalWrite(DIRC2,LOW); analogWrite(PWMC,0);}while(0)
#define MOTORC_BACKOFF(pwm)    do{digitalWrite(DIRC1,HIGH);digitalWrite(DIRC2,LOW); analogWrite(PWMC,pwm);}while(0)

#define MOTORD_FORWARD(pwm)    do{digitalWrite(DIRD1,LOW); digitalWrite(DIRD2,HIGH);analogWrite(PWMD,pwm);}while(0)
#define MOTORD_STOP(x)         do{digitalWrite(DIRD1,LOW); digitalWrite(DIRD2,LOW); analogWrite(PWMD,0);}while(0)
#define MOTORD_BACKOFF(pwm)    do{digitalWrite(DIRD1,HIGH);digitalWrite(DIRD2,LOW); analogWrite(PWMD,pwm);}while(0)


int Motor_PWM = SPD_SLOW;  

enum State {
  IDLE = 0,
  ALIGN_WALL = 1,
  PARK_8CM = 2,
  DONE = 3,
  FAIL = 4
};

State currentState = IDLE;


unsigned long lastLoopTime = 0;
unsigned long lastTelemetryTime = 0;
const unsigned long LOOP_INTERVAL = 15;     
const unsigned long TELEMETRY_INTERVAL = 100; 


float distLeft = 0;
float distRight = 0;
bool leftValid = false;
bool rightValid = false;




//    ↓A-----B↓
//     |  |  |
//     |  ↓  |
//    ↓C-----D↓
void BACK() {
  MOTORA_FORWARD(Motor_PWM); 
  MOTORB_BACKOFF(Motor_PWM);
  MOTORC_FORWARD(Motor_PWM); 
  MOTORD_BACKOFF(Motor_PWM);
}


//    ↑A-----B↑
//     |  ↑  |
//     |  |  |
//    ↑C-----D↑
void ADVANCE() {
  MOTORA_BACKOFF(Motor_PWM); 
  MOTORB_FORWARD(Motor_PWM);
  MOTORC_BACKOFF(Motor_PWM); 
  MOTORD_FORWARD(Motor_PWM);
}


//    ↑A-----B↓
//     | ↗ ↘ |
//     | ↖ ↙ |
//    ↑C-----D↓
void rotate_1() {
  MOTORA_BACKOFF(Motor_PWM); 
  MOTORB_BACKOFF(Motor_PWM);
  MOTORC_BACKOFF(Motor_PWM); 
  MOTORD_BACKOFF(Motor_PWM);
}


//    ↓A-----B↑
//     | ↙ ↖ |
//     | ↘ ↗ |
//    ↓C-----D↑
void rotate_2() {
  MOTORA_FORWARD(Motor_PWM);
  MOTORB_FORWARD(Motor_PWM);
  MOTORC_FORWARD(Motor_PWM);
  MOTORD_FORWARD(Motor_PWM);
}


void STOP_ALL() {
  MOTORA_STOP(0);
  MOTORB_STOP(0);
  MOTORC_STOP(0);
  MOTORD_STOP(0);
}



void driveForwardFast() {
  Motor_PWM = SPD_FAST;
  ADVANCE();
}

void driveForwardSlow() {
  Motor_PWM = SPD_SLOW;
  ADVANCE();
}

void driveBackwardSlow() {
  Motor_PWM = SPD_SLOW;
  BACK();
}

void turnLeft() {
  Motor_PWM = SPD_TURN;
  rotate_1();  
}

void turnRight() {
  Motor_PWM = SPD_TURN;
  rotate_2(); 
}

void driveStop() {
  STOP_ALL();
}



float readUltrasonicCM(int trigPin, int echoPin) {

  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  

  unsigned long duration = pulseIn(echoPin, HIGH, 30000UL);
  
  if (duration == 0) {
    return -1.0; 
  }
  

  float distance = (float)duration / 58.0;
  return distance;
}


float smoothUltraCM(int trigPin, int echoPin) {
  float readings[3];
  

  for (int i = 0; i < 3; i++) {
    readings[i] = readUltrasonicCM(trigPin, echoPin);
    if (readings[i] < 0) {
      return -1.0; 
    }
    if (i < 2) delay(10); 
  }
  

  for (int i = 0; i < 2; i++) {
    for (int j = 0; j < 2 - i; j++) {
      if (readings[j] > readings[j + 1]) {
        float temp = readings[j];
        readings[j] = readings[j + 1];
        readings[j + 1] = temp;
      }
    }
  }
  

  return readings[1];
}


void updateSensors() {
  distLeft = smoothUltraCM(ULTRA_L_TRIG, ULTRA_L_ECHO);
  distRight = smoothUltraCM(ULTRA_R_TRIG, ULTRA_R_ECHO);
  
  leftValid = (distLeft > 0 && distLeft < MAX_VALID_CM);
  rightValid = (distRight > 0 && distRight < MAX_VALID_CM);
}


bool checkSafety() {
  if (leftValid && distLeft < MIN_SAFE_CM) {
    return false;
  }
  if (rightValid && distRight < MIN_SAFE_CM) {
    return false;
  }
  return true;
}


void printTelemetry() {
  Serial.print("STATE,");
  Serial.print(currentState);
  Serial.print(", UL,");
  if (leftValid) {
    Serial.print(distLeft, 1);
  } else {
    Serial.print("N/A");
  }
  Serial.print(", UR,");
  if (rightValid) {
    Serial.print(distRight, 1);
  } else {
    Serial.print("N/A");
  }
  Serial.println();
}


void handleStateMachine() {

  if (!checkSafety() && currentState != FAIL) {
    Serial.println("!!! EMERGENCY STOP - Too close to wall !!!");
    driveStop();
    currentState = FAIL;
    return;
  }
  

  if (!leftValid && !rightValid && currentState != FAIL && currentState != DONE) {
    Serial.println(">>> Lost both sensors, rotating to recover...");
    turnLeft(); 
    return;
  }
  

  switch (currentState) {
    case IDLE:

      currentState = ALIGN_WALL;
      Serial.println(">>> Entering ALIGN_WALL state");
      break;
      
    case ALIGN_WALL:
      if (leftValid && rightValid) {

        float diff = distLeft - distRight;
        
        if (fabs(diff) <= ALIGN_EPS_CM) {

          driveStop();
          currentState = PARK_8CM;
          Serial.println(">>> Aligned! Entering PARK_8CM state");
        } else if (diff > 0) {
 
          turnRight();
        } else {

          turnLeft();
        }
      } else {

        turnLeft();
      }
      break;
      
    case PARK_8CM:
      if (!leftValid || !rightValid) {

        driveStop();
        currentState = ALIGN_WALL;
        Serial.println(">>> Sensor lost, back to ALIGN_WALL");
      } else {
        float avg = (distLeft + distRight) / 2.0;
        float diffAbs = fabs(distLeft - distRight);
        

        if (diffAbs > 2.0 * ALIGN_EPS_CM) {
 
          Serial.println(">>> Misaligned during parking, correcting...");
          if (distLeft > distRight) {
            turnRight();
          } else {
            turnLeft();
          }
        } else if (avg > TARGET_DIST_CM + TARGET_EPS_CM) {

          driveForwardSlow();
        } else if (avg < TARGET_DIST_CM - TARGET_EPS_CM) {

          driveBackwardSlow();
        } else {

          driveStop();
          currentState = DONE;
          Serial.println(">>> PARKING COMPLETE! Distance achieved.");
        }
      }
      break;
      
    case DONE:

      driveStop();
      break;
      
    case FAIL:

      driveStop();
      break;
  }
}



void setup() {

  Serial.begin(115200);
  Serial.println("=== AutoPark US Only - Starting ===");
  

  pinMode(PWMA, OUTPUT);
  pinMode(DIRA1, OUTPUT);
  pinMode(DIRA2, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(DIRB1, OUTPUT);
  pinMode(DIRB2, OUTPUT);
  pinMode(PWMC, OUTPUT);
  pinMode(DIRC1, OUTPUT);
  pinMode(DIRC2, OUTPUT);
  pinMode(PWMD, OUTPUT);
  pinMode(DIRD1, OUTPUT);
  pinMode(DIRD2, OUTPUT);
  

  STOP_ALL();
  

  pinMode(ULTRA_L_TRIG, OUTPUT);
  pinMode(ULTRA_L_ECHO, INPUT);
  pinMode(ULTRA_R_TRIG, OUTPUT);
  pinMode(ULTRA_R_ECHO, INPUT);
  

  currentState = ALIGN_WALL;
  
  Serial.println("=== Initialization Complete ===");
  Serial.println("Target: Align to wall and park at 8cm");
  Serial.println();
  
  delay(1000); 
}

void loop() {
  unsigned long currentTime = millis();

  if (currentTime - lastLoopTime >= LOOP_INTERVAL) {
    lastLoopTime = currentTime;
    

    updateSensors();
    
    handleStateMachine();
  }
  
  if (currentTime - lastTelemetryTime >= TELEMETRY_INTERVAL) {
    lastTelemetryTime = currentTime;
    printTelemetry();
  }
}
