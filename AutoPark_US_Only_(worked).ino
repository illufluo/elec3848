/*
 * AutoPark_US_Only.ino
 * 超声波自动泊车系统（最小可用版）
 * 硬件：Arduino Mega 2560 + 2个HC-SR04 + 四轮底盘
 * 功能：使用左右超声波对齐墙面，并停车到8cm处
 */

#include <Arduino.h>

// ========== 超声波传感器引脚定义 ==========
#define ULTRA_L_TRIG  28  // 左侧超声波 TRIG
#define ULTRA_L_ECHO  29  // 左侧超声波 ECHO
#define ULTRA_R_TRIG  32  // 右侧超声波 TRIG
#define ULTRA_R_ECHO  33  // 右侧超声波 ECHO

// ========== 距离阈值参数（单位：cm）==========
#define TARGET_DIST_CM   8.0    // 目标停车距离
#define TARGET_EPS_CM    1.0    // 停车容差
#define ALIGN_EPS_CM     1.5    // 平行阈值
#define MIN_SAFE_CM      5.0    // 紧急停车距离
#define MAX_VALID_CM     350.0  // 超声波有效上限

// ========== 速度参数（0~255）==========
#define SPD_FAST  60
#define SPD_SLOW  30
#define SPD_TURN  40

// ========== 电机引脚定义（复刻自参考文件）==========
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

// ========== 电机控制宏（复刻自参考文件）==========
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

// ========== 全局变量 ==========
int Motor_PWM = SPD_SLOW;  // 当前电机PWM值（0-255）

// 状态机定义
enum State {
  IDLE = 0,
  ALIGN_WALL = 1,
  PARK_8CM = 2,
  DONE = 3,
  FAIL = 4
};

State currentState = IDLE;

// 时间控制
unsigned long lastLoopTime = 0;
unsigned long lastTelemetryTime = 0;
const unsigned long LOOP_INTERVAL = 15;      // 主循环周期 15ms
const unsigned long TELEMETRY_INTERVAL = 100; // 遥测输出周期 100ms

// 传感器数据
float distLeft = 0;
float distRight = 0;
bool leftValid = false;
bool rightValid = false;

// ========== 基础电机动作函数（复刻自参考文件）==========

// 前进（参考文件的ADVANCE）
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

// 后退（参考文件的BACK）
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

// 原地旋转1（逆时针）
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

// 原地旋转2（顺时针）
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

// 停止
void STOP_ALL() {
  MOTORA_STOP(0);
  MOTORB_STOP(0);
  MOTORC_STOP(0);
  MOTORD_STOP(0);
}

// ========== 高级电机控制函数 ==========

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
  rotate_1();  // 如果实际方向相反，改为rotate_2()
}

void turnRight() {
  Motor_PWM = SPD_TURN;
  rotate_2();  // 如果实际方向相反，改为rotate_1()
}

void driveStop() {
  STOP_ALL();
}

// ========== 超声波传感器函数 ==========

/**
 * 读取单次超声波距离
 * @return 距离(cm)，失败返回-1
 */
float readUltrasonicCM(int trigPin, int echoPin) {
  // 发送触发信号
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  
  // 读取回波时间（超时30ms）
  unsigned long duration = pulseIn(echoPin, HIGH, 30000UL);
  
  if (duration == 0) {
    return -1.0;  // 超时
  }
  
  // 换算距离：duration(us) / 58 = cm
  float distance = (float)duration / 58.0;
  return distance;
}

/**
 * 平滑读取超声波距离（3次取中位数）
 * @return 距离(cm)，失败返回-1
 */
float smoothUltraCM(int trigPin, int echoPin) {
  float readings[3];
  
  // 读取3次
  for (int i = 0; i < 3; i++) {
    readings[i] = readUltrasonicCM(trigPin, echoPin);
    if (readings[i] < 0) {
      return -1.0;  // 任一次失败则返回-1
    }
    if (i < 2) delay(10);  // 两次读取之间短暂延时
  }
  
  // 简单排序（冒泡排序）
  for (int i = 0; i < 2; i++) {
    for (int j = 0; j < 2 - i; j++) {
      if (readings[j] > readings[j + 1]) {
        float temp = readings[j];
        readings[j] = readings[j + 1];
        readings[j + 1] = temp;
      }
    }
  }
  
  // 返回中位数
  return readings[1];
}

/**
 * 更新传感器数据
 */
void updateSensors() {
  distLeft = smoothUltraCM(ULTRA_L_TRIG, ULTRA_L_ECHO);
  distRight = smoothUltraCM(ULTRA_R_TRIG, ULTRA_R_ECHO);
  
  leftValid = (distLeft > 0 && distLeft < MAX_VALID_CM);
  rightValid = (distRight > 0 && distRight < MAX_VALID_CM);
}

/**
 * 检查安全距离
 * @return true表示安全，false表示危险
 */
bool checkSafety() {
  if (leftValid && distLeft < MIN_SAFE_CM) {
    return false;
  }
  if (rightValid && distRight < MIN_SAFE_CM) {
    return false;
  }
  return true;
}

// ========== 遥测输出 ==========

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

// ========== 状态机处理 ==========

void handleStateMachine() {
  // 先检查安全
  if (!checkSafety() && currentState != FAIL) {
    Serial.println("!!! EMERGENCY STOP - Too close to wall !!!");
    driveStop();
    currentState = FAIL;
    return;
  }
  
  // 兜底逻辑：如果双侧都无效且不在FAIL/DONE状态，原地旋转找回信号
  if (!leftValid && !rightValid && currentState != FAIL && currentState != DONE) {
    Serial.println(">>> Lost both sensors, rotating to recover...");
    turnLeft();  // 小角速度旋转
    return;
  }
  
  // 状态机逻辑
  switch (currentState) {
    case IDLE:
      // 直接进入对齐状态
      currentState = ALIGN_WALL;
      Serial.println(">>> Entering ALIGN_WALL state");
      break;
      
    case ALIGN_WALL:
      if (leftValid && rightValid) {
        // 双侧都有效，进行对齐
        float diff = distLeft - distRight;
        
        if (fabs(diff) <= ALIGN_EPS_CM) {
          // 已经对齐，进入停车状态
          driveStop();
          currentState = PARK_8CM;
          Serial.println(">>> Aligned! Entering PARK_8CM state");
        } else if (diff > 0) {
          // 左侧距离大，需要向右转
          turnRight();
        } else {
          // 右侧距离大，需要向左转
          turnLeft();
        }
      } else {
        // 某一侧无效，小角速度旋转
        turnLeft();
      }
      break;
      
    case PARK_8CM:
      if (!leftValid || !rightValid) {
        // 传感器失效，回到对齐状态
        driveStop();
        currentState = ALIGN_WALL;
        Serial.println(">>> Sensor lost, back to ALIGN_WALL");
      } else {
        float avg = (distLeft + distRight) / 2.0;
        float diffAbs = fabs(distLeft - distRight);
        
        // 检查姿态是否偏离
        if (diffAbs > 2.0 * ALIGN_EPS_CM) {
          // 姿态偏了，先纠姿
          Serial.println(">>> Misaligned during parking, correcting...");
          if (distLeft > distRight) {
            turnRight();
          } else {
            turnLeft();
          }
        } else if (avg > TARGET_DIST_CM + TARGET_EPS_CM) {
          // 距离太远，前进
          driveForwardSlow();
        } else if (avg < TARGET_DIST_CM - TARGET_EPS_CM) {
          // 距离太近，后退
          driveBackwardSlow();
        } else {
          // 到达目标距离，停车
          driveStop();
          currentState = DONE;
          Serial.println(">>> PARKING COMPLETE! Distance achieved.");
        }
      }
      break;
      
    case DONE:
      // 保持停止
      driveStop();
      break;
      
    case FAIL:
      // 保持停止
      driveStop();
      break;
  }
}

// ========== Arduino主函数 ==========

void setup() {
  // 初始化串口
  Serial.begin(115200);
  Serial.println("=== AutoPark US Only - Starting ===");
  
  // 初始化电机引脚
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
  
  // 电机初始停止
  STOP_ALL();
  
  // 初始化超声波引脚
  pinMode(ULTRA_L_TRIG, OUTPUT);
  pinMode(ULTRA_L_ECHO, INPUT);
  pinMode(ULTRA_R_TRIG, OUTPUT);
  pinMode(ULTRA_R_ECHO, INPUT);
  
  // 初始化状态
  currentState = ALIGN_WALL;
  
  Serial.println("=== Initialization Complete ===");
  Serial.println("Target: Align to wall and park at 8cm");
  Serial.println();
  
  delay(1000);  // 启动延时，让系统稳定
}

void loop() {
  unsigned long currentTime = millis();
  
  // 主循环：每15ms执行一次
  if (currentTime - lastLoopTime >= LOOP_INTERVAL) {
    lastLoopTime = currentTime;
    
    // 更新传感器数据
    updateSensors();
    
    // 执行状态机逻辑
    handleStateMachine();
  }
  
  // 遥测输出：每100ms输出一次
  if (currentTime - lastTelemetryTime >= TELEMETRY_INTERVAL) {
    lastTelemetryTime = currentTime;
    printTelemetry();
  }
}

