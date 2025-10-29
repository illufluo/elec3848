/*
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  🚗 自动停车系统 - 基于有限状态机 (FSM)
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

【项目说明】
  本程序实现麦克纳姆全向轮车的自动停车功能，分为 5 个阶段：
  
  S1 (LIGHT_SEEKING)    : 使用光敏传感器找光源并旋转对齐
  S2 (APPROACHING)      : 直线前进直到超声波检测到障碍物 <70cm
  S3 (ALIGNING)         : 使用两个超声波传感器做平行对齐 (UF ≈ UR)
  S4 (POSITIONING)      : 纯横移调整距离至目标 8cm
  S5 (PARKED)           : 停车完成，显示状态

【安全机制】
  - 任意阶段若检测到距离 <6cm，立即急停 + 短暂后退 + 返回 S3
  - 超声波交替触发，避免相互干扰
  - 使用中值滤波减少噪声

【传感器策略】
  - 每个阶段只使用对应的传感器，其他传感器不参与决策
  - PWM 范围：0-255
  - 所有可调参数集中在顶部，方便后期调试

【作者信息】
  项目：ELEC3848 自动停车系统
  基于：Car_Volt_Feedback_24A.ino 模板
  日期：2025-10-28
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
*/

#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// ═══════════════════════════════════════════════════════════════════════════
// ▶ OLED 显示器配置
// ═══════════════════════════════════════════════════════════════════════════
#define SCREEN_WIDTH  128
#define SCREEN_HEIGHT 32
#define OLED_RESET    28
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// ═══════════════════════════════════════════════════════════════════════════
// ▶ 引脚定义
// ═══════════════════════════════════════════════════════════════════════════

// --- 电机引脚 (来自参考模板) ---
#define PWMA   12    // 电机 A PWM
#define DIRA1  34    // 电机 A 方向 1
#define DIRA2  35    // 电机 A 方向 2

#define PWMB   8     // 电机 B PWM
#define DIRB1  37    // 电机 B 方向 1
#define DIRB2  36    // 电机 B 方向 2

#define PWMC   9     // 电机 C PWM
#define DIRC1  43    // 电机 C 方向 1
#define DIRC2  42    // 电机 C 方向 2

#define PWMD   5     // 电机 D PWM
#define DIRD1  A4    // 电机 D 方向 1
#define DIRD2  A5    // 电机 D 方向 2

// --- 光敏传感器引脚 ---
#define LDR_L_PIN  A1    // 左侧光敏电阻
#define LDR_R_PIN  A2    // 右侧光敏电阻

// --- 超声波传感器引脚 ---
#define UF_TRIG    22    // 前超声波 Trig
#define UF_ECHO    23    // 前超声波 Echo
#define UR_TRIG    24    // 右超声波 Trig
#define UR_ECHO    25    // 右超声波 Echo

// ═══════════════════════════════════════════════════════════════════════════
// ▶ 可调参数 - 集中配置区域
// ═══════════════════════════════════════════════════════════════════════════

// --- PWM 档位设置 (0-255) ---
uint8_t PWM_FWD    = 180;    // 前进 PWM
uint8_t PWM_STRAFE = 160;    // 横移 PWM
uint8_t PWM_SPIN   = 140;    // 旋转 PWM
uint8_t PWM_CRAWL  = 100;    // 爬行 PWM (精细调整)

// --- 光敏传感器阈值 (S1 阶段) ---
int T_SUM      = 1500;     // 总亮度阈值 (L+R > T_SUM)
int T_DIFF     = 50;       // 左右差值阈值 (|L-R| < T_DIFF)
int N_LOCK     = 10;       // 连续稳定次数，达到后锁定

// --- 超声波距离阈值 (cm) ---
float NEAR_GATE_CM   = 70.0;  // S2→S3 触发距离
float TARGET_DIST_CM = 8.0;   // 目标停车距离
float STOP_GUARD_CM  = 6.0;   // 安全急停距离
float DELTA_PAR      = 2.0;   // S3 平行对齐误差容忍 (|UF-UR| < DELTA_PAR)
float EPSILON_DIST   = 0.5;   // S4 距离到位误差容忍

// --- 超声波偏移补偿 (如果传感器安装位置不对称) ---
float SENSOR_OFFSET  = 0.0;   // 传感器偏移量 (cm)，可正可负

// --- 脉冲控制参数 ---
int PULSE_MS      = 80;     // 单次动作脉冲时长 (ms)
int SETTLE_MS     = 100;    // 动作后稳定等待时间 (ms)
int US_PERIOD_MS  = 60;     // 超声波测量间隔 (ms)，避免频繁触发

// --- 方向反转开关 (若实际运动方向相反，改为 true) ---
bool INVERT_STRAFE_DIR   = false;  // 横移方向反转
bool INVERT_ROTATE_DIR   = false;  // 旋转方向反转

// --- 调试开关 ---
bool ENABLE_SERIAL_DEBUG = true;   // 串口调试输出
bool ENABLE_CSV_OUTPUT   = false;  // CSV 格式输出 (用于数据分析)

// ═══════════════════════════════════════════════════════════════════════════
// ▶ 状态机定义
// ═══════════════════════════════════════════════════════════════════════════
enum State {
  S1_LIGHT_SEEKING = 1,  // 找光并旋转对齐
  S2_APPROACHING   = 2,  // 直线前进
  S3_ALIGNING      = 3,  // 平行对齐
  S4_POSITIONING   = 4,  // 距离调整
  S5_PARKED        = 5   // 停车完成
};

State currentState = S1_LIGHT_SEEKING;

// ═══════════════════════════════════════════════════════════════════════════
// ▶ 全局变量
// ═══════════════════════════════════════════════════════════════════════════
int lockCounter = 0;              // S1 锁定计数器
bool lightLocked = false;         // S1 是否已锁定
float ambientBrightness = 0;      // 环境亮度基准 (setup 时标定)

unsigned long lastUltrasonicTime = 0;  // 上次超声波测量时间
bool ultrasonicToggle = false;         // 超声波交替触发标志

float distUF = 999.0;             // 前超声波距离 (cm)
float distUR = 999.0;             // 右超声波距离 (cm)

unsigned long lastDebugTime = 0;  // 上次调试输出时间

// ═══════════════════════════════════════════════════════════════════════════
// ▶ 电机宏定义 (来自参考模板)
// ═══════════════════════════════════════════════════════════════════════════
#define MOTORA_FORWARD(pwm)  do{digitalWrite(DIRA1,LOW); digitalWrite(DIRA2,HIGH); analogWrite(PWMA,pwm);}while(0)
#define MOTORA_STOP(x)       do{digitalWrite(DIRA1,LOW); digitalWrite(DIRA2,LOW);  analogWrite(PWMA,0);}while(0)
#define MOTORA_BACKOFF(pwm)  do{digitalWrite(DIRA1,HIGH);digitalWrite(DIRA2,LOW);  analogWrite(PWMA,pwm);}while(0)

#define MOTORB_FORWARD(pwm)  do{digitalWrite(DIRB1,LOW); digitalWrite(DIRB2,HIGH); analogWrite(PWMB,pwm);}while(0)
#define MOTORB_STOP(x)       do{digitalWrite(DIRB1,LOW); digitalWrite(DIRB2,LOW);  analogWrite(PWMB,0);}while(0)
#define MOTORB_BACKOFF(pwm)  do{digitalWrite(DIRB1,HIGH);digitalWrite(DIRB2,LOW);  analogWrite(PWMB,pwm);}while(0)

#define MOTORC_FORWARD(pwm)  do{digitalWrite(DIRC1,LOW); digitalWrite(DIRC2,HIGH); analogWrite(PWMC,pwm);}while(0)
#define MOTORC_STOP(x)       do{digitalWrite(DIRC1,LOW); digitalWrite(DIRC2,LOW);  analogWrite(PWMC,0);}while(0)
#define MOTORC_BACKOFF(pwm)  do{digitalWrite(DIRC1,HIGH);digitalWrite(DIRC2,LOW);  analogWrite(PWMC,pwm);}while(0)

#define MOTORD_FORWARD(pwm)  do{digitalWrite(DIRD1,LOW); digitalWrite(DIRD2,HIGH); analogWrite(PWMD,pwm);}while(0)
#define MOTORD_STOP(x)       do{digitalWrite(DIRD1,LOW); digitalWrite(DIRD2,LOW);  analogWrite(PWMD,0);}while(0)
#define MOTORD_BACKOFF(pwm)  do{digitalWrite(DIRD1,HIGH);digitalWrite(DIRD2,LOW);  analogWrite(PWMD,pwm);}while(0)

// ═══════════════════════════════════════════════════════════════════════════
// ▶ 基础运动函数 (来自参考模板，适配可变 PWM)
// ═══════════════════════════════════════════════════════════════════════════

//    ↓A-----B↓        前进
//     |  ↓  |
//    ↓C-----D↓
void ADVANCE(uint8_t pwm) {
  MOTORA_FORWARD(pwm);
  MOTORB_BACKOFF(pwm);
  MOTORC_FORWARD(pwm);
  MOTORD_BACKOFF(pwm);
}

//    ↑A-----B↑        后退
//     |  ↑  |
//    ↑C-----D↑
void BACK(uint8_t pwm) {
  MOTORA_BACKOFF(pwm);
  MOTORB_FORWARD(pwm);
  MOTORC_BACKOFF(pwm);
  MOTORD_FORWARD(pwm);
}

//    ↑A-----B↓        向右横移
//     |  →  |
//    ↓C-----D↑
void RIGHT_2(uint8_t pwm) {
  MOTORA_FORWARD(pwm);
  MOTORB_FORWARD(pwm);
  MOTORC_BACKOFF(pwm);
  MOTORD_BACKOFF(pwm);
}

//    ↓A-----B↑        向左横移
//     |  ←  |
//    ↑C-----D↓
void LEFT_2(uint8_t pwm) {
  MOTORA_BACKOFF(pwm);
  MOTORB_BACKOFF(pwm);
  MOTORC_FORWARD(pwm);
  MOTORD_FORWARD(pwm);
}

//    ↑A-----B↓        顺时针旋转
//     | ↗ ↘ |
//    ↑C-----D↓
void rotate_1(uint8_t pwm) {
  MOTORA_BACKOFF(pwm);
  MOTORB_BACKOFF(pwm);
  MOTORC_BACKOFF(pwm);
  MOTORD_BACKOFF(pwm);
}

//    ↓A-----B↑        逆时针旋转
//     | ↙ ↖ |
//    ↓C-----D↑
void rotate_2(uint8_t pwm) {
  MOTORA_FORWARD(pwm);
  MOTORB_FORWARD(pwm);
  MOTORC_FORWARD(pwm);
  MOTORD_FORWARD(pwm);
}

//    =A-----B=        停止
//     |  =  |
//    =C-----D=
void STOP() {
  MOTORA_STOP(0);
  MOTORB_STOP(0);
  MOTORC_STOP(0);
  MOTORD_STOP(0);
}

// ═══════════════════════════════════════════════════════════════════════════
// ▶ 辅助运动函数 (考虑方向反转开关)
// ═══════════════════════════════════════════════════════════════════════════

// 向右横移 (考虑反转开关)
void strafeRight(uint8_t pwm) {
  if (INVERT_STRAFE_DIR) {
    LEFT_2(pwm);
  } else {
    RIGHT_2(pwm);
  }
}

// 向左横移 (考虑反转开关)
void strafeLeft(uint8_t pwm) {
  if (INVERT_STRAFE_DIR) {
    RIGHT_2(pwm);
  } else {
    LEFT_2(pwm);
  }
}

// 顺时针旋转 (考虑反转开关)
void rotateCW(uint8_t pwm) {
  if (INVERT_ROTATE_DIR) {
    rotate_2(pwm);
  } else {
    rotate_1(pwm);
  }
}

// 逆时针旋转 (考虑反转开关)
void rotateCCW(uint8_t pwm) {
  if (INVERT_ROTATE_DIR) {
    rotate_1(pwm);
  } else {
    rotate_2(pwm);
  }
}

// ═══════════════════════════════════════════════════════════════════════════
// ▶ 传感器读取函数
// ═══════════════════════════════════════════════════════════════════════════

// 读取光敏传感器 (返回模拟值，越大越亮)
int readLDR_L() {
  return analogRead(LDR_L_PIN);
}

int readLDR_R() {
  return analogRead(LDR_R_PIN);
}

// 超声波测距 (返回距离 cm，失败返回 999.0)
float measureUltrasonic(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  
  long duration = pulseIn(echoPin, HIGH, 30000);  // 超时 30ms
  if (duration == 0) {
    return 999.0;  // 测量失败
  }
  
  float distance = duration * 0.034 / 2.0;  // 声速 340m/s
  return distance;
}

// 中值滤波 (3 次测量取中值)
float medianOf3(float a, float b, float c) {
  if (a > b) {
    if (b > c) return b;
    else if (a > c) return c;
    else return a;
  } else {
    if (a > c) return a;
    else if (b > c) return c;
    else return b;
  }
}

// 读取前超声波 (带滤波)
float readUF() {
  float d1 = measureUltrasonic(UF_TRIG, UF_ECHO);
  delay(10);
  float d2 = measureUltrasonic(UF_TRIG, UF_ECHO);
  delay(10);
  float d3 = measureUltrasonic(UF_TRIG, UF_ECHO);
  return medianOf3(d1, d2, d3);
}

// 读取右超声波 (带滤波)
float readUR() {
  float d1 = measureUltrasonic(UR_TRIG, UR_ECHO);
  delay(10);
  float d2 = measureUltrasonic(UR_TRIG, UR_ECHO);
  delay(10);
  float d3 = measureUltrasonic(UR_TRIG, UR_ECHO);
  return medianOf3(d1, d2, d3);
}

// 交替更新超声波数据 (避免干扰)
void updateUltrasonicAlternating() {
  unsigned long now = millis();
  if (now - lastUltrasonicTime < US_PERIOD_MS) {
    return;  // 未到测量间隔
  }
  
  lastUltrasonicTime = now;
  
  if (ultrasonicToggle) {
    distUF = readUF();
  } else {
    distUR = readUR();
  }
  ultrasonicToggle = !ultrasonicToggle;
}

// ═══════════════════════════════════════════════════════════════════════════
// ▶ OLED 显示函数
// ═══════════════════════════════════════════════════════════════════════════

void displayStatus(const char* line1, const char* line2 = "") {
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println(line1);
  if (strlen(line2) > 0) {
    display.setCursor(0, 12);
    display.println(line2);
  }
  display.display();
}

void displayDistance(float dist) {
  display.clearDisplay();
  display.setTextSize(2);
  display.setCursor(0, 0);
  display.print(dist, 1);
  display.print("cm");
  display.display();
}

void displayParked() {
  display.clearDisplay();
  display.setTextSize(2);
  display.setCursor(0, 0);
  display.println("PARKED!");
  display.setTextSize(1);
  display.setCursor(0, 20);
  display.print("Dist: 8.0cm OK");
  display.display();
}

// ═══════════════════════════════════════════════════════════════════════════
// ▶ 安全检查函数 (急停逻辑)
// ═══════════════════════════════════════════════════════════════════════════

bool checkEmergencyStop() {
  // 检查是否需要急停 (任一传感器 < 6cm)
  if (distUF < STOP_GUARD_CM || distUR < STOP_GUARD_CM) {
    if (ENABLE_SERIAL_DEBUG) {
      Serial.println("!!! EMERGENCY STOP !!! Distance < 6cm");
    }
    
    // 立即停止
    STOP();
    delay(100);
    
    // 短暂后退
    BACK(PWM_CRAWL);
    delay(300);
    STOP();
    delay(100);
    
    // 返回 S3 重新对齐
    currentState = S3_ALIGNING;
    displayStatus("EMERGENCY", "Back to S3");
    delay(500);
    
    return true;  // 发生了急停
  }
  return false;  // 安全
}

// ═══════════════════════════════════════════════════════════════════════════
// ▶ 脉冲控制函数 (小幅度调整)
// ═══════════════════════════════════════════════════════════════════════════

// 脉冲旋转 (小角度)
void pulseRotate(bool clockwise, uint8_t pwm, int duration_ms) {
  if (clockwise) {
    rotateCW(pwm);
  } else {
    rotateCCW(pwm);
  }
  delay(duration_ms);
  STOP();
  delay(SETTLE_MS);
}

// 脉冲横移 (小距离)
void pulseStrafe(bool toRight, uint8_t pwm, int duration_ms) {
  if (toRight) {
    strafeRight(pwm);
  } else {
    strafeLeft(pwm);
  }
  delay(duration_ms);
  STOP();
  delay(SETTLE_MS);
}

// ═══════════════════════════════════════════════════════════════════════════
// ▶ 状态机函数
// ═══════════════════════════════════════════════════════════════════════════

// ─────────────────────────────────────────────────────────────────────────
// S1: 找光并旋转对齐
// ─────────────────────────────────────────────────────────────────────────
void state_LightSeeking() {
  if (lightLocked) {
    // 已锁定，进入 S2
    currentState = S2_APPROACHING;
    displayStatus("S1->S2", "Light Locked");
    delay(200);
    return;
  }
  
  // 读取光敏传感器
  int L = readLDR_L();
  int R = readLDR_R();
  int sum = L + R;
  int diff = abs(L - R);
  
  // 调试输出
  if (ENABLE_SERIAL_DEBUG && (millis() - lastDebugTime > 200)) {
    Serial.print("S1| L=");
    Serial.print(L);
    Serial.print(" R=");
    Serial.print(R);
    Serial.print(" Sum=");
    Serial.print(sum);
    Serial.print(" Diff=");
    Serial.print(diff);
    Serial.print(" Lock=");
    Serial.println(lockCounter);
    lastDebugTime = millis();
  }
  
  // OLED 显示
  char buf1[32], buf2[32];
  sprintf(buf1, "S1:SEEK L=%d R=%d", L, R);
  sprintf(buf2, "Sum=%d Df=%d Lk=%d", sum, diff, lockCounter);
  displayStatus(buf1, buf2);
  
  // 判断是否达到阈值
  if (sum > T_SUM && diff < T_DIFF) {
    lockCounter++;
    if (lockCounter >= N_LOCK) {
      lightLocked = true;
      STOP();
      if (ENABLE_SERIAL_DEBUG) {
        Serial.println("S1: Light locked!");
      }
    }
  } else {
    lockCounter = 0;  // 重置计数器
    
    // 根据左右差值旋转
    if (L > R) {
      // 左边更亮，逆时针旋转
      rotateCCW(PWM_SPIN);
    } else if (R > L) {
      // 右边更亮，顺时针旋转
      rotateCW(PWM_SPIN);
    } else {
      // 相等，缓慢旋转搜索
      rotateCW(PWM_SPIN);
    }
  }
}

// ─────────────────────────────────────────────────────────────────────────
// S2: 直线前进
// ─────────────────────────────────────────────────────────────────────────
void state_Approaching() {
  // 更新超声波数据
  updateUltrasonicAlternating();
  
  // 检查急停
  if (checkEmergencyStop()) {
    return;
  }
  
  // 调试输出
  if (ENABLE_SERIAL_DEBUG && (millis() - lastDebugTime > 200)) {
    Serial.print("S2| UF=");
    Serial.print(distUF);
    Serial.print(" UR=");
    Serial.println(distUR);
    lastDebugTime = millis();
  }
  
  // OLED 显示
  char buf1[32];
  sprintf(buf1, "S2:FWD UF=%.1fcm", distUF);
  displayStatus(buf1);
  
  // 检查是否到达近距离阈值
  if (distUF < NEAR_GATE_CM || distUR < NEAR_GATE_CM) {
    STOP();
    currentState = S3_ALIGNING;
    displayStatus("S2->S3", "Near wall");
    delay(300);
    return;
  }
  
  // 直线前进 (不修正方向)
  ADVANCE(PWM_FWD);
}

// ─────────────────────────────────────────────────────────────────────────
// S3: 平行对齐 (使 UF ≈ UR)
// ─────────────────────────────────────────────────────────────────────────
void state_Aligning() {
  // 更新超声波数据
  updateUltrasonicAlternating();
  
  // 检查急停
  if (checkEmergencyStop()) {
    return;
  }
  
  // 计算距离差 (考虑偏移补偿)
  float delta = (distUF + SENSOR_OFFSET) - distUR;
  
  // 调试输出
  if (ENABLE_SERIAL_DEBUG && (millis() - lastDebugTime > 200)) {
    Serial.print("S3| UF=");
    Serial.print(distUF);
    Serial.print(" UR=");
    Serial.print(distUR);
    Serial.print(" Delta=");
    Serial.println(delta);
    lastDebugTime = millis();
  }
  
  // OLED 显示
  char buf1[32], buf2[32];
  sprintf(buf1, "S3:ALIGN UF=%.1f", distUF);
  sprintf(buf2, "UR=%.1f Dlt=%.1f", distUR, delta);
  displayStatus(buf1, buf2);
  
  // 判断是否已对齐
  if (abs(delta) < DELTA_PAR) {
    STOP();
    currentState = S4_POSITIONING;
    displayStatus("S3->S4", "Aligned!");
    delay(300);
    return;
  }
  
  // 小角度旋转调整
  if (delta > 0) {
    // UF > UR，前端离墙更远，需要顺时针旋转
    pulseRotate(true, PWM_CRAWL, PULSE_MS);
  } else {
    // UF < UR，前端离墙更近，需要逆时针旋转
    pulseRotate(false, PWM_CRAWL, PULSE_MS);
  }
}

// ─────────────────────────────────────────────────────────────────────────
// S4: 距离调整 (纯横移至 8cm)
// ─────────────────────────────────────────────────────────────────────────
void state_Positioning() {
  // 更新超声波数据
  updateUltrasonicAlternating();
  
  // 检查急停
  if (checkEmergencyStop()) {
    return;
  }
  
  // 计算平均距离
  float avgDist = (distUF + distUR) / 2.0;
  float error = avgDist - TARGET_DIST_CM;
  
  // 调试输出
  if (ENABLE_SERIAL_DEBUG && (millis() - lastDebugTime > 200)) {
    Serial.print("S4| AvgDist=");
    Serial.print(avgDist);
    Serial.print(" Error=");
    Serial.println(error);
    lastDebugTime = millis();
  }
  
  // OLED 显示
  char buf1[32], buf2[32];
  sprintf(buf1, "S4:POS Avg=%.1fcm", avgDist);
  sprintf(buf2, "Tgt=%.1f Err=%.1f", TARGET_DIST_CM, error);
  displayStatus(buf1, buf2);
  
  // 判断是否到位
  if (abs(error) < EPSILON_DIST) {
    STOP();
    currentState = S5_PARKED;
    displayParked();
    if (ENABLE_SERIAL_DEBUG) {
      Serial.println("S4->S5: PARKED!");
    }
    return;
  }
  
  // 脉冲横移调整
  if (error > 0) {
    // 距离太远，向右横移 (靠近墙)
    pulseStrafe(true, PWM_CRAWL, PULSE_MS);
  } else {
    // 距离太近，向左横移 (远离墙)
    pulseStrafe(false, PWM_CRAWL, PULSE_MS);
  }
}

// ─────────────────────────────────────────────────────────────────────────
// S5: 停车完成
// ─────────────────────────────────────────────────────────────────────────
void state_Parked() {
  // 确保停止
  STOP();
  
  // 显示完成状态
  displayParked();
  
  // 可选：输出最终数据
  if (ENABLE_SERIAL_DEBUG) {
    Serial.println("==================");
    Serial.println("  PARKING COMPLETE");
    Serial.println("==================");
    Serial.print("Final UF: ");
    Serial.print(distUF);
    Serial.println(" cm");
    Serial.print("Final UR: ");
    Serial.print(distUR);
    Serial.println(" cm");
    Serial.print("Average: ");
    Serial.print((distUF + distUR) / 2.0);
    Serial.println(" cm");
  }
  
  // 停在这里，不再执行其他操作
  delay(1000);
}

// ═══════════════════════════════════════════════════════════════════════════
// ▶ Arduino Setup
// ═══════════════════════════════════════════════════════════════════════════
void setup() {
  // 串口初始化
  Serial.begin(115200);
  Serial.println("=================================");
  Serial.println("  Auto Parking System Started");
  Serial.println("=================================");
  
  // --- 电机引脚初始化 ---
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
  
  // 确保电机停止
  STOP();
  
  // --- 光敏传感器引脚初始化 ---
  pinMode(LDR_L_PIN, INPUT);
  pinMode(LDR_R_PIN, INPUT);
  
  // --- 超声波引脚初始化 ---
  pinMode(UF_TRIG, OUTPUT);
  pinMode(UF_ECHO, INPUT);
  pinMode(UR_TRIG, OUTPUT);
  pinMode(UR_ECHO, INPUT);
  
  // --- OLED 初始化 ---
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println("ERROR: OLED init failed!");
  }
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("Auto Parking");
  display.println("System Ready");
  display.display();
  
  // --- 环境光标定 (可选) ---
  delay(1000);
  int L0 = readLDR_L();
  int R0 = readLDR_R();
  ambientBrightness = (L0 + R0) / 2.0;
  
  Serial.print("Ambient Light: L=");
  Serial.print(L0);
  Serial.print(" R=");
  Serial.print(R0);
  Serial.print(" Avg=");
  Serial.println(ambientBrightness);
  
  // --- 参数确认 ---
  Serial.println("\n--- System Parameters ---");
  Serial.print("PWM_FWD=");      Serial.println(PWM_FWD);
  Serial.print("PWM_STRAFE=");   Serial.println(PWM_STRAFE);
  Serial.print("PWM_SPIN=");     Serial.println(PWM_SPIN);
  Serial.print("PWM_CRAWL=");    Serial.println(PWM_CRAWL);
  Serial.print("T_SUM=");        Serial.println(T_SUM);
  Serial.print("T_DIFF=");       Serial.println(T_DIFF);
  Serial.print("NEAR_GATE=");    Serial.print(NEAR_GATE_CM);   Serial.println("cm");
  Serial.print("TARGET_DIST=");  Serial.print(TARGET_DIST_CM); Serial.println("cm");
  Serial.print("STOP_GUARD=");   Serial.print(STOP_GUARD_CM);  Serial.println("cm");
  Serial.println("-------------------------\n");
  
  delay(1000);
  
  // 开始状态机
  currentState = S1_LIGHT_SEEKING;
  Serial.println("Starting S1: Light Seeking...");
}

// ═══════════════════════════════════════════════════════════════════════════
// ▶ Arduino Loop
// ═══════════════════════════════════════════════════════════════════════════
void loop() {
  // 状态机调度
  switch (currentState) {
    case S1_LIGHT_SEEKING:
      state_LightSeeking();
      break;
      
    case S2_APPROACHING:
      state_Approaching();
      break;
      
    case S3_ALIGNING:
      state_Aligning();
      break;
      
    case S4_POSITIONING:
      state_Positioning();
      break;
      
    case S5_PARKED:
      state_Parked();
      break;
      
    default:
      Serial.println("ERROR: Unknown state!");
      STOP();
      break;
  }
  
  // 小延迟，避免 CPU 占用过高
  delay(20);
}

/*
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  📝 调试建议
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

【参数调优顺序】
1. 先在 S1 阶段测试光敏传感器，调整 T_SUM、T_DIFF、N_LOCK
2. 在 S2 阶段测试前进速度，调整 PWM_FWD
3. 在 S3 阶段测试对齐逻辑，调整 DELTA_PAR、PWM_CRAWL、PULSE_MS
4. 在 S4 阶段测试横移精度，调整 EPSILON_DIST

【常见问题】
- 如果横移方向反了，设置 INVERT_STRAFE_DIR = true
- 如果旋转方向反了，设置 INVERT_ROTATE_DIR = true
- 如果超声波读数不稳定，增加 US_PERIOD_MS
- 如果脉冲调整幅度太大，减小 PULSE_MS 或 PWM_CRAWL
- 如果传感器安装不对称，调整 SENSOR_OFFSET

【串口输出格式】
S1| L=xxx R=xxx Sum=xxx Diff=xxx Lock=x
S2| UF=xx.x UR=xx.x
S3| UF=xx.x UR=xx.x Delta=xx.x
S4| AvgDist=xx.x Error=xx.x

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
*/

