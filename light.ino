// ────────────────────────────────────────────────────────────────
//  只用感光元件寻找光源并对准（无超声、无陀螺）
//  Arduino Mega 2560 + 麦克纳姆四轮（仅用原地旋转动作）
// ────────────────────────────────────────────────────────────────

#include <Arduino.h>

// ================== 可根据实际接线修改 ==================
// 光敏传感器：建议将每只 LDR 做分压后把“信号点”接到以下模拟口
#define LDR_L_PIN  A8   // 左 LDR 模拟输入（示例：A8）
#define LDR_R_PIN  A9   // 右 LDR 模拟输入（示例：A9）

// 车体电机引脚（沿用你们模板的定义）
#define PWMA 12
#define DIRA1 34
#define DIRA2 35

#define PWMB 8
#define DIRB1 37
#define DIRB2 36

#define PWMC 9
#define DIRC1 43
#define DIRC2 42

#define PWMD 5
#define DIRD1 A4
#define DIRD2 A5

// 旋转方向反转开关（若转向与预期反了，改为 true）
bool INVERT_ROTATE_DIR = false;

// 旋转 PWM（0~255），建议低速旋转更稳
uint8_t PWM_SPIN = 120;

// 亮度阈值与死区
// T_SUM：只有当 L+R 大于该值才算“看到亮光”（防止在黑暗里误对齐）
// T_DIFF：左右差值小于该值才认为“已经居中对准”
int T_SUM   = 1500;  // 0~2046（两路 0~1023 累加）
int T_DIFF  = 50;    // 0~1023（差值阈值，越小越严格）
int N_LOCK  = 6;     // 连续满足帧数（去抖）

// 采样与滤波
uint8_t SMOOTH_N = 5;  // 简单滑动平均窗口

// ================== 电机动作宏（0~255 PWM） ==================
#define MOTORA_FORWARD(pwm)  do{digitalWrite(DIRA1,LOW);  digitalWrite(DIRA2,HIGH); analogWrite(PWMA,constrain(pwm,0,255));}while(0)
#define MOTORA_BACKOFF(pwm)  do{digitalWrite(DIRA1,HIGH); digitalWrite(DIRA2,LOW);  analogWrite(PWMA,constrain(pwm,0,255));}while(0)
#define MOTORA_STOP()        do{digitalWrite(DIRA1,LOW);  digitalWrite(DIRA2,LOW);  analogWrite(PWMA,0);}while(0)

#define MOTORB_FORWARD(pwm)  do{digitalWrite(DIRB1,LOW);  digitalWrite(DIRB2,HIGH); analogWrite(PWMB,constrain(pwm,0,255));}while(0)
#define MOTORB_BACKOFF(pwm)  do{digitalWrite(DIRB1,HIGH); digitalWrite(DIRB2,LOW);  analogWrite(PWMB,constrain(pwm,0,255));}while(0)
#define MOTORB_STOP()        do{digitalWrite(DIRB1,LOW);  digitalWrite(DIRB2,LOW);  analogWrite(PWMB,0);}while(0)

#define MOTORC_FORWARD(pwm)  do{digitalWrite(DIRC1,LOW);  digitalWrite(DIRC2,HIGH); analogWrite(PWMC,constrain(pwm,0,255));}while(0)
#define MOTORC_BACKOFF(pwm)  do{digitalWrite(DIRC1,HIGH); digitalWrite(DIRC2,LOW);  analogWrite(PWMC,constrain(pwm,0,255));}while(0)
#define MOTORC_STOP()        do{digitalWrite(DIRC1,LOW);  digitalWrite(DIRC2,LOW);  analogWrite(PWMC,0);}while(0)

#define MOTORD_FORWARD(pwm)  do{digitalWrite(DIRD1,LOW);  digitalWrite(DIRD2,HIGH); analogWrite(PWMD,constrain(pwm,0,255));}while(0)
#define MOTORD_BACKOFF(pwm)  do{digitalWrite(DIRD1,HIGH); digitalWrite(DIRD2,LOW);  analogWrite(PWMD,constrain(pwm,0,255));}while(0)
#define MOTORD_STOP()        do{digitalWrite(DIRD1,LOW);  digitalWrite(DIRD2,LOW);  analogWrite(PWMD,0);}while(0)

void STOP(){
  MOTORA_STOP(); MOTORB_STOP(); MOTORC_STOP(); MOTORD_STOP();
}

// 原地旋转：顺时针 / 逆时针（根据装轮镜像，必要时用 INVERT_ROTATE_DIR 反转）
void rotateCW(uint8_t pwm){
  if (INVERT_ROTATE_DIR){
    // 实际反转
    // “逆时针”实现
    MOTORA_FORWARD(pwm); MOTORB_FORWARD(pwm); MOTORC_FORWARD(pwm); MOTORD_FORWARD(pwm);
  }else{
    // “顺时针”实现（四轮同向）
    MOTORA_BACKOFF(pwm); MOTORB_BACKOFF(pwm); MOTORC_BACKOFF(pwm); MOTORD_BACKOFF(pwm);
  }
}
void rotateCCW(uint8_t pwm){
  if (INVERT_ROTATE_DIR){
    // 反转
    MOTORA_BACKOFF(pwm); MOTORB_BACKOFF(pwm); MOTORC_BACKOFF(pwm); MOTORD_BACKOFF(pwm);
  }else{
    MOTORA_FORWARD(pwm); MOTORB_FORWARD(pwm); MOTORC_FORWARD(pwm); MOTORD_FORWARD(pwm);
  }
}

// ================== 简单滑动平均 ==================
int readSmooth(uint8_t pin){
  long acc = 0;
  for(uint8_t i=0;i<SMOOTH_N;i++){
    acc += analogRead(pin);
    delay(3);
  }
  return (int)(acc / SMOOTH_N);
}

// ================== 主流程 ==================
int lockCnt = 0;

void setup(){
  Serial.begin(115200);

  // 电机引脚输出
  pinMode(PWMA,OUTPUT); pinMode(DIRA1,OUTPUT); pinMode(DIRA2,OUTPUT);
  pinMode(PWMB,OUTPUT); pinMode(DIRB1,OUTPUT); pinMode(DIRB2,OUTPUT);
  pinMode(PWMC,OUTPUT); pinMode(DIRC1,OUTPUT); pinMode(DIRC2,OUTPUT);
  pinMode(PWMD,OUTPUT); pinMode(DIRD1,OUTPUT); pinMode(DIRD2,OUTPUT);
  STOP();

  // 光敏输入
  pinMode(LDR_L_PIN, INPUT);
  pinMode(LDR_R_PIN, INPUT);

  Serial.println(F("Light-only pointing demo ready."));
  Serial.println(F("Rotate to face the brightest direction, then stop."));
}

void loop(){
  // 采样
  int L = readSmooth(LDR_L_PIN);   // 左
  int R = readSmooth(LDR_R_PIN);   // 右
  int sum  = L + R;
  int diff = abs(L - R);

  // 串口观察
  static unsigned long lastPrint=0;
  if (millis() - lastPrint > 200){
    Serial.print(F("L=")); Serial.print(L);
    Serial.print(F(" R=")); Serial.print(R);
    Serial.print(F(" Sum=")); Serial.print(sum);
    Serial.print(F(" Diff=")); Serial.print(diff);
    Serial.print(F(" Lock=")); Serial.println(lockCnt);
    lastPrint = millis();
  }

  // 阈值判定：亮度够 & 左右几乎相等 → 计数；否则清零并继续旋转
  if (sum > T_SUM && diff < T_DIFF){
    lockCnt++;
    if (lockCnt >= N_LOCK){
      STOP();                      // 视为对准光源
      // 保持停止（如果想继续跟随，可把 lockCnt 清0 并继续循环）
      delay(50);
      return;
    }
    // 对准过程中也保持轻微旋转可选：这里选择“停住”，避免抖动
    STOP();
  }else{
    lockCnt = 0;
    // 哪边更亮向哪边缓慢转
    if (L > R){
      rotateCCW(PWM_SPIN);  // 左更亮 → 逆时针转向左
    }else if (R > L){
      rotateCW(PWM_SPIN);   // 右更亮 → 顺时针转向右
    }else{
      // 完全相等时缓慢扫描
      rotateCW(PWM_SPIN);
    }
  }

  delay(10); // 小节拍
}
