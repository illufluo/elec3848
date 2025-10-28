// ────────────────────────────────────────────────────────────────
// 只用 A0/A1 两个光敏传感器，让车头对准光源（无超声/无IMU）
// 标定参照你们 lab：环境光、遮挡，得到 0~100 强度；用 TOL/K 逐步逼近
// ────────────────────────────────────────────────────────────────
#include <Arduino.h>

// ==== LDR 模拟口（按你们 lab 用 A0/A1） ====
#define LDR_LEFT_PIN   A0
#define LDR_RIGHT_PIN  A1

// ==== 麦克纳姆底盘电机引脚（与你原工程一致） ====
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

// 电机宏（0~255）
#define MOTORA_FORWARD(p)  do{digitalWrite(DIRA1,LOW);  digitalWrite(DIRA2,HIGH); analogWrite(PWMA,constrain(p,0,255));}while(0)
#define MOTORA_BACKOFF(p)  do{digitalWrite(DIRA1,HIGH); digitalWrite(DIRA2,LOW);  analogWrite(PWMA,constrain(p,0,255));}while(0)
#define MOTORA_STOP()      do{digitalWrite(DIRA1,LOW);  digitalWrite(DIRA2,LOW);  analogWrite(PWMA,0);}while(0)
#define MOTORB_FORWARD(p)  do{digitalWrite(DIRB1,LOW);  digitalWrite(DIRB2,HIGH); analogWrite(PWMB,constrain(p,0,255));}while(0)
#define MOTORB_BACKOFF(p)  do{digitalWrite(DIRB1,HIGH); digitalWrite(DIRB2,LOW);  analogWrite(PWMB,constrain(p,0,255));}while(0)
#define MOTORB_STOP()      do{digitalWrite(DIRB1,LOW);  digitalWrite(DIRB2,LOW);  analogWrite(PWMB,0);}while(0)
#define MOTORC_FORWARD(p)  do{digitalWrite(DIRC1,LOW);  digitalWrite(DIRC2,HIGH); analogWrite(PWMC,constrain(p,0,255));}while(0)
#define MOTORC_BACKOFF(p)  do{digitalWrite(DIRC1,HIGH); digitalWrite(DIRC2,LOW);  analogWrite(PWMC,constrain(p,0,255));}while(0)
#define MOTORC_STOP()      do{digitalWrite(DIRC1,LOW);  digitalWrite(DIRC2,LOW);  analogWrite(PWMC,0);}while(0)
#define MOTORD_FORWARD(p)  do{digitalWrite(DIRD1,LOW);  digitalWrite(DIRD2,HIGH); analogWrite(PWMD,constrain(p,0,255));}while(0)
#define MOTORD_BACKOFF(p)  do{digitalWrite(DIRD1,HIGH); digitalWrite(DIRD2,LOW);  analogWrite(PWMD,constrain(p,0,255));}while(0)
#define MOTORD_STOP()      do{digitalWrite(DIRD1,LOW);  digitalWrite(DIRD2,LOW);  analogWrite(PWMD,0);}while(0)

void STOP(){ MOTORA_STOP(); MOTORB_STOP(); MOTORC_STOP(); MOTORD_STOP(); }

// 顺/逆时针原地旋（如方向与预期相反，把 INVERT_ROTATE 置 true）
bool INVERT_ROTATE = false;
void rotateCW(uint8_t pwm){   // 顺时针
  if (INVERT_ROTATE){ MOTORA_FORWARD(pwm); MOTORB_FORWARD(pwm); MOTORC_FORWARD(pwm); MOTORD_FORWARD(pwm); }
  else               { MOTORA_BACKOFF(pwm); MOTORB_BACKOFF(pwm); MOTORC_BACKOFF(pwm); MOTORD_BACKOFF(pwm); }
}
void rotateCCW(uint8_t pwm){  // 逆时针
  if (INVERT_ROTATE){ MOTORA_BACKOFF(pwm); MOTORB_BACKOFF(pwm); MOTORC_BACKOFF(pwm); MOTORD_BACKOFF(pwm); }
  else               { MOTORA_FORWARD(pwm); MOTORB_FORWARD(pwm); MOTORC_FORWARD(pwm); MOTORD_FORWARD(pwm); }
}

// ==== 参数（等价你们 lab 的 TOL / K） ====
int    TOL          = 5;      // 左右强度的“死区”（单位：强度 0~100）
int    N_LOCK       = 6;      // 连续满足死区帧数才锁定
int    BRIGHT_MIN   = 20;     // 总亮度阈值（避免黑暗中误锁定），0~200
uint8_t PWM_SPIN    = 120;    // 旋转速度（低速更稳）
int    PULSE_MS     = 70;     // 旋转脉冲时长
int    SETTLE_MS    = 90;     // 脉冲后静置再测

// ==== 标定得到的线性映射（模仿你们 lab） ====
int adcL_amb=0, adcR_amb=0;   // 环境光 ADC
int adcL_blk=0, adcR_blk=0;   // 遮挡（近似 0 光）ADC
float mL=1, mR=1;             // 斜率： (amb - blk)/100
// 强度 0~100 计算： (读数 - blk)/m

// 平滑读取
int readSmooth(uint8_t pin){
  long acc=0; for(int i=0;i<5;i++){ acc+=analogRead(pin); delay(2); } return acc/5;
}

void calibrate(){
  Serial.println(F("校准1/3：请把传感器置于环境光下，保持不动 5 秒..."));
  delay(5000);
  adcL_amb = readSmooth(LDR_LEFT_PIN);
  adcR_amb = readSmooth(LDR_RIGHT_PIN);
  Serial.print(F("环境光 Left=")); Serial.print(adcL_amb);
  Serial.print(F(" Right=")); Serial.println(adcR_amb);

  Serial.println(F("\n校准2/3：请用手指覆盖两只传感器，持续 5 秒..."));
  delay(2000);
  Serial.println(F("开始采样..."));
  delay(3000);
  adcL_blk = readSmooth(LDR_LEFT_PIN);
  adcR_blk = readSmooth(LDR_RIGHT_PIN);
  Serial.print(F("遮挡 Left=")); Serial.print(adcL_blk);
  Serial.print(F(" Right=")); Serial.println(adcR_blk);

  // 斜率（避免除0）
  mL = max(1, (adcL_amb - adcL_blk)) / 100.0f;
  mR = max(1, (adcR_amb - adcR_blk)) / 100.0f;

  Serial.println(F("\n校准3/3：完成，移开手，准备开始。"));
  delay(1000);
}

void setup(){
  Serial.begin(115200);

  // 电机引脚
  pinMode(PWMA,OUTPUT); pinMode(DIRA1,OUTPUT); pinMode(DIRA2,OUTPUT);
  pinMode(PWMB,OUTPUT); pinMode(DIRB1,OUTPUT); pinMode(DIRB2,OUTPUT);
  pinMode(PWMC,OUTPUT); pinMode(DIRC1,OUTPUT); pinMode(DIRC2,OUTPUT);
  pinMode(PWMD,OUTPUT); pinMode(DIRD1,OUTPUT); pinMode(DIRD2,OUTPUT);
  STOP();

  // LDR
  pinMode(LDR_LEFT_PIN,  INPUT);
  pinMode(LDR_RIGHT_PIN, INPUT);

  calibrate();
  Serial.println(F("\n开始找光：哪侧更亮向哪侧小脉冲旋转；进入死区连续 N 次即锁定并停止"));
}

void loop(){
  // 读原始 ADC
  int rawL = readSmooth(LDR_LEFT_PIN);
  int rawR = readSmooth(LDR_RIGHT_PIN);

  // 映射为强度 0~100（可超界，后面夹紧）
  int left  = (int)((rawL - adcL_blk)/mL);
  int right = (int)((rawR - adcR_blk)/mR);
  left  = constrain(left,  0, 100);
  right = constrain(right, 0, 100);

  int diff = left - right;              // 正：左更亮；负：右更亮
  int sumI = left + right;              // 总亮度 0~200
  static int lockCnt=0;

  // 串口观察
  static unsigned long t0=0;
  if(millis()-t0>200){
    Serial.print(F("L=")); Serial.print(left);
    Serial.print(F(" R=")); Serial.print(right);
    Serial.print(F(" diff=")); Serial.print(diff);
    Serial.print(F(" sum=")); Serial.print(sumI);
    Serial.print(F(" lock=")); Serial.println(lockCnt);
    t0=millis();
  }

  // 判定：亮度足够 且 进入死区 → 计数锁定
  if (sumI >= BRIGHT_MIN && abs(diff) <= TOL){
    lockCnt++;
    STOP();
    if (lockCnt >= N_LOCK){
      // 对正 → 长停
      STOP();
      // 若想持续跟随，把 lockCnt 清零即可
      delay(20);
      return;
    }
  }else{
    lockCnt = 0;
    // 逐步逼近：向更亮一侧做一个“短脉冲旋转”，再停→稳→再测
    if (diff > TOL){
      // 左更亮 → 逆时针一小步
      rotateCCW(PWM_SPIN);
      delay(PULSE_MS);
      STOP();
      delay(SETTLE_MS);
    }else if (diff < -TOL){
      // 右更亮 → 顺时针一小步
      rotateCW(PWM_SPIN);
      delay(PULSE_MS);
      STOP();
      delay(SETTLE_MS);
    }else{
      // 亮度不够或刚好在死区但总亮度太低：做慢速扫描（避免卡死）
      rotateCW(max<uint8_t>(60, PWM_SPIN/2));
      delay(50);
      STOP();
      delay(70);
    }
  }
}
