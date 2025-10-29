/*
 * autopark_ultrasonic_only.ino
 * 最小可用版：仅用两个 HC-SR04，假设上电时已正对目标墙。
 * 行为：对齐(|dL-dR|<=ALIGN_EPS_CM) -> 慢速到 8±1 cm -> 停车；任一侧<MIN_SAFE_CM 紧急刹停。
 * 说明：沿用你 reference 的电机引脚与宏风格；analogWrite 统一 0~255。
 */

 #include <Arduino.h>
 #include <Wire.h>
 #include <SPI.h>
 #include <Adafruit_GFX.h>
 #include <Adafruit_SSD1306.h>
 #include <Servo.h>
 
 /* ---------------- OLED ---------------- */
 #define SCREEN_WIDTH 128
 #define SCREEN_HEIGHT 32
 #define OLED_RESET 28
 Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
 
 /* ---------------- 舵机（云台）---------------- */
 Servo servo_pan, servo_tilt;
 int pan = 90, tilt = 120;
 const int servo_min = 20, servo_max = 160;
 
 /* ---------------- 电机引脚 & 宏（与 reference 对齐）---------------- */
 // PWM
 #define PWMA 12
 #define PWMB 8
 #define PWMC 9
 #define PWMD 5
 // 方向
 #define DIRA1 34
 #define DIRA2 35
 #define DIRB1 37
 #define DIRB2 36
 #define DIRC1 43
 #define DIRC2 42
 #define DIRD1 A4
 #define DIRD2 A5
 
 // 统一 PWM（0~255）
 int Motor_PWM = 160;
 
 // A 电机
 #define MOTORA_FORWARD(pwm)  do{digitalWrite(DIRA1,LOW);  digitalWrite(DIRA2,HIGH); analogWrite(PWMA,Motor_PWM);}while(0)
 #define MOTORA_BACKOFF(pwm)  do{digitalWrite(DIRA1,HIGH); digitalWrite(DIRA2,LOW);  analogWrite(PWMA,Motor_PWM);}while(0)
 #define MOTORA_STOP(x)       do{digitalWrite(DIRA1,LOW);  digitalWrite(DIRA2,LOW);  analogWrite(PWMA,0);}while(0)
 // B 电机
 #define MOTORB_FORWARD(pwm)  do{digitalWrite(DIRB1,LOW);  digitalWrite(DIRB2,HIGH); analogWrite(PWMB,Motor_PWM);}while(0)
 #define MOTORB_BACKOFF(pwm)  do{digitalWrite(DIRB1,HIGH); digitalWrite(DIRB2,LOW);  analogWrite(PWMB,Motor_PWM);}while(0)
 #define MOTORB_STOP(x)       do{digitalWrite(DIRB1,LOW);  digitalWrite(DIRB2,LOW);  analogWrite(PWMB,0);}while(0)
 // C 电机
 #define MOTORC_FORWARD(pwm)  do{digitalWrite(DIRC1,LOW);  digitalWrite(DIRC2,HIGH); analogWrite(PWMC,Motor_PWM);}while(0)
 #define MOTORC_BACKOFF(pwm)  do{digitalWrite(DIRC1,HIGH); digitalWrite(DIRC2,LOW);  analogWrite(PWMC,Motor_PWM);}while(0)
 #define MOTORC_STOP(x)       do{digitalWrite(DIRC1,LOW);  digitalWrite(DIRC2,LOW);  analogWrite(PWMC,0);}while(0)
 // D 电机
 #define MOTORD_FORWARD(pwm)  do{digitalWrite(DIRD1,LOW);  digitalWrite(DIRD2,HIGH); analogWrite(PWMD,Motor_PWM);}while(0)
 #define MOTORD_BACKOFF(pwm)  do{digitalWrite(DIRD1,HIGH); digitalWrite(DIRD2,LOW);  analogWrite(PWMD,Motor_PWM);}while(0)
 #define MOTORD_STOP(x)       do{digitalWrite(DIRD1,LOW);  digitalWrite(DIRD2,LOW);  analogWrite(PWMD,0);}while(0)
 
 // 动作组合（与 reference 语义一致）
 void ADVANCE(){           // 前进
   MOTORA_FORWARD(Motor_PWM); MOTORB_BACKOFF(Motor_PWM);
   MOTORC_FORWARD(Motor_PWM); MOTORD_BACKOFF(Motor_PWM);
 }
 void BACK(){              // 后退
   MOTORA_BACKOFF(Motor_PWM); MOTORB_FORWARD(Motor_PWM);
   MOTORC_BACKOFF(Motor_PWM); MOTORD_FORWARD(Motor_PWM);
 }
 void rotate_1(){          // 原地旋（方向与 reference 保持）
   MOTORA_BACKOFF(Motor_PWM); MOTORB_BACKOFF(Motor_PWM);
   MOTORC_BACKOFF(Motor_PWM); MOTORD_BACKOFF(Motor_PWM);
 }
 void rotate_2(){          // 原地反向旋
   MOTORA_FORWARD(Motor_PWM); MOTORB_FORWARD(Motor_PWM);
   MOTORC_FORWARD(Motor_PWM); MOTORD_FORWARD(Motor_PWM);
 }
 void STOP_ALL(){ MOTORA_STOP(0); MOTORB_STOP(0); MOTORC_STOP(0); MOTORD_STOP(0); }
 
 /* ---------------- 两个 HC-SR04 ---------------- */
 #define ULTRA_L_TRIG 22
 #define ULTRA_L_ECHO 23
 #define ULTRA_R_TRIG 24
 #define ULTRA_R_ECHO 25
 
 // 距离阈值（cm）
 #define TARGET_DIST_CM   8
 #define TARGET_EPS_CM    1
 #define ALIGN_EPS_CM     1.5
 #define MIN_SAFE_CM      7
 #define MAX_VALID_CM     200
 
 // 速度（0~255）
 #define SPD_FAST 180
 #define SPD_SLOW 120
 #define SPD_TURN 100
 
 // 读取超声
 long readUltrasonicCM(uint8_t trig, uint8_t echo){
   digitalWrite(trig, LOW);  delayMicroseconds(2);
   digitalWrite(trig, HIGH); delayMicroseconds(10);
   digitalWrite(trig, LOW);
   unsigned long dur = pulseIn(echo, HIGH, 30000UL); // 30ms 超时
   if (dur == 0) return -1;
   return (long)(dur / 58); // us/58 ≈ cm
 }
 // 三次取中值
 long smoothUltraCM(uint8_t trig, uint8_t echo){
   const int N=3; long v[N];
   for(int i=0;i<N;i++){ v[i]=readUltrasonicCM(trig,echo); if(v[i]<0) return -1; }
   for(int i=0;i<N-1;i++) for(int j=i+1;j<N;j++) if(v[j]<v[i]){ long t=v[i]; v[i]=v[j]; v[j]=t; }
   return v[1];
 }
 
 /* ---------------- 动作封装 ---------------- */
 inline void driveStop(){ STOP_ALL(); }
 inline void driveForwardFast(){ Motor_PWM = SPD_FAST; ADVANCE(); }
 inline void driveForwardSlow(){ Motor_PWM = SPD_SLOW; ADVANCE(); }
 inline void driveBackwardSlow(){ Motor_PWM = SPD_SLOW; BACK(); }
 inline void turnLeft(){ Motor_PWM = SPD_TURN; rotate_2(); }   // 如方向相反，与 turnRight 互换
 inline void turnRight(){ Motor_PWM = SPD_TURN; rotate_1(); }
 
 /* ---------------- 状态机 ---------------- */
 enum State { IDLE, ALIGN_WALL, PARK_8CM, DONE, FAIL };
 State state = IDLE;
 
 unsigned long lastTick = 0;
 unsigned long lastTelemetry = 0;
 
 /* ---------------- setup ---------------- */
 void setup(){
   Serial.begin(115200);
 
   // 电机引脚
   pinMode(PWMA,OUTPUT); pinMode(PWMB,OUTPUT);
   pinMode(PWMC,OUTPUT); pinMode(PWMD,OUTPUT);
   pinMode(DIRA1,OUTPUT); pinMode(DIRA2,OUTPUT);
   pinMode(DIRB1,OUTPUT); pinMode(DIRB2,OUTPUT);
   pinMode(DIRC1,OUTPUT); pinMode(DIRC2,OUTPUT);
   pinMode(DIRD1,OUTPUT); pinMode(DIRD2,OUTPUT);
   driveStop();
 
   // 舵机（与 reference 一致：48/47）
   servo_pan.attach(48);
   servo_tilt.attach(47);
 
   // OLED
   if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
     // OLED 失败也不阻止运行
   }
   display.clearDisplay();
   display.setTextSize(2);
   display.setTextColor(SSD1306_WHITE);
   display.setCursor(0,0);
   display.println("US Park v1");
   display.display();
 
   // 超声引脚
   pinMode(ULTRA_L_TRIG, OUTPUT); pinMode(ULTRA_L_ECHO, INPUT);
   pinMode(ULTRA_R_TRIG, OUTPUT); pinMode(ULTRA_R_ECHO, INPUT);
 
   state = ALIGN_WALL; // 上电即开始：先对齐再停车
 }
 
 /* ---------------- loop ---------------- */
 void loop(){
   // ~66Hz 调度
   if (millis() - lastTick >= 15){
     lastTick += 15;
 
     // 云台保持限幅（可选）
     pan  = constrain(pan, servo_min, servo_max);
     tilt = constrain(tilt, servo_min, servo_max);
     servo_pan.write(pan);
     servo_tilt.write(tilt);
 
     // 读超声
     long dL = smoothUltraCM(ULTRA_L_TRIG, ULTRA_L_ECHO);
     long dR = smoothUltraCM(ULTRA_R_TRIG, ULTRA_R_ECHO);
     bool Lok = (dL>0 && dL<MAX_VALID_CM);
     bool Rok = (dR>0 && dR<MAX_VALID_CM);
 
     // 安全急停
     if ((Lok && dL<MIN_SAFE_CM) || (Rok && dR<MIN_SAFE_CM)){
       driveStop(); state = FAIL;
     }
 
     // 串口遥测（~10Hz）
     if (millis() - lastTelemetry > 100){
       lastTelemetry = millis();
       Serial.print("STATE,"); Serial.print(state);
       Serial.print(", UL,"); Serial.print(dL);
       Serial.print(", UR,"); Serial.print(dR);
       Serial.println();
     }
 
     switch(state){
       case IDLE:
         driveStop();
         break;
 
       case ALIGN_WALL: {
         // 需要双侧同时有效才推进；否则原地慢转找回双侧有效
         if (!(Lok && Rok)){
           // 小角速度旋转，直到双侧都有效
           turnLeft();
           break;
         }
         float diff = (float)dL - (float)dR; // 正=左更远->右转
         if (fabs(diff) <= ALIGN_EPS_CM){
           driveStop();
           state = PARK_8CM;
           display.clearDisplay(); display.setCursor(0,0);
           display.println("Aligned"); display.display();
         }else{
           if (diff > 0) turnRight(); else turnLeft();
         }
       } break;
 
       case PARK_8CM: {
         if (!(Lok && Rok)){ // 丢测则回对齐阶段
           driveStop(); state = ALIGN_WALL; break;
         }
         float avg  = 0.5f * (dL + dR);
         float diff = fabs((float)dL - (float)dR);
 
         // 若走偏，先纠姿再走
         if (diff > ALIGN_EPS_CM * 2.0f){
           if (dL > dR) turnRight(); else turnLeft();
           break;
         }
 
         if (avg > TARGET_DIST_CM + TARGET_EPS_CM){
           driveForwardSlow();
         }else if (avg < TARGET_DIST_CM - TARGET_EPS_CM){
           driveBackwardSlow(); // 轻微倒回
         }else{
           driveStop(); state = DONE;
           display.clearDisplay(); display.setCursor(0,0);
           display.println("PARKED 8cm"); display.display();
         }
       } break;
 
       case DONE:
         driveStop();
         break;
 
       case FAIL:
         driveStop();
         display.clearDisplay(); display.setCursor(0,0);
         display.println("EMERGENCY!"); display.display();
         break;
     }
   }
 }
 