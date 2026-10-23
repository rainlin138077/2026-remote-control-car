#include <PulsePosition.h>
#include <Servo.h>

// ==========================================
// 1. PPM 接收器與 MG996R 舵機設定
// ==========================================
PulsePositionInput ReceiverInput(RISING); 
float ReceiverValue[10] = {0}; // 儲存 10 個通道資料

const int PPM_PIN = 9;      // FS-iA6B PPM 訊號線接在 Pin 9
const int SERVO_PIN = 35;   // MG996R 馬達訊號線接在 Pin 35
Servo mg996Servo;

// ==========================================
// 2. TB6612FNG 馬達腳位 (已更正 LR 腳位)
// ==========================================
const int STBY = 6; // 左側 STBY 腳位

// --- 左前輪 (LF) ---
const int LF_PWMA = 3;
const int LF_AIN2 = 4;
const int LF_AIN1 = 5;

// --- 左後輪 (LR) ---
const int LR_BIN1 = 10;
const int LR_BIN2 = 11;
const int LR_PWMB = 12;

// --- 右前輪 (RF) ---
const int RF_PWMB = 23;
const int RF_BIN2 = 19;
const int RF_BIN1 = 18;

// --- 右後輪 (RR) ---
const int RR_PWMA = 13;
const int RR_AIN1 = 15;
const int RR_AIN2 = 14;

// 通用馬達驅動函式
void driveMotor(int pinPWM, int pinIN1, int pinIN2, int pwm) {
  pwm = constrain(pwm, -255, 255);
  
  if (pwm > 0) {
    digitalWrite(pinIN1, HIGH); 
    digitalWrite(pinIN2, LOW);
    analogWrite(pinPWM, pwm);
  } else if (pwm < 0) {
    digitalWrite(pinIN1, LOW);  
    digitalWrite(pinIN2, HIGH);
    analogWrite(pinPWM, -pwm);
  } else {
    // 靜止狀態：完全釋放馬達 (LOW/LOW)
    digitalWrite(pinIN1, LOW);  
    digitalWrite(pinIN2, LOW);
    analogWrite(pinPWM, 0);
  }
}

void setup() {
  Serial.begin(115200);

  // 初始化 Servo 舵機 (MG996R)
  mg996Servo.attach(SERVO_PIN, 544, 2400);
  mg996Servo.write(90); // 預設靜止

  // 初始化 TB6612 腳位
  pinMode(STBY, OUTPUT); 
  digitalWrite(STBY, HIGH); // 解除待機模式

  pinMode(LF_PWMA, OUTPUT); pinMode(LF_AIN1, OUTPUT); pinMode(LF_AIN2, OUTPUT);
  pinMode(LR_PWMB, OUTPUT); pinMode(LR_BIN1, OUTPUT); pinMode(LR_BIN2, OUTPUT);
  pinMode(RF_PWMB, OUTPUT); pinMode(RF_BIN1, OUTPUT); pinMode(RF_BIN2, OUTPUT);
  pinMode(RR_PWMA, OUTPUT); pinMode(RR_AIN1, OUTPUT); pinMode(RR_AIN2, OUTPUT);

  // 設置高頻 PWM (20kHz) 消除馬達低頻蜂鳴聲
  analogWriteFrequency(LF_PWMA, 20000); analogWriteFrequency(LR_PWMB, 20000);
  analogWriteFrequency(RF_PWMB, 20000); analogWriteFrequency(RR_PWMA, 20000);

  // 啟動 PPM 接收器 (Pin 9)
  ReceiverInput.begin(PPM_PIN);
}

void loop() {
  // 1. 讀取 PPM 通道訊號
  int channelCount = ReceiverInput.available();
  if (channelCount > 0) {
    for (int i = 1; i <= min(channelCount, 10); i++) {
      ReceiverValue[i - 1] = ReceiverInput.read(i);
    }
  }

  float ch1_steering = ReceiverValue[0]; // 通道 1: 右搖桿左右 (轉向)
  float ch2_throttle = ReceiverValue[1]; // 通道 2: 右搖桿前後 (油門)
  float ch7_servo    = ReceiverValue[6]; // 通道 7: 控制 996 馬達

  // 防護機制：訊號遺失時歸零
  if (ch1_steering < 500) ch1_steering = 1500;
  if (ch2_throttle < 500) ch2_throttle = 1500; 

  // ==========================================
  // 2. MG996R 控制邏輯 (CH7 / SwC 開關)
  // ==========================================
  if (ch7_servo > 1700) {
    mg996Servo.write(180); // 開關下撥：正轉
  } else {
    mg996Servo.write(90);  // 靜止停止
  }

  // ==========================================
  // 3. 計算油門 (Throttle) 與 轉向 (Steering) PWM
  // ==========================================
  int throttlePWM = 0;
  int steeringPWM = 0;

  // --- 油門 (CH2) 1498 ~ 1501 死區過濾 ---
  if (ch2_throttle > 1501) {
    throttlePWM = map(ch2_throttle, 1502, 2000, 0, 255);
  } else if (ch2_throttle < 1498) {
    throttlePWM = map(ch2_throttle, 1497, 1000, 0, -255);
  }

  // --- 轉向 (CH1) 1498 ~ 1501 死區過濾 ---
  if (ch1_steering > 1501) {
    steeringPWM = map(ch1_steering, 1502, 2000, 0, 255);
  } else if (ch1_steering < 1498) {
    steeringPWM = map(ch1_steering, 1497, 1000, 0, -255);
  }

  // ==========================================
  // 4. 左右差速混控計算 (Differential Steering)
  // ==========================================
  int leftBasePWM  = throttlePWM + steeringPWM; // 左側基礎 PWM
  int rightBasePWM = throttlePWM - steeringPWM; // 右側基礎 PWM

  // 限制 PWM 範圍在 -255 ~ 255
  leftBasePWM  = constrain(leftBasePWM, -255, 255);
  rightBasePWM = constrain(rightBasePWM, -255, 255);

  // ==========================================
  // 5. 驅動 4 顆馬達 (包含後輪反相修正)
  // ==========================================
  // 左側兩輪 (左前正向，左後反相)
  driveMotor(LF_PWMA, LF_AIN1, LF_AIN2, leftBasePWM);
  driveMotor(LR_PWMB, LR_BIN1, LR_BIN2, -leftBasePWM);

  // 右側兩輪 (右前正向，右後反相)
  driveMotor(RF_PWMB, RF_BIN1, RF_BIN2, rightBasePWM);
  driveMotor(RR_PWMA, RR_AIN1, RR_AIN2, -rightBasePWM);

  // 除錯訊息
  Serial.print("CH1_Steer:"); Serial.print(ch1_steering);
  Serial.print("\tCH2_Throt:"); Serial.print(ch2_throttle);
  Serial.print("\tLeftPWM:"); Serial.print(leftBasePWM);
  Serial.print("\tRightPWM:"); Serial.println(rightBasePWM);

  delay(20);
}