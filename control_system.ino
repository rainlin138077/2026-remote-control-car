#include <PulsePosition.h>
#include <Encoder.h> // 匯入 Teensy 高效率硬體中斷編碼器函式庫

// ==========================================
// 1. PPM 接收器設定 (FS-i6X)
// ==========================================
PulsePositionInput ReceiverInput(RISING); 
float ReceiverValue[10] = {0};

// ==========================================
// 2. TB6612FNG 馬達驅動板腳位定義
// ==========================================
const int STBY = 9; // 接 STBY，同時也是 Teensy 板載 LED

// 馬達 A (左馬達) 控制引腳
const int PWMA = 4;
const int AIN1 = 6;
const int AIN2 = 5;

// 馬達 B (右馬達) 控制引腳
const int PWMB = 10;
const int BIN1 = 11;
const int BIN2 = 12;

// ==========================================
// 3. JGB37-520 霍爾編碼器腳位與設定
// ==========================================
// 馬達 A (左)：C2(黃)接 Pin 2, C1(綠)接 Pin 3
Encoder motorAEncoder(1, 0); 

// 馬達 B (右)：C2(黃)接 Pin 7, C1(綠)接 Pin 8
Encoder motorBEncoder(8, 7); 

// 速度計算時間與歷史紀錄變數
unsigned long previousMillis = 0;
const long interval = 50; // 每 50 毫秒 (Hz = 20) 計算並印出一次速度

long oldPosA = -999;
long oldPosB = -999;

// 馬達物理規格參數 (請根據您購買的馬達實際減速比修改)
const float GEAR_RATIO = 30.0; // 假設減速比為 1:30 (若是 1:90 請改 90.0)
const float ENCODER_PPR = 11.0 * 4.0 * GEAR_RATIO; // 輸出軸轉一圈的總計數脈衝 (1320 Ticks)

void setup() {
  // 開啟高速序列埠通訊，方便即時觀看數據
  Serial.begin(115200); 
  
  // 初始化 TB6612FNG 的啟動腳位
  pinMode(STBY, OUTPUT);
  digitalWrite(STBY, HIGH);
  
  // 初始化馬達 A、B 的控制引腳
  pinMode(PWMA, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);

  // 將 Teensy 的馬達控制 PWM 頻率提升至 20kHz
  analogWriteFrequency(PWMA, 20000);
  analogWriteFrequency(PWMB, 20000);

  // 初始化 PPM 接收腳位 (Pin 28)
  ReceiverInput.begin(28);
}

void loop() {
  // ==========================================
  // 核心功能一：讀取遙控器訊號
  // ==========================================
  int channlNumber = ReceiverInput.available();
  if (channlNumber > 0) {
    for(int i = 1; i <= channlNumber; i++) {
      ReceiverValue[i-1] = ReceiverInput.read(i);
    }
  }

  // 根據 FS-i6X 模型 1 (飛機模式)，CH3 (ReceiverValue[2]) 為右搖桿上下油門
  float throttleSignal = ReceiverValue[2]; 

  // 安全防護：若剛開機沒收到遙控器訊號，將數值鎖定在中點 1500us
  if (throttleSignal < 500) throttleSignal = 1500; 

  int motorSpeed = 0;

  // ==========================================
  // 核心功能二：遙控旋轉邏輯控制
  // ==========================================
  if (throttleSignal > 1550) { 
    // 【右搖桿往前推】 -> 原地順時針旋轉 (左輪前進、右輪後退)
    motorSpeed = map(throttleSignal, 1550, 2000, 0, 255);
    motorSpeed = constrain(motorSpeed, 0, 255);

    // 左馬達 (A) 前進
    digitalWrite(AIN1, HIGH);   digitalWrite(AIN2, LOW);
    analogWrite(PWMA, motorSpeed);

    // 右馬達 (B) 後退
    digitalWrite(BIN1, LOW);    digitalWrite(BIN2, HIGH);
    analogWrite(PWMB, motorSpeed);

  } 
  else if (throttleSignal < 1450) {
    // 【右搖桿往後拉】 -> 原地逆時針旋轉 (左輪後退、右輪前進)
    motorSpeed = map(throttleSignal, 1450, 1000, 0, 255);
    motorSpeed = constrain(motorSpeed, 0, 255);

    // 左馬達 (A) 後退
    digitalWrite(AIN1, LOW);    digitalWrite(AIN2, HIGH);
    analogWrite(PWMA, motorSpeed);

    // 右馬達 (B) 前進
    digitalWrite(BIN1, HIGH);   digitalWrite(BIN2, LOW);
    analogWrite(PWMB, motorSpeed);

  } 
  else {
    // 【搖桿放開回中】 -> 兩輪急速電子煞車停止
    digitalWrite(AIN1, HIGH);   digitalWrite(AIN2, HIGH);
    analogWrite(PWMA, 0);

    digitalWrite(BIN1, HIGH);   digitalWrite(BIN2, HIGH);
    analogWrite(PWMB, 0);
  }

  // ==========================================
  // 核心功能三：定時計算雙馬達即時轉速 (RPM)
  // ==========================================
  unsigned long currentMillis = millis();
  
  if (currentMillis - previousMillis >= interval) {
    // 讀取當前編碼器的累積總脈衝計數
    long posA = motorAEncoder.read();
    long posB = motorBEncoder.read();
    
    // 計算自上一次（50ms前）到現在的變化量
    long deltaA = posA - oldPosA; 
    long deltaB = posB - oldPosB; 
    
    // 換算為每分鐘轉速 (RPM)
    float rpmA = ((float)deltaA / ENCODER_PPR) * (1000.0 / interval) * 60.0;
    float rpmB = ((float)deltaB / ENCODER_PPR) * (1000.0 / interval) * 60.0;

    // 將各項數據即時輸出至序列埠監控器
    Serial.print("訊號量: ");   Serial.print(throttleSignal);
    Serial.print(" | 左輪計數: "); Serial.print(posA);
    Serial.print(" -> 左轉速 (RPM): "); Serial.print(rpmA, 1);
    Serial.print(" | 右輪計數: "); Serial.print(posB);
    Serial.print(" -> 右轉速 (RPM): "); Serial.println(rpmB, 1);

    // 更新歷史紀錄，供下一個 50ms 週期計算
    oldPosA = posA;
    oldPosB = posB;
    previousMillis = currentMillis;
  }
}