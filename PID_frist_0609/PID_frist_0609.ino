#include <PulsePosition.h>
#include <Encoder.h>

// --- PPM 接收器與馬達腳位 (維持不變) ---
PulsePositionInput ReceiverInput(RISING); 
float ReceiverValue[] = {0,0,0,0,0,0,0,0};  

// --- TB6612FNG 馬達腳位定義 (對應前述 Teensy 4.1 接線) ---
const int STBY = 9; // 剛好是板載 LED，接 TB6612 的 STBY

// 馬達 A (左馬達)
const int PWMA = 4;
const int AIN2 = 5;
const int AIN1 = 6;

// 馬達 B (右馬達)
const int PWMB = 12;
const int BIN1 = 10;
const int BIN2 = 11;

Encoder motorAEncoder(2, 3); 
Encoder motorBEncoder(7, 8); 

unsigned long previousMillis = 0;
const long interval = 20; // 建議將控制週期縮短至 20ms (50Hz)，對 PID 的穩定度更好

long oldPosA = -999; long oldPosB = -999;
const float GEAR_RATIO = 30.0; 
const float ENCODER_PPR = 11.0 * 4.0 * GEAR_RATIO; 

// ==========================================
// 核心：PID 參數與變數宣告
// ==========================================
// 調整這三個參數（調參），可以改變馬達的跟隨表現
float Kp = 1.2;  
float Ki = 0.5;  
float Kd = 0.05; 

// 左輪 (A) PID 內部變數
float errorA = 0, lastErrorA = 0, integralA = 0, derivativeA = 0;
float outputPWM_A = 0;

// 右輪 (B) PID 內部變數
float errorB = 0, lastErrorB = 0, integralB = 0, derivativeB = 0;
float outputPWM_B = 0;

// 馬達最大設計轉速 (根據您的馬達規格調整，假設 12V 下空載為 160 RPM)
const float MAX_RPM = 160.0; 

void setup() {
  Serial.begin(115200); 
  pinMode(STBY, OUTPUT); digitalWrite(STBY, HIGH);
  pinMode(PWMA, OUTPUT); pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT);
  pinMode(PWMB, OUTPUT); pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT);
  analogWriteFrequency(PWMA, 20000); analogWriteFrequency(PWMB, 20000);
  ReceiverInput.begin(14);
}

void loop() {
  // 1. 讀取遙控器
  int channlNumber = ReceiverInput.available();
  if (channlNumber > 0) {
    for(int i = 1; i <= channlNumber; i++) ReceiverValue[i-1] = ReceiverInput.read(i);
  }
  float throttleSignal = ReceiverValue[2]; 
  if (throttleSignal < 500) throttleSignal = 1500; 

  // ==========================================
  // 2. 定時 PID 計算與馬達驅動 (每 20ms 執行一次)
  // ==========================================
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    
    // A. 計算當前兩輪實際轉速 (RPM)
    long posA = motorAEncoder.read(); long posB = motorBEncoder.read();
    float rpmA = ((float)(posA - oldPosA) / ENCODER_PPR) * (1000.0 / interval) * 60.0;
    float rpmB = ((float)(posB - oldPosB) / ENCODER_PPR) * (1000.0 / interval) * 60.0;
    oldPosA = posA; oldPosB = posB;

    // B. 定義兩輪目標轉速 (Target RPM)
    float targetRPM_A = 0;
    float targetRPM_B = 0;

    if (throttleSignal > 1550) { 
      // 順時針原地旋轉：左輪正轉、右輪反轉
      float targetSpeed = map(throttleSignal, 1550, 2000, 0, MAX_RPM);
      targetRPM_A = targetSpeed;
      targetRPM_B = -targetSpeed; // 右輪目標是負轉速
    } 
    else if (throttleSignal < 1450) {
      // 逆時針原地旋轉：左輪反轉、右輪正轉
      float targetSpeed = map(throttleSignal, 1450, 1000, 0, MAX_RPM);
      targetRPM_A = -targetSpeed;
      targetRPM_B = targetSpeed;
    }

    // C. 左輪 (A) PID 運算
    errorA = targetRPM_A - rpmA;                  // 1. 計算誤差
    integralA += errorA * ((float)interval/1000.0); // 2. 累積積分
    integralA = constrain(integralA, -100, 100);   // 抗積分飽和 (Anti-windup)
    derivativeA = (errorA - lastErrorA) / ((float)interval/1000.0); // 3. 計算微分
    
    outputPWM_A += (Kp * errorA) + (Ki * integralA) + (Kd * derivativeA); // PID 總輸出
    outputPWM_A = constrain(outputPWM_A, -255, 255); // 限制在 Teensy PWM 範圍
    lastErrorA = errorA;                           // 儲存本次誤差

    // D. 右輪 (B) PID 運算
    errorB = targetRPM_B - rpmB;
    integralB += errorB * ((float)interval/1000.0);
    integralB = constrain(integralB, -100, 100);
    derivativeB = (errorB - lastErrorB) / ((float)interval/1000.0);
    
    outputPWM_B += (Kp * errorB) + (Ki * integralB) + (Kd * derivativeB);
    outputPWM_B = constrain(outputPWM_B, -255, 255);
    lastErrorB = errorB;

    // E. 將控制結果輸出給 TB6612FNG 硬體
    driveMotorA(outputPWM_A);
    driveMotorB(outputPWM_B);

    // F. 列印除錯數據，方便用 Serial Plotter (繪圖器) 觀察曲線
    Serial.print("TargetA:"); Serial.print(targetRPM_A);
    Serial.print(",ActualA:"); Serial.print(rpmA);
    Serial.print(",TargetB:"); Serial.print(targetRPM_B);
    Serial.print(",ActualB:"); Serial.println(rpmB);

    previousMillis = currentMillis;
  }
}

// 馬達 A 驅動子程式 (支援正負輸入)
void driveMotorA(float pwm) {
  if (pwm > 0) {
    digitalWrite(AIN1, HIGH); digitalWrite(AIN2, LOW);
    analogWrite(PWMA, (int)pwm);
  } else if (pwm < 0) {
    digitalWrite(AIN1, LOW);  digitalWrite(AIN2, HIGH);
    analogWrite(PWMA, (int)(-pwm));
  } else {
    digitalWrite(AIN1, HIGH); digitalWrite(AIN2, HIGH); // 煞車
    analogWrite(PWMA, 0);
  }
}

// 馬達 B 驅動子程式 (支援正負輸入)
void driveMotorB(float pwm) {
  if (pwm > 0) {
    digitalWrite(BIN1, HIGH); digitalWrite(BIN2, LOW);
    analogWrite(PWMB, (int)pwm);
  } else if (pwm < 0) {
    digitalWrite(BIN1, LOW);  digitalWrite(BIN2, HIGH);
    analogWrite(PWMB, (int)(-pwm));
  } else {
    digitalWrite(BIN1, HIGH); digitalWrite(BIN2, HIGH); // 煞車
    analogWrite(PWMB, 0);
  }
}