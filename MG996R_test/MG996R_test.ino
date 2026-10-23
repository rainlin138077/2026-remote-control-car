#include <PulsePosition.h>
#include <Servo.h>

// 宣告 PPM 接收器物件 (使用 Teensy 的解碼硬體)
PulsePositionInput ppmReceiver; 
Servo mg996Servo; // 宣告 996 馬達物件

const int PPM_PIN = 14;      // 假設你的 FS-iA6B PPM 訊號線接在 Teensy 的 Pin 14
const int SERVO_PIN = 35;    // 假設你的 MG996R 訊號線（橘/白）接在 Teensy 的 Pin 35

void setup() {
  Serial.begin(9600);
  ppmReceiver.begin(PPM_PIN); // 開始監聽 PPM 腳位
  mg996Servo.attach(SERVO_PIN); // 將馬達物件綁定到 Pin 9
}

void loop() {
  // 檢查是否有收到合法的 PPM 訊號區段
  int availableChannels = ppmReceiver.available();
  
  if (availableChannels >= 7) { // 確保至少有讀到 7 個通道以上
    // 讀取第 7 通道的微秒數 (PWM 訊號範圍通常在 1000 到 2000 之間)
    int ch7Value = ppmReceiver.read(7); 
    
    Serial.print("CH7 Value: ");
    Serial.println(ch7Value);

    // 判斷開關是否「下轉/下撥」
    // 通常開關往下撥時，數值會接近 2000 微秒（有些遙控器相反則是 1000，可看序列通訊埠確認）
    if (ch7Value > 1700) {
      // 【一直正轉】
      // 對於 360度伺服馬達來說，write(180) 代表全速正轉，write(0) 代表全速反轉，write(90) 是停止。
      mg996Servo.write(180); 
    } 
    else {
      // 【停止旋轉】
      // 當開關往上撥或在中間時，讓馬達靜止
      mg996Servo.write(90); 
    }
  }
  
  delay(20); // 稍微小睡一下，配合同步率
}