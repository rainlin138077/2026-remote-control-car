#include <PulsePosition.h>

const int PWM_PIN = 9; // PWM訊號能接在大部分的Teensy腳位上
PulsePositionInput ReceiverInput(RISING);

unsigned long lastPrintTime = 0;
const unsigned long printInterval = 100; // 每 100ms印一次

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000); // 等待序列埠連線
  
  ReceiverInput.begin(PWM_PIN);
  Serial.println("單通道 PWM 監測程式已啟動");
}

void loop() {
  // 直接讀取指定的單一通道（括號內的數字代表通道編號）
  float targetChannelValue = ReceiverInput.read(1); 

  // 控制序列埠印出頻率，避免高頻傳輸造成卡頓
  if (millis() - lastPrintTime >= printInterval) {
    lastPrintTime = millis();

    Serial.print("CH1 數值: ");
    Serial.print((int)targetChannelValue);
    Serial.println(" us");
  }
}
