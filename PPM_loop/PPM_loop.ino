#include <PulsePosition.h>

const int PPM_PIN = 9; // PPM可以接在6、9、10、14
PulsePositionInput ReceiverInput(RISING);

float channels[10] = {0}; // 儲存 10 個通道數值

unsigned long lastPrintTime = 0;
const unsigned long printInterval = 50; // 縮短為 50ms

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000); // 等待序列埠連線
  
  ReceiverInput.begin(PPM_PIN);
}

void loop() {
  // 一次性讀取 10 個通道
  for (int i = 1; i <= 10; i++) {
    channels[i - 1] = ReceiverInput.read(i);
  }

  // 定時輸出波形數據
  if (millis() - lastPrintTime >= printInterval) {
    lastPrintTime = millis();

    // 輸出符合 Arduino 序列繪圖器格式的字串 (以 Tab 分隔)
    for (int i = 0; i < 10; i++) {
      Serial.print("CH");
      Serial.print(i + 1);
      Serial.print(":");
      Serial.print((int)channels[i]);
      
      // 除了最後一個通道外，其餘用 Tab 隔開
      if (i < 9) {
        Serial.print("\t");
      }
    }
    Serial.println();
  }
}