// --- 霍爾傳感器腳位定義 ---
const int encoderPinB = 2; // C2 黄
const int encoderPinA = 3; // C1 綠

// 儲存上一次的狀態，用來比對有沒有變化
int lastStateA = -1;
int lastStateB = -1;

void setup() {
  Serial.begin(250000); // 維持高速通訊
  
  pinMode(encoderPinA, INPUT_PULLUP);
  pinMode(encoderPinB, INPUT_PULLUP);
}

void loop() {
  // 1. 讀取當前狀態
  int stateA = digitalRead(encoderPinA);
  int stateB = digitalRead(encoderPinB);

  // 2. 關鍵判定：只有當 A 狀態 或 B 狀態跟上一次不一樣時，才執行列印
  if (stateA != lastStateA || stateB != lastStateB) {
    
    // 輸出 A 相 (0 或 1)
    Serial.print("Phase_A_C1:");
    Serial.print(stateA);
    Serial.print(",");
    
    // 輸出 B 相 (加上偏移量 2，變成 2 或 3，方便分開觀察)
    Serial.print("Phase_B_C2_Offset:");
    Serial.println(stateB + 2);

    // 3. 更新歷史紀錄
    lastStateA = stateA;
    lastStateB = stateB;
  }
  
  // 這裡依然不加 delay，但因為有了上面的 if 判斷，
  // 圖形跑的速度現在完全由你「手轉的速度」來決定了！
}