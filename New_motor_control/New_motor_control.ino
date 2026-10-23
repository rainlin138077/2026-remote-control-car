#include <PulsePosition.h>
PulsePositionInput ReceiverInput(RISING); //接收器會以此為名稱並傳輸此名稱的訊號
//declare the variables to store channel info
float ReceiverValue[]={0,0,0,0,0,0,0,0,0,0};  //可以存處10個通道值
int ChannlNumber=0;
float InputThrottle; //馬達油門變數。0%=1000 μs ， 100%= 2000 μs


void read_receiver(void){                 //define a funtion to read the receiver data
  ChannlNumber = ReceiverInput.available(); //available() 檢查設備接收緩衝區內是否有等待讀取的資料，並回傳剩餘的未讀取位元組數
  if(ChannlNumber>0){
    for(int i=1;i<=ChannlNumber;i++){
      ReceiverValue[i-1]=ReceiverInput.read(i);
    }
  }
}
void setup() {
  Serial.begin(115200);
  pinMode(13,OUTPUT);
  digitalWrite(13, HIGH);
  ReceiverInput.begin(28);
  analogWriteFrequency(1, 250); //send PWM signals form your teensy
  analogWriteResolution(12);
  delay(250);
  while (ReceiverValue[2]<1020 || ReceiverValue[2]>1050) { //依據之前測的數值做數值估計
    read_receiver();
    delay(50);
  }
}

void loop() { //擷取數值提供PWM給馬達
  read_receiver();
  InputThrottle=ReceiverValue[2];
  analogWrite(1,1.024*InputThrottle);
}
