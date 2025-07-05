//https://55life555.blog.fc2.com/blog-entry-3194.html
//https://qiita.com/suruseas/items/14ae3c7514775f0b1fc4
//https://github.com/akshaybaweja/SoftwareSerial
//wifi使うと2, 4, 12, 14, 15, 26, 27が使えなくなる
//#include <Wire.h>
#include <math.h>
//#include <SPI.h>
//#include <SoftwareSerial.h>

const int RX_PIN = 22;
const int TX_PIN = 21;

volatile int timeCounter1;
hw_timer_t *timer1 = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

char dat[30];   // 格納用文字列
int count = 0;  // 文字数のカウンタ
char v1[10];
int winddir=0;

uint16_t Gsens=0;

String myString;

void IRAM_ATTR onTimer1(){
  portENTER_CRITICAL_ISR(&timerMux);
  timeCounter1++;
  portEXIT_CRITICAL_ISR(&timerMux);
}


void setup() {
  Serial.begin(38400);
  Serial1.begin(38400,SERIAL_8N1, RX_PIN, TX_PIN);

  timer1 = timerBegin(0, 80, true);
  timerAttachInterrupt(timer1, &onTimer1, true);
  timerAlarmWrite(timer1, 1000000, true);
  timerAlarmEnable(timer1);
}

void loop() {  
  if (timeCounter1 > 0) {
    portENTER_CRITICAL(&timerMux);
    timeCounter1--;
    portEXIT_CRITICAL(&timerMux);
    RoutineWork();
  }
}

//1秒間隔の割り込み
void RoutineWork() {
    Serial1.print("<RD,M05,>??");
    Serial1.print("\r\n");
    while (Serial1.available()!=0) {
      dat[count] = Serial1.read();
        if (dat[count] == '\n') {  // 文字数が既定の個数を超えた場合、又は終了文字を受信した場合
          dat[count+1] = '\0';     // 末尾に終端文字を入れる
          count = 0;               // 文字カウンタをリセット
          break;
        }
        else {
          count++;                              // 文字カウンタに 1 加算
        }
    }
    //Convert received character data to numeric data
    sscanf(dat, "<AD,M05=%d,>%s\r\n", &winddir,v1);
    Serial.println(winddir);
}