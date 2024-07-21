#include <MsTimer2.h>

int dt = 50;
long timeCount = 0;
long pulseWidth = 100;
long interval = 900;
int pulseNum = 60;
bool valveState = false;
int inputchar;

void setup() {
	pinMode(8, OUTPUT); 
	pinMode(9, OUTPUT);
	closeValve();
	Serial.begin(38400);
	MsTimer2::set(dt, timerInterrupt);
}

void loop() {
	if(Serial.available()) {
		inputchar = Serial.read();
		switch(inputchar){
			case 'c':
				closeValve();
				pulseNum = 1;
				timeCount = 0;
				MsTimer2::start();
				Serial.print("Cal\n");
				break;
			case 'r':
				closeValve();
				pulseNum = 200000;
				pulseWidth = 100;
				interval = 900;
				MsTimer2::stop();
				delay(5000);
				timeCount = 0;
				MsTimer2::start();
				Serial.print("Reset\n");
				break;
		}
	}
}

void timerInterrupt() {
	timeCount += dt;
	bool isTimeOut = checkTimeOut();
	if (isTimeOut == false) {
		// Check what is the state in duty cycles
		if(timeCount%(pulseWidth+interval) < pulseWidth)
			openValve();
		else
			closeValve();
		Serial.println(valveState);
	}
}

void openValve() {
	valveState = true;
	digitalWrite(8, HIGH);
	digitalWrite(9, HIGH);
}

void closeValve() {
    valveState = false;
    digitalWrite(8, LOW);
    digitalWrite(9, LOW);
}

bool checkTimeOut() {
	if(timeCount >= (pulseWidth+interval)*pulseNum){
		MsTimer2::stop();
		closeValve();
		return true;
	}
	return false;
}