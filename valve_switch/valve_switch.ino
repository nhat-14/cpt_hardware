/**
* @file valve_switch.ino
*
* @brief Program to open and close the solenoid valve at regular intervals.
* Triggered to start cyclic operation of solenoid valve.
* Serial command: 'g' for ON, 'q' for OFF
*
* @author Duc-Nhat Luong （ニャット）
* Contact: luong.d.aa@m.titech.ac.jp
* 
* @copyright Copyright 2021, The Chemical Plume Tracing (CPT) Robot Project"
* credits ["Luong Duc Nhat"]
* license GPL
*/


const int ledPin = 9;
const int valvePin = 8;

int period = 1000;		// 1000ms ~ 1Hz
int pulse_width = 200; 	//ms
bool releaseFlag = false;

void setup() {
  	pinMode(valvePin, OUTPUT); 	//HIGH : Solenoid Valve is open 
  	pinMode(ledPin, OUTPUT); 	//HIGH : LED bright
	digitalWrite(valvePin, LOW);
    digitalWrite(ledPin, LOW);
  	
	Serial.begin(115200);
  	Serial.println("Program Started");
}

void loop() {
 	if (Serial.available() > 0) {
		char cmd = Serial.read();

		if(cmd == 'g'){
			Serial.print("Running");
			releaseFlag = true;
		}
		else if (cmd == 'q') {
			Serial.println("Stopped");
			releaseFlag = false;
		}
		else {
			Serial.println("Wrong command");
		}
  	}
  
	if (releaseFlag == true) {
		digitalWrite(valvePin, HIGH);
		digitalWrite(9, HIGH);
		delay(pulse_width);

		digitalWrite(valvePin, LOW);
		digitalWrite(ledPin, LOW);
		delay(period-pulse_width);
	}
	else {
		digitalWrite(valvePin, LOW);
		digitalWrite(ledPin, LOW);
		delay(period);
	}
}