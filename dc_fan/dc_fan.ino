/**
* @file dc_fan.ino
*
* @brief Varies DC fan speed and direction for recording different shape 
* of smoke for PIV experiment.
*
* @author Duc-Nhat Luong （ニャット）
* Contact: luong.d.aa@m.titech.ac.jp
* 
* @copyright Copyright 2022, The Chemical Plume Tracing (CPT) Robot Project"
* credits ["Luong Duc Nhat"]
* license GPL
*/

#include <Gaussian.h>
#include <Servo.h>

#define fanPin_1 7      //Control fan direction pin
#define fanPin_2 8      //Control fan direction pin
#define fanSpeedPin 12  //Control fan input pin
#define servoPin 9      //Control servo motor pin

int timer1_counter;
double mean_voltage = 10;
double var = 2;         //variance = 0 means constant speed

int pos = 0;                // servo position
const int min_angle = 90;   // minimum angle the fan can roate to
const int max_angle = 160;  // maximum angle the fan can roate to

//Generate random voltage for DC fan
Gaussian myGaussian = Gaussian(mean_voltage, var); 
Servo myservo;

void stop() {
    analogWrite(fanSpeedPin,0);
}


ISR(TIMER1_OVF_vect) {        // interrupt service routine 
    TCNT1 = timer1_counter;   // preload timer
    int pmw = 255;
    digitalWrite(fanPin_1,HIGH);
    digitalWrite(fanPin_2,LOW);
    analogWrite(fanSpeedPin,pmw);
}


void setup() {
    // initialize timer1 
    noInterrupts();           // disable all interrupts
    TCCR1A = 0;
    TCCR1B = 0;
    
    timer1_counter = 34286;   // preload timer 65536-16MHz/256/2Hz
    TCNT1 = timer1_counter;   // preload timer
    TCCR1B |= (1 << CS12);    // 256 prescaler 
    TIMSK1 |= (1 << TOIE1);   // enable timer overflow interrupt
    interrupts();             // enable all interrupts
    
    myservo.attach(servoPin);  // attaches the servo on pin 9 to the servo object
    pinMode(fanPin_1, OUTPUT);
    pinMode(fanPin_2, OUTPUT); 
    pinMode(fanSpeedPin, OUTPUT);
    
    stop();
    Serial.begin(9600);
}

void loop(){
    // goes from min degrees to max degrees and reverse
    for (pos = min_angle; pos <= max_angle; pos += 1) { 
        // in steps of 1 degree
        myservo.write(pos);   // tell servo to go to position in variable 'pos'
        delay(20);            // waits for the servo to reach the position
    }

    for (pos = max_angle; pos >= min_angle; pos -= 1) { 
        myservo.write(pos);              
        delay(20);                
    }
}