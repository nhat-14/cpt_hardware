#include <Servo.h>   // Library for controling servo motor

int servo_pin = 9; 
Servo motor; 
float realease_rate = 0.4;  // 0.4Hz ~ 2.5s per release


void setup() { 
   // Attac servo to used pin number 
   motor.attach(servo_pin); 
}


void loop() { 
   motor.write(80);  // Make servo go to 90 degrees 
   delay(100); 
   motor.write(170); 
   delay(int(1/realease_rate)); // wait until next release
}