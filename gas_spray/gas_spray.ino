/**
* @file gas_spray.ino
*
* @brief This code is used to automated the ethanol spay action using relay module
* and electric spray: https://www.monotaro.com/g/03004290/
*
* @author Duc-Nhat Luong （ニャット）
* Contact: luong.d.aa@m.titech.ac.jp
* 
* @copyright Copyright 2022, The Chemical Plume Tracing (CPT) Robot Project"
* credits ["Luong Duc Nhat"]
* license GPL
*/

const int sprayPin = 11;
const float frequencyHz = 0.5; 
const float release_rate = 0.4;

void setup() {
    pinMode(sprayPin, OUTPUT);
}

void loop() {
    digitalWrite(sprayPin, HIGH); 
    delay(1000*release_rate/frequencyHz);
    digitalWrite(sprayPin, LOW); 
    delay(1000*(1-release_rate)/frequencyHz); 
}