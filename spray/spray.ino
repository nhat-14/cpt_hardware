//------------------------------------------------------------------
// This code is used to automated the ethanol spay action using relay module
// and electric spray: https://www.monotaro.com/g/03004290/
//
// This code is written and maintained by Luong Duc Nhat
//
// Japan, Tokyo Institute of Technology, 2023 June.
//------------------------------------------------------------------

// These constants won't change. They're used to give names to the pins used:
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
