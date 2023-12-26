/**
* @file co2_humid.ino
*
* @brief This code demonstrate communication between Arduino and 
* C02-Temperature-Humid sensor SCD30 using software serial. 
*
* @author Duc-Nhat Luong （ニャット）
* Contact: luong.d.aa@m.titech.ac.jp
* 
* @copyright Copyright 2021, The Chemical Plume Tracing (CPT) Robot Project"
* credits ["Luong Duc Nhat"]
* license GPL
*/

#include "SCD30.h"

#if defined(ARDUINO_ARCH_AVR)
    #pragma message("Defined architecture for ARDUINO_ARCH_AVR.")
    #define SERIAL Serial
#elif defined(ARDUINO_ARCH_SAM)
    #pragma message("Defined architecture for ARDUINO_ARCH_SAM.")
    #define SERIAL SerialUSB
#elif defined(ARDUINO_ARCH_SAMD)
    #pragma message("Defined architecture for ARDUINO_ARCH_SAMD.")
    #define SERIAL SerialUSB
#elif defined(ARDUINO_ARCH_STM32F4)
    #pragma message("Defined architecture for ARDUINO_ARCH_STM32F4.")
    #define SERIAL SerialUSB
#else
    #pragma message("Not found any architecture.")
    #define SERIAL Serial
#endif


void setup() {
    Wire.begin();
    SERIAL.begin(115200);
    scd30.initialize();
    delay(2000);
}


void loop() {
    float result[3] = {0};
    delay(100);
    if (scd30.isAvailable()) {
        scd30.getCarbonDioxideConcentration(result);
        Serial.println(result[0]);
    }
}
