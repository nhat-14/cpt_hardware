#include <WindSensorHWD_asukiaaa.hpp>
#define SERIAL_FOR_WINDSENSOR Serial2

WindSensorHWD_asukiaaa::Core windSensor(&SERIAL_FOR_WINDSENSOR);

void setup() {
  Serial.begin(115200);
  windSensor.beginWithSerial();
}

void loop() {
  auto infoSensor = windSensor.readInfoSensor();
  if (infoSensor.readResult == 0) {
    // infoSensor.printToStream(&Serial);
    String dir = angleToDirection(infoSensor.windDirection);
    Serial.println(dir);
    Serial.println(infoSensor.windDirection);
  } 
  else {
    Serial.print("Error infoSensor.readResult: ");
    Serial.println(String(infoSensor.readResult));
  }

  // Serial.print("at ");
  Serial.println("===============================");
  delay(500);
}

String angleToDirection(int angle) {
  // Clamp the angle to the range [-180, 0]
  // if (angle > 0) angle = 0;
  // if (angle < -180) angle = -180;

  if (angle >= -45 && angle <= 45) {
    return "front";
  } else if (angle >= 45 && angle < 135) {
    return "right";
  } else if (angle >= -135 && angle < -45) {
    return "left";
  } else { // angle >= -180 && angle < -135
    return "back";
  }
}