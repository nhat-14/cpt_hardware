#include "turtlebot3_conveyor.h"
#include "turtlebot3_conveyor_motor_driver.h"

#define LOOP_TIME_SEC 0.010f

DynamixelStatus conveyor;
Turtlebot3MotorDriver motor_driver;

uint8_t conveyor_joint[4] = {JOINT_L_R, JOINT_R_R, JOINT_L_F, JOINT_R_F};
uint8_t conveyor_wheel[4] = {WHEEL_L_R, WHEEL_R_R, WHEEL_L_F, WHEEL_R_F};
int test = 14;
void setup() {
  Serial.begin(57600);
  motor_driver.init();
  
}

void loop() {
  static uint32_t previous_time = 0;
  uint32_t present_time = millis();
//  getRC100Data();
//
//  if((present_time - previous_time) >= (LOOP_TIME_SEC * 1000)) {
//    motor_driver.controlJoints(conveyor.setJointAngle());
//    motor_driver.controlWheels(conveyor.setWheelVel());
//
//    previous_time = millis();
//  } 
}

void getRC100Data() {   
    int rcData = 1;
    conveyor.getDirection(rcData);
    delay(1);
    conveyor.setParams();
}
