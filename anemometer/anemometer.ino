/**
* @file valve_switch.ino
*
* @brief This code demonstrate communication between Arduino and Wind sensor 
* using software serial. Wind data is read and published to ROS 1 network
* using olfaction_msgs::anemometer type
*
* @author Duc-Nhat Luong （ニャット）
* Contact: luong.d.aa@m.titech.ac.jp
* 
* @copyright Copyright 2021, The Chemical Plume Tracing (CPT) Robot Project"
* credits ["Luong Duc Nhat"]
* license GPL
*/

#include <SoftwareSerial.h>
#include <ros.h>
#include <olfaction_msgs/anemometer.h>

SoftwareSerial windSerial(10,11);

ros::NodeHandle nh;
olfaction_msgs::anemometer wind_msg;
ros::Publisher windPublisher("chatter", &wind_msg);

void setup() {
    nh.initNode();
    nh.advertise(windPublisher);
    
    windSerial.begin(9600);
    Serial.begin(9600);
    delay(6000);

    //SET communication interface of the sensor to UART
    windSerial.write("$01CIU*//\r\n");
    delay(2000);
    
    //QUERY to check if the communicaiton protocol is set or not
    windSerial.write("$01CI?*//\r\n");
    delay(1000);

    //Disable compass
    windSerial.write("$01CFD*//\r\n");
    delay(1000);
}


double degree2rad(double angle) {
    if (angle <= 180.0 ) {
        return angle * M_PI / 180.0; 
    }
    else {
        return (direction - 360.0)* M_PI / 180.0;
    }
}


void loop() {
    //send query for WIND speed and direction
    windSerial.write("$//WV?*//\r\n");
    
    String storedData = "";
    while (windSerial.available()) {
        char inChar = windSerial.read();
        storedData += inChar;
    }
    
    double speed, direction;
    char str1 [10], str2 [10];
    sscanf(storedData.c_str(),"%[^=]= %lf, %lf, %[^,],", str1, &speed, &direction, str2);

    wind_msg.header.stamp = nh.now();  
    wind_msg.header.frame_id = "/anemometer_frame";
    wind_msg.sensor_label = "1";
    wind_msg.wind_speed = speed;
    wind_msg.wind_direction = degree2rad(direction);
    
    windPublisher.publish( &wind_msg );
    nh.spinOnce();
    delay(200);
}