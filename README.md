# cpt_hardware
Compilation of Arduino codes to control several devices such as dc fans and gas release devices serving for CPT experiments in Kurabayashi Lab in ToKyo Institute of Technology. 

## anemometer
The code establishes the communication between Arduino Uno and the Wind sensor using serial communication. Wind data is read and published to the ROS 1 network.
> Dependence ROS package: [olfaction_msgs](https://github.com/MAPIRlab/olfaction_msgs)

## arx
Using the ARX model to output the binary detection of gas presence from the raw data value of the gas sensor. All the constants are pre-determined in this [paper](https://ieeexplore.ieee.org/abstract/document/7987788)

## co2_humid
The code establishes the communication between Arduino Uno and CO2-Temperature-humid sensors using serial communication. 
 Device URL: [SCD30](https://wiki.seeedstudio.com/Grove-CO2_Temperature_Humidity_Sensor-SCD30/)

## dc_fan
Varying DC fan speed and swing motion for recording different shapes of smoke for the PIV experiment.

## dynamixel_test_connect
Test the connection of the Dynamixel motor to the Arduino.

## gas_spray
This code is used to automate the ethanol spay action using a relay module and [electric spray](https://www.monotaro.com/g/03004290/).

## valve_switch
Program to open and close the solenoid valve at regular intervals. Triggered to start cyclic operation of the solenoid valve. 
> Serial command: 'g' for ON, 'q' for OFF.
