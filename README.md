# arduino_hardware_ctrl
Compilation of Arduino codes to control several devices such as dc fan and gas relief device serving for CPT experiments in Kurabayashi Lab in ToKyo Institute of Technology. 

## anemometer
The code establish the communication between Arduino Uno and Wind sensor using serial communication. Wind data is read and published to ROS 1 network.
> Dependence ROS package: [olfaction_msgs](https://github.com/MAPIRlab/olfaction_msgs)
