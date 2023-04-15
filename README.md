# Autonomous gas detector robot

-> Objective

The scope of this work is to remotely monitor gas emissions emulating, in a controlled manner, the situation in industries to aid worker safety in environments with the presence of hazardous gases using mobile robotics and LoRa, given the challenge of using low-cost sensors and communication interference. While the robot is in motion, its displacement is estimated using odometry. The gas concentration and odometry information is sent to and stored on the ThingSpeak server and MATLAB.

-> Materials Used

Arduino UNO
Track-type robot + DC motors + H-bridge + LiPo battery
Dragino Kit (LoraShield + LoRa LG01-P Gateway + Arduino UNO)
Ultrasonic Sensor
MQ Gas Sensors
Optical Encoder 
Arduino IDE
ThingSpeak
Matlab

-> Built Prototype

![1](https://user-images.githubusercontent.com/54686271/232245294-59626b40-547a-4e88-815d-3aad9b0eccf9.png)

-> Results

Methane gas concentration information on ThingSpeak

![1](https://user-images.githubusercontent.com/54686271/232245476-5d5fb667-c101-4b4d-bb67-958b4dd84292.png)

Robot trajectory estimated by odometry

![1](https://user-images.githubusercontent.com/54686271/232245548-46d6d411-e7c2-4506-8e87-69ff1ee5c4d0.png)


