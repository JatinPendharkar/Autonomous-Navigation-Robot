# Autonomous-Navigation-Robot
SRH Hochschule Heidelberg 2022
Master Thesis 

![image](https://user-images.githubusercontent.com/86410054/164705961-ee05591b-7916-4578-9851-e225d23faaef.png)


The Thesis is based on Robot Operating System(ROS Noetic) on Ubuntu 20.04 on a Raspberry Pi 4 with 4gb RAM

The thesis is designed into two parts 
1. Autonomous NAvigation
The Locomotion is handled by the Arduino MEGA 2560 and connected to the Raspberry Pi using Serial interface
YdLiDAR x2L is used for the mappong the environment.
THe Navigation stack is based on SLAM(Simultaneous Localization and Mapping)
Gmapping for Creating MAP
Map_server for Saving the MAP
AMCL/iris_lama for Localization
Move_base for Navigation.
THe MAP on which the Robot was tobe Operated was

![image](https://user-images.githubusercontent.com/86410054/164705807-4b06b9ad-2a64-41be-ba88-9b1aa8968c93.png)


2. ArUco Detection
USB_cam Package is created for using the USB camera with ROS and using the OpenCv and ArUCo_detect package the ArUco detection is achieved.
The Output gives the Marker ID and also the distance from the USB_CAM frame along with the pose Estimation.
The Output Images for ArUco Detection are given below
![image](https://user-images.githubusercontent.com/86410054/164706157-4303cd55-75a0-48bb-b2ff-d60836db282f.png)
![image](https://user-images.githubusercontent.com/86410054/164706200-9d64961e-586c-451b-b095-da996f58de06.png)
![image](https://user-images.githubusercontent.com/86410054/164706270-f6c7089b-b9d0-4dd9-a430-d96206ddb970.png)
![image](https://user-images.githubusercontent.com/86410054/164706335-03795918-f7cc-4489-95cc-4e0335b7728e.png)
