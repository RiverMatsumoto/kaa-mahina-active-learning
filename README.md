# Kaa Mahina Active Learning Rover

#### Rover Software for Robotics Space Exploration Lab at UH Manoa
This repository is for the active learning rover developed at the Robotics Space Exploration Lab at University of Hawaii at Manoa.
There are two rovers that use similar software and the mobility rover software can be found at
https://github.com/RiverMatsumoto/kaa-mahina-mobility.

The difference between this rover and the mobility rover is in the moisture sensor probe mechanism and GPS functionality.
This rover needs to be able to communicate to the ground station computer which must be running ROS Humble or Ubuntu 22.04.


There are two rovers using the software in this repository, the Active Learning Rover and Mobility Rover.
Each rover requires a LAN to connect to the ground station computer running Ubuntu 22.04 and ROS Humble.
There will be an installation script will soon be added to this repository for the ground station computer
to install the required Vicon shared object libraries and python packages.

If you have the rover and are trying to use a new laptop to use the rovers, follow the instructions for 
installation for the ground station computer and ignore the Raspberry Pi software installation steps.

## Build Software for Active Learning Rover (Ground station computer and rover onboard computers)

### On Ground Station Computer
Clone The repository, cd into the directory and build
```
git clone --recursive https://github.com/RiverMatsumoto/kaa-mahina-active-learning
cd kaa-mahina-active-learning
./ccbuild.sh
```

### On Rover Raspberry pi and Jetson Orin Nano
Follow the steps outlined in this guideline: https://docs.google.com/document/d/185jutgDcAFHJFgQgFOSVkJMf8qD-lXxG1FFA9OOJPeI/edit?usp=sharing.

The guide details field testing set-up procedures, wifi set-up, connection to onboard computers, launching of all hardware in ROS2, and how to run a complete trial.  

## Rover Hardware Components Used
### Mobility Rover
Motor Controllers - 2x Roboclaw 2x7A Motor Controllers
IMU - BNO055
Cameras - IMX219 Webcam 1x per Raspberry Pi
2x Raspberry Pi (1 USB to UART to communicate with motor controllers)
Battery with 2 USB A outputs

### Active Learning Rover
Motor Controllers - 2x Roboclaw 2x7A Motor controllers (Onboard computers communicate with USB to UART)
IMU - BNO055
GPS - 2x ZED_F9P RTK GPS Boards (and antennas)
Arduino Nano with Moisture Sensor (powered from connection to Raspberry Pi)
Ubuntu 22.04 Laptop with ROS Humble installed
Windows Laptop with Ublox's `u-center` GPS utilities app installed 
Batteries with 1 USB A output, 1 AC output, 1 DC output
