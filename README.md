 A soft arm state estimator (SESA) using IMUs
## Goal
This project aims at estimating the state of soft hyperredundant arms using Inertial Measurement Units (IMU) placed along the arm. This repo contains tools to calibrate and stream your IMUs, and to reconstruct the shape based on your custom parameters. A GUI makes the  modeling process more convenient.

## Compatibility
- ROS Melodic
- Ubuntu 18.04

## State estimation method
To estimate the state of the arm, we sense its orientation at each IMU emplacement by reading the corresponding IMU. Then, we model the arm as a series of rigid bodies. Between each IMUs, the arm is cut into a number of rigid bodies set by the user. Each rigid body is oriented using spherical linear interpolation. 

![Modeling of your soft arm](images/models.jpg)

The state estimation method is based on the IROS 2022 paper: "A Proprioceptive Method for Soft Robots Using Inertial Measurement Units", by Yves J. Martin, Daniel Bruder and Robert J. Wood. This paper is stored at the root of this Git repo. 

## Material needed
- 1 to 8 Bosch BNO055 (IMU)
- a I2C Multiplexer (MUX) such as the TCA9548A
- a board Arduino Mega 2560
- a LCD screen such as the E334023
- a potentiometer such as the PDB181-K220K-102B
- a switch such as Mouser's 485-1478

## Key performances
Using 8 IMUs and modeling the arm by 32 rigid bodies:
- the package runs real-time on a P14s Lenovo i7 16Go RAM
- the reading frequency of the IMUs is around 30Hz, but could be improved since the BNO055 output rate is 100Hz.
- the experimentation described in the paper shows around 10% of accuracy if the arm has a piece-wise constant curvature behavior between the IMUs and if the maximum angle between two IMUs is pi/2 rad.

# Repo structure
## SESA Package
This ROS package streams IMUs using I2C protocol and a multiplexer. Then, it estimates the shape of the soft arm and displays it on a RVIZ interface.
## SESA Firmware
Contains the Arduino firmware that is pushed to the Arduino Board. 
## Data processing
A Jupyter Notebook to evaluate the performance of this method w.r.t motion capture (only useful to check the results of the paper).


# Wire up your hardware
Your hardware circuit should look like on the following shematic. Good pratise is to numberize your IMUs from 1 to 8. 

#TODO #3 
hardware setup

Connect your board to your computer.

# Installation
## Run Ubuntu 18.04 
This package runs on ROS Melodic. It is recommanded to use ROS Melodic on Ubuntu 18.04. 

Ubuntu 18.04 can be downloaded [here](https://releases.ubuntu.com/18.04/).

It can be run as a virtual machine using Virtual Box [here](https://www.virtualbox.org/wiki/Downloads).

## Install ROS Melodic
A tutorial to install ROS Melodic can be found [here](http://wiki.ros.org/melodic/Installation/Ubuntu).

Note: we recommend to create your catkin workspace under the folder `home/[YOUR USER]`.  

## Clone the repo
Clone the repo to your `src/` folder:

```
cd ~/catkin_ws/src
git clone https://github.com/albatrosravageur/State-Estimator-for-Soft-Arm-SESA
```

## Make
Make using the command:
```
cd ~/catkin_ws
catkin_make
```


## Test your installation
As a sanity check, try to run the main launch file.
```
roslaunch sesa my_launch.launch
```

A GUI should appear. Click "quit" to exit.

# Calibration
Each IMU will be calibrated one by one. 

## Run the launcher
```
roslaunch sesa my_launch.launch
```

## Select a configuration using the GUI

### Select calibration mode
Select "calibration" as mode. Check out the top right of the GUI.

### Choose a calibration file 
And write it in the corresponding text line editor. It will be stored in `[path to main folder]/sesa/calibs`. Default is my_calib.yaml.

### Select the # of the IMU that you are calibrating
The calibration attached to this value will be changed in your calibration file.

### Click the arduino boot case
This button will push the calibration firmware directly to the arduino board. You need to click this button when you want to switch mode from stream to calibration and from calibration to stream.

### Set the Arduino port and Baud rate
Select the right port to connect your Arduino board. 
Defaut Baud rate is 115200, but if you change it in the firmware, you have to change it here. 

### Select what topic you want to see 
On the rostopic echo part, you can decide to monitor different topics such as the state of calibration (how well is the calibration done), the calibration itself and rosout. It is recommended to click the state of calibration on.

### You're all set!
You can click the start! button.

## Calibrate the IMU
The IMU has 3 sensors to calibrate, the gyroscope, the magnetometer and the accelerometer. Once the calibration status is 3 for all of the sensors and for the system, you can kill the program. Calibration will be saved in your file automatically.
### The gyroscope
Just keep the IMU still for a few seconds to calibrate the gyroscope.
### The magnetometer
Draw an 8 in the air like shown on this figure.
![Calibrate magnetometer](images/magcalibration.png)
### The accelerometer
Hold your IMU in different positions such as shown on this figure. 
![Calibrate accelerometer](images/accelerometer.png)

streaming

