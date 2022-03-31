
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

## Wire up your hardware
Your hardware circuit should look like on the following shematic.

#TODO #3 
hardware setup

Connect your board to your computer.

## Test your installation
As a sanity check, try to run the firmware.
```
roslaunch sesa my_launch.launch
```

A GUI should appear. Click "quit" to exit.

# Calibration


calibrate IMUs one by one
run 

calibration

creating your model

placing markers

streaming

This repo streams several BNO055 IMUs using an I2C MUX, in order

 implements the paper 

