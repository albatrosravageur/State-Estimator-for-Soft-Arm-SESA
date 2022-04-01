# A soft arm state estimator (SESA) using IMUs
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
- the experimentation described in the paper shows around 10% of accuracy if the arm has a piece-wise constant curvature behavior between the IMUs and if the maximum angle between two IMUs is ```math 
\frac{\pi}{2}
```.

# Repo structure
## SESA Package
This ROS package streams IMUs using I2C protocol and a multiplexer. Then, it estimates the shape of the soft arm and displays it on a RVIZ interface.
## SESA Firmware
Contains the Arduino firmware that is pushed to the Arduino Board. 
## Data processing
A Jupyter Notebook to evaluate the performance of this method w.r.t motion capture (only useful to check the results of the paper).

