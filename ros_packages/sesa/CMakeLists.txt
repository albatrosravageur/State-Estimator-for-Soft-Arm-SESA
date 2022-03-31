cmake_minimum_required(VERSION 3.0.2)
project(sesa)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  rosserial_arduino
  rosparam
  message_runtime
  rosserial_client
  std_msgs
  message_generation
  sensor_msgs
  ros_numpy
)

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
 add_message_files(
   FILES
   calib.msg
   calibration_status.msg
   acc.msg
   quat.msg
   marker.msg
 )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )


## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs 
   sensor_msgs
)


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
catkin_package(
  CATKIN_DEPENDS message_runtime 
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  ${catkin_INCLUDE_DIRS}
)


catkin_install_python(PROGRAMS
scripts/sesa/save_calibration.py 
scripts/sesa/launch.py 
src/GUI_class.py
src/make_my_urdf.py 
src/make_my_GUI.py 
    scripts/sesa/estimator.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
