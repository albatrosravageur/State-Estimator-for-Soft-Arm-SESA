cmake_minimum_required(VERSION 2.8.3)

# EDIT HERE 
set(my_port /dev/ttyACM0)
set(my_board mega2560)
set(ARDUINO_SDK_PATH,~/snap/arduino/61/Arduino)
# Remove this if using an Arduino without native USB (eg, other than Leonardo)
#add_definitions(-DUSB_CON)

# END EDIT

include_directories(
    ${ROS_LIB_DIR}
    libs/myButton libs/ros_lib 
    libs/LiquidCrystal 
    libs/Adafruit_Unified_Sensor 
    libs/Adafruit_BusIO 
    libs/Adafruit_BNO055 
    libs/TCA9548A
    )

include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX14)

CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX14)
    set(CMAKE_CXX_FLAGS "-std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "-std=c++0x")
else()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()


generate_arduino_firmware(stream
    SRCS  scripts/stream.cpp   
    ${ROS_LIB_DIR}/time.cpp 
    BOARD mega2560
    PORT /dev/ttyACM1
)

generate_arduino_firmware(calibrate
  SRCS  scripts/calibrate.cpp   
        ${ROS_LIB_DIR}/time.cpp 
  BOARD mega2560
  PORT /dev/ttyACM1
)




