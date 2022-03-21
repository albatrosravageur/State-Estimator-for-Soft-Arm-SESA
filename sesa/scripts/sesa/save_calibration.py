#!/usr/bin/env python

import rospy
from sesa.msg import calib, calibration_status
from std_msgs.msg import Header
import rosparam
import os

import rospkg
rp = rospkg.RosPack()
package_path = rp.get_path('sesa')
file = rospy.get_param('/calib/calibrate_file')
path = rospy.get_param('/calib/calibrate_path')
my_calibration_status = calibration_status()

def callback(data):
    sensor = str(data.ID)
    my_set_param = lambda this_param, this_data: rospy.set_param('/calib/measurements/'+sensor+this_param, this_data)

    # Push the config to parameters
    my_set_param('/sys/status', data.sys)
    my_set_param('/gyro/status', data.gyro)
    my_set_param('/acc/status', data.acc)
    my_set_param('/mag/status', data.mag)

    ## The offset, only when parameters are sufficient (Arduino decides when to push those)
    my_set_param('/acc/x', data.off_accX)
    my_set_param('/acc/y', data.off_accY)
    my_set_param('/acc/z', data.off_accZ)
    my_set_param('/mag/x', data.off_magX)
    my_set_param('/mag/y', data.off_magY)
    my_set_param('/mag/z', data.off_magZ)
    my_set_param('/gyro/x',data.off_gyroX)
    my_set_param('/gyro/y',data.off_gyroY)
    my_set_param('/gyro/z',data.off_gyroZ)
    my_set_param('/rad/acc',data.rad_acc)
    my_set_param('/rad/mag',data.rad_mag)

    setattr(my_calibration_status,'imu_'+sensor,data.sys)
    pub = rospy.Publisher('calibration_status', calibration_status, queue_size=10)
    pub.publish(my_calibration_status)

    rosparam.dump_params(path+file, '/calib/measurements')

def listener():	
    rospy.init_node('calibration_status', anonymous=True)
    rospy.Subscriber("calib_meas", calib, callback)
    rospy.spin()

if __name__ == '__main__':
    if os.path.exists(path+file):
        f = open(path+file, "w")
        f.close()
    listener()




        
