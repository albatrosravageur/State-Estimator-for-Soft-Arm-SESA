#!/usr/bin/env python

import rospy
from sesa.msg import calib, calibration_status
from std_msgs.msg import Header
import rosparam
import os, yaml

import rospkg
rp = rospkg.RosPack()
package_path = rp.get_path('sesa')
file = rospy.get_param('/calib/calibrate_file')
file_path = package_path+'/calibs/'+file
my_calibration_status = calibration_status()

def callback(data):
    sensor = str(data.ID)
    with open(file_path) as f:
        doc = yaml.safe_load(f)
        f.close()

    my_set_param = lambda this_param, this_data: rospy.set_param('/calib/measurements/imu_'+sensor+this_param, this_data)

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
    setattr(my_calibration_status, 'header',"sys  gyro   acc    mag")
    setattr(my_calibration_status,'imu_'+sensor,str(data.sys)+'     '+str(data.gyro)+'     '+str(data.acc)+'     '+str(data.mag))
    pub = rospy.Publisher('calib_status', calibration_status, queue_size=10)
    pub.publish(my_calibration_status)
    if data.off_accX or data.off_accY or data.off_accZ: #If the calib is not zero, i.e if the IMU is connected
        doc[sensor]= {
            'sys':{'status':data.sys},
            'acc':{
                'status':data.acc,
                'x':data.off_accX,
                'y':data.off_accY,
                'z':data.off_accZ
            },
            'gyro':{
                'status':data.gyro,
                'x':data.off_gyroX,
                'y':data.off_gyroY,
                'z':data.off_gyroZ
            },
            'mag':{
                'status':data.mag,
                'x':data.off_magX,
                'y':data.off_magY,
                'z':data.off_magZ
            },
            'rad':{
                'mag':data.rad_mag,
                'acc':data.rad_acc
            }            
        }
        with open(file_path, 'w') as f:
            yaml.dump(doc, f)


def listener():	
    rospy.init_node('calibration_status', anonymous=True)
    rospy.Subscriber("calib_meas", calib, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()




        
