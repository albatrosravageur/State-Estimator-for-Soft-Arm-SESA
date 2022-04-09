#!/usr/bin/env python
from gettext import translation
from operator import xor

from numpy import False_
import rospy
from sesa.msg import quat, marker, acc
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from tf.transformations import*
import tf
import ros_numpy
import numpy as np
import math
from tf import TransformBroadcaster
import rospy
from rospy import Time 
import rosparam
import rospkg

rp = rospkg.RosPack()
package_path = rp.get_path('sesa')
PI = 3.14159265359

def callback_quat(data):
    # Pushed the rotation and translation of the segments to robot_state_publisher
    b = TransformBroadcaster()

    # IMU re-orientation to fit their placement along the arm
    global Q
    base_quat = rospy.get_param('imus/offset_quaternion')
    Q[data.ID] = quaternion_multiply([data.x,data.y,data.z,data.w], [base_quat['x'], base_quat['y'], base_quat['z'], base_quat['w']])

    this_joint, next_joint = np.array([0,0,0], dtype=float) , np.array([0,0,0], dtype=float)
    marker_iter, imu_iter_for_output = 0, 0

    for imu in imus['list']:
        #TODO #2
        imu_iter_for_output += 1
        this_imu, prev_imu = imu, imus['list'][imus['list'].index(imu)-1] if imus['list'].index(imu) else 0
        q0, q1  = Q[prev_imu], Q[this_imu]

        for segment in range(segments[this_imu-1]):
            segment_length = (imus['positions'][this_imu]-imus['positions'][prev_imu])/segments[this_imu-1]

            # Compute the local orientation
            t = float(2*segment+1)/float(2*segments[this_imu-1])
            qt = quaternion_slerp(q0,q1,t)

            # Compute the position of the next joint
            rot_mat_3rd_column =  np.array([2*(qt[0]*qt[2] + qt[3]*qt[1]), \
                                           2*(qt[1]*qt[2]-qt[3]*qt[0]), \
                                           1-2*(qt[0]*qt[0]+qt[1]*qt[1])], dtype=float)
            this_joint = next_joint
            next_joint = this_joint + segment_length*rot_mat_3rd_column
                
            # Send the transform 
            target_ID = str(imu_iter_for_output)+str(segment+1)
            b.sendTransform(this_joint, qt, Time.now(), target_ID, 'base_link')

            # Save MoCap marker position
            if markers['use'] is False: continue 
            while marker_iter<markers['amount'] and \
                markers_wrt_imus_segments[marker_iter]['imu'] == prev_imu and \
                markers_wrt_imus_segments[marker_iter]['segment'] == segment:

                marker_coordinates = this_joint + markers_wrt_imus_segments[marker_iter]['dist_to_joint']*rot_mat_3rd_column
                # Publishs
                pub_marker = rospy.Publisher('markers', marker, queue_size=100) 
                pub_marker.publish(fill_marker(data, marker_coordinates, marker_iter))
                b.sendTransform(marker_coordinates, qt, Time.now(), 'marker_'+str(marker_iter), 'base_link')
               
                marker_iter += 1

    

def find_marker_seg(markers, segments, imus):
    markers_wrt_imus_segments, m_iter = [], 0
    if markers['positions'] is None: 
        return None

    while m_iter < markers['amount']:
        length, pos_marker, break_flag = 0, markers['positions'][m_iter], 0
        for imu in range(imus['amount']):
            segment_length = (imus['positions'][imu+1]-imus['positions'][imu])/segments[imu]
            for segment in range(segments[imu]):
                length += segment_length
                if length > pos_marker:
                    dist_to_joint = length-pos_marker
                    markers_wrt_imus_segments.append({'imu':imu,
                                             'segment':segment, 
                                             'dist_to_joint':dist_to_joint})
                    m_iter +=1
                    break_flag = 1
                    break
            if break_flag: break
    return markers_wrt_imus_segments

def fill_marker(data, this_translation, m_iter):
    out = marker()
    out.header = data.header
    out.header.stamp = rospy.Time.now()
    setattr(out, 'ID', m_iter)
    setattr(out, 'l', markers['positions'][m_iter])
    setattr(out, 'x', this_translation[0])
    setattr(out, 'y', this_translation[1])
    setattr(out, 'z', this_translation[2])
    return out


def listener():	
    rospy.init_node('estimator', anonymous=True)
    rospy.Subscriber("/quat_meas", quat, callback_quat)
    rospy.spin()


if __name__ == '__main__':
    # Variables IMUs
    my_rosbag = rospy.get_param('/my_rosbag')
    imus = rospy.get_param('/imus')
    rospy.logout(imus)

    # Variable segments and markers
    segments = rospy.get_param('/segments')
    markers = rospy.get_param('/markers')
    markers_wrt_imus_segments = find_marker_seg(markers, segments, imus)

    # Variable quaternions
    Q = np.zeros((imus['amount']+1,4), dtype=float)   # Imu data under the quaternion ([x y z w]) form
    base_orientation_quaternion = rospy.get_param('/base_orientation_quaternion')               
    Q[0] = np.array((base_orientation_quaternion['x'], \
                    base_orientation_quaternion['y'], \
                    base_orientation_quaternion['z'], \
                    base_orientation_quaternion['w']))

    use_saved_calib = rospy.get_param('/calib/use_saved')
    if use_saved_calib:
        my_params = rosparam.load_file(package_path+'/calibs/'+ rospy.get_param('/calib/saved_file'))[0][0]
        for key in my_params:
           rospy.set_param('/'+key, my_params[key])
    listener()



        
