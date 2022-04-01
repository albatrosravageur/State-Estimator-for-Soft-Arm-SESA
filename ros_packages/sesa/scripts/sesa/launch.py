#!/usr/bin/env python

import rospy, rosparam, rospkg, roslaunch, os, sys
import subprocess

rp = rospkg.RosPack()
package_path = rp.get_path('sesa')
sys.path.append(package_path+'/src')
from make_my_urdf import make_my_urdf
from make_my_GUI import make_my_GUI

if __name__ == '__main__':

    make_my_GUI()

    rp = rospkg.RosPack()
    package_path = rp.get_path('sesa')

    # Get config from file and push it to rosparam
    my_params = rosparam.load_file(package_path+'/config/'+rospy.get_param('/config_file'))[0][0]
    for key in my_params:
        rospy.set_param('/'+key, my_params[key])

    # Setup parameters
    imus = rospy.get_param('/imus')
    rospy.set_param('/imus/amount', sum(imus['enabled']))
    rospy.set_param('/imus/ids', [x+1 for x in range(8) if imus['enabled'][x]])


    markers = rospy.get_param('/markers')
    rospy.set_param('/markers/use',bool(sum(markers['enabled'])))
    rospy.set_param('/markers/ids', [x+1 for x in range(8) if markers['enabled'][x]])

    source = rospy.get_param('/source')
    mode = rospy.get_param('/mode')

              
    # Load useful arguments for the launch
    arduino_boot = rospy.get_param('/arduino/boot')
    arduino_port = rospy.get_param('/arduino/port')
    arduino_baud = rospy.get_param('/arduino/baud')
    record_accelerometers = rospy.get_param('/my_rosbag/to_record/accelerometers')
    record_markers = rospy.get_param('/my_rosbag/to_record/markers')
    record_quaternions = rospy.get_param('/my_rosbag/to_record/quaternions')
    record_file = rospy.get_param('/my_rosbag/to_record/file')
    rviz_gui = rospy.get_param('/rviz_gui')
    my_rosbag_play_rate =  rospy.get_param('/my_rosbag/to_play/rate')
    my_rosbag_play_start =  rospy.get_param('/my_rosbag/to_play/start')
    my_rosbag_play_file =  rospy.get_param('/my_rosbag/to_play/file')
    topics = rospy.get_param('/topics')
    topics_per_mode = rospy.get_param('/topics_per_mode')
    # Sanity check
    if arduino_boot and not source=='arduino':
        raise "Warning: you are booting code to the Arduino board but not reading data from arduino"

    # Push code to Arduino
    if arduino_boot:
        os.system("cd ~/catkin_ws && catkin_make sesa_firmware_arduino_"+mode+"-upload")

    node_list = []
    # Serial communication with Arduino
    if source == 'arduino':
        node_rosserial_arduino = roslaunch.core.Node(
                        package='rosserial_arduino', \
                        node_type= 'serial_node.py', \
                        args="_port:={} _baud:={}".format(arduino_port, arduino_baud), \
                        name='rosserial_arduino')  
        node_list.append(node_rosserial_arduino)

    # Estimator
    if mode == 'stream':
        node_estimator = roslaunch.core.Node(
            package='sesa', \
            node_type='estimator.py', \
            name='estimator')
        node_list.append(node_estimator)
    
    # Calibration
    if mode == 'calibrate':
        node_calibrate = roslaunch.core.Node(
            package='sesa', \
            node_type='save_calibration.py', \
            name='calibration_saver')
        node_list.append(node_calibrate)       

    # RVIZ (visualization)
    if rviz_gui and mode =='stream':
        make_my_urdf()
  
        my_urdf_cmd = ['rosrun xacro xacro '+package_path+'/rviz/myrobot.urdf.xacro']
        robot_description = subprocess.Popen(my_urdf_cmd, shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE).communicate()
        rospy.set_param('/robot_description', robot_description[0])
        rvizconfig = package_path+'/rviz/urdf.rviz'
        rospy.set_param('/rvizconfig', rvizconfig)   

        node_rviz = roslaunch.core.Node(
            package='rviz',\
            node_type='rviz',\
            name='rviz',
            args='-d '+rvizconfig, 
            required="true")    
        node_list.append(node_rviz)

        node_robot_state_publisher = roslaunch.core.Node(
            package='robot_state_publisher',\
            node_type='robot_state_publisher',\
            name='robot_state_publisher')  
        node_list.append(node_robot_state_publisher)
    
    # My rosbag
    # Play
    if source == 'my_rosbag' and mode =='stream':
        node_my_rosbag_play = roslaunch.core.Node(
            package='rosbag', \
            node_type='play', \
            name='my_rosbag_play',
            args=package_path+'/bags/'+my_rosbag_play_file+ \
                ' --topics /quat_meas ' +\
                ' -r '+str(my_rosbag_play_rate) +\
                ' -s '+str(my_rosbag_play_start))
        node_list.append(node_my_rosbag_play)

    # Record
    topics_to_record = ""
    if record_accelerometers: topics_to_record += ' /acc_meas '
    if record_markers: topics_to_record += ' /markers '
    if record_quaternions: topics_to_record += ' /quat_meas '

    if topics_to_record is not "" and  mode =='stream':
        node_my_rosbag_record = roslaunch.core.Node(
            package='rosbag', \
            node_type='record', \
            name='my_rosbag_record',
            args= topics_to_record + '-O '+ package_path+'/bags/'+record_file )
        node_list.append(node_my_rosbag_record)

    # Play topics
    for topic in topics_per_mode[mode]:
        if topic is not '':
            if topics[topic]: 
                os.system("gnome-terminal --title='"+topic+"' -e 'rostopic echo /"+topic+"'")

    launch = roslaunch.scriptapi.ROSLaunch()
    launch.start()
    for node in node_list:
        process = launch.launch(node)
    try:
        launch.spin()
    finally:
    # After Ctrl+C, stop all nodes from running
        launch.shutdown()
