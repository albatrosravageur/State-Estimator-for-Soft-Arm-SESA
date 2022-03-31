import rospy, rosparam, rospkg, roslaunch, os, sys
import subprocess

rp = rospkg.RosPack()
package_path = rp.get_path('sesa')
sys.path.append(package_path+'/src')
from make_my_urdf import make_my_urdf


if __name__ == '__main__':

    rp = rospkg.RosPack()
    package_path = rp.get_path('sesa')

    # Get config from file and push it to rosparam
    my_params = rosparam.load_file(package_path+'/config/my_params.yaml')[0][0]
    for key in my_params:
        rospy.set_param('/'+key, my_params[key])

    # Setup parameters
    imus = rospy.get_param('/imus')
    rospy.set_param('/imus/list', [x+1 for x in range(8) if imus['enabled'][x]])
    rospy.set_param('/imus/amount', sum([x for x in imus['enabled']]))
    
    markers = rospy.get_param('/markers')
    rospy.set_param('/markers/amount', len(markers['positions']))

    source = rospy.get_param('/source')

    my_urdf_cmd = ['rosrun xacro xacro '+rp.get_path('sesa')+'/rviz/myrobot.urdf.xacro']
    robot_description = subprocess.Popen(my_urdf_cmd, shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE).communicate()
    rospy.set_param('/robot_description', robot_description[0])
    rvizconfig = rp.get_path('sesa')+'/rviz/urdf.rviz'
    rospy.set_param('/rvizconfig', rvizconfig)
                            
    # Load useful arguments for the launch
    arduino_boot = rospy.get_param('/arduino/boot')
    arduino_port = rospy.get_param('/arduino/port')
    arduino_baud = rospy.get_param('/arduino/baud')
    record_accelerometers = rospy.get_param('/my_rosbag/to_save/accelerometers')
    record_quaternions = rospy.get_param('/my_rosbag/to_save/quaternions')
    rviz_gui = rospy.get_param('/rviz_gui')
    my_rosbag_rate =  rospy.get_param('/my_rosbag/to_play/rate')
    my_rosbag_start =  rospy.get_param('/my_rosbag/to_play/start')
    my_rosbag_file =  rospy.get_param('/my_rosbag/to_play/file')

    # Sanity check
    if arduino_boot and not source=='arduino':
        raise "Warning: you are booting code to the Arduino board but not reading data from arduino"

    # Push code to Arduino
    if arduino_boot:
        os.system("cd ~/catkin_ws && catkin_make sesa_firmware_arduino_calibrate-upload")

    node_list = []
    # Serial communication with Arduino
    if source == 'arduino':
        node_rosserial_arduino = roslaunch.core.Node(
                        package='rosserial_arduino', \
                        node_type= 'serial_node', \
                        args="_port:={} _baud:={}".format(arduino_port, arduino_baud), \
                        name='rosserial_arduino')  
        node_list.append(node_rosserial_arduino)

    # Estimator
    node_estimator = roslaunch.core.Node(
        package='sesa', \
        node_type='estimator.py', \
        name='estimator',
        args='-d '+rvizconfig, 
        required="true")  
    node_list.append(node_estimator)

    # RVIZ (visualization)
    if rviz_gui:
        make_my_urdf()
        rospy.sleep(0.1) # Time to build the model 

        node_rviz = roslaunch.core.Node(
            package='rviz',\
            node_type='rviz',\
            name='rviz')  
        node_list.append(node_rviz)

        node_robot_state_publisher = roslaunch.core.Node(
            package='robot_state_publisher',\
            node_type='robot_state_publisher',\
            name='robot_state_publisher')  
        node_list.append(node_robot_state_publisher)
    
    # My rosbag
    if source == 'my_rosbag':
        node_my_rosbag = roslaunch.core.Node(
            package='rosbag', \
            node_type='play', \
            name='my_rosbag_play',
            args=package_path+'/saved/bags/'+my_rosbag_file+ \
                ' -r '+str(my_rosbag_rate) +\
                ' -s '+str(my_rosbag_start))
        node_list.append(node_my_rosbag)

    launch = roslaunch.scriptapi.ROSLaunch()
    launch.start()
    for node in node_list:
        process = launch.launch(node)

    try:
        launch.spin()
    finally:
    # After Ctrl+C, stop all nodes from running
        launch.shutdown()
