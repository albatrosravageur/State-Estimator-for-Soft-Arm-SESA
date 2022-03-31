#!/usr/bin/env python
from distutils.log import error
import rospy
import rospkg

def make_my_urdf():
  rp = rospkg.RosPack()
  package_path = rp.get_path('sesa')

  imus = rospy.get_param('/imus')
  segments = rospy.get_param('/segments')
  markers = rospy.get_param('/markers')  
  arm_radius = rospy.get_param('/arm_radius')

  f = open(package_path+'/rviz/myrobot.urdf.xacro', "w")
  f.write( """<?xml version="1.0"?>

  <robot name="my_rigid_arm" xmlns:xacro="http://ros.org/wiki/xacro">

  <material name="black">
    <color rgba="0 0 0 1"/>
  </material>

  <material name="purple">
    <color rgba="1 0 1 1"/>
  </material>


  <link name="base_link" />

  <xacro:macro name="addSegment" params="parent child color leng">

  <!-- create the next link -->
  <link name="${child}">
    <visual>
      <geometry>
            <cylinder radius=\"""" +str(arm_radius)+"""\" length="${leng}"/>
      </geometry>
      <origin rpy="0 0 0" xyz="0 0 ${leng/2}"/>
      <material name="${color}"/>
    </visual>
  </link>

  <!-- setup the coordinate frame -->
  <joint name="${parent}${child}" type="floating" >
      <origin xyz="0 0 0" />
      <parent link="${parent}" />
      <child link="${child}" />
  </joint> 
  </xacro:macro>

  <xacro:macro name="addMarker" params="name">

    <link name="${name}">
      <visual>
        <geometry>
          <sphere radius=\""""+str(markers['radius'])+"""\"/>
        </geometry>
        <material name="green">
              <color rgba="0 1 0 1"/>
        </material>
      </visual>
    </link>

    <!-- setup the coordinate frame -->
  <joint name="base_link${name}" type="floating" >
      <origin xyz="0 0 0" />
      <parent link="base_link" />
      <child link="${name}" />
  </joint> 
  </xacro:macro>

  """)

  links = ['base_link']
  for imu in range(imus['amount']):
    for segment in range(segments[imu]):
      links.append(str(imu+1)+str(segment+1))

  past_link = links[0]
  for link in links[1:]:
    seg = int(link[0])
    color = 'black' if seg%2 else 'purple'
    parent = past_link
    child = link
    length = (imus['positions'][seg]-imus['positions'][seg-1])/segments[seg-1]
    f.write('<xacro:addSegment parent="%s" child="%s" color="%s" leng="%s"/> \n' \
        % (parent, child, color, length))
  past_link = link

  f.write("\n")

  for marker in range(markers['amount']):
    f.write("<xacro:addMarker name='marker_%s'/> \n" % (marker))

  f.write("</robot> \n")
  f.close()
  return 1
