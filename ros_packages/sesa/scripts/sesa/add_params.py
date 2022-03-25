#!/usr/bin/env python
import rospy, rosparam, rospkg 

rp = rospkg.RosPack()

package_path = rp.get_path('sesa')

imus = rospy.get_param('/imus')
rospy.set_param('/imus/amount', len(imus['positions']))

markers = rospy.get_param('/markers')
rospy.set_param('/markers/amount', len(markers['positions']))
