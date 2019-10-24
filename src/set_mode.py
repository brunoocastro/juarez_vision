#!/usr/bin/env python
# -*- coding: utf-8 -*-


import rospy
from robotis_controller_msgs.srv import SetModule
from std_msgs.msg import String
import time

#a = SetJointModule()
global pub
pub = rospy.Publisher('robotis/walking/command', String, queue_size=1)

#a.joint_name = ['r_hip_yaw', 'r_hip_roll', 'r_hip_pitch', 'r_knee', 'r_ankle_pitch', 'r_ankle_roll', 'l_hip_yaw', 'l_hip_roll', 'l_hip_pitch', 'l_knee', 'l_ankle_pitch', 'l_ankle_roll']
#a.module_name = ['walking_module','walking_module','walking_module','walking_module','walking_module','walking_module','walking_module','walking_module','walking_module','walking_module','walking_module']
def service():

    service = rospy.ServiceProxy('robotis/set_present_ctrl_modules', SetModule)
    service("walking_module")

def topic():
    global pub
    time.sleep(10)

    rospy.init_node('set_juarez_module', anonymous=True)
    while not rospy.is_shutdown():
        pub.publish("start")

def shutdown():
     global pub
     pub.publish("stop")
     rospy.loginfo("Stopping the robot...")
     # always make sure to leave the robot stopped
     

if __name__ == '__main__':
    try:
        rospy.on_shutdown(shutdown)
        service()
	topic()
    except rospy.ROSInterruptException:
        pass
