#!/usr/bin/env python3

import rospy
from cable_loop_gripper.msg import CLGstatus

def talker():
    pub = rospy.Publisher('chatter', CLGstatus)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        hello_str = "hello world %s" % rospy.get_time()
        rospy.loginfo(CLG_status)
        pub.publish(CLG_status)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException: pass