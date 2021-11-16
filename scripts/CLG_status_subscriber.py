#!/usr/bin/env python3
import rospy
from cable_loop_gripper.msg import CLGstatus

def callback(data):
    rospy.loginfo("current force%f" % (data.current_force))
    
def listener():
    rospy.init_node('status_listener', anonymous=True)
    rospy.Subscriber("CLG_status", CLGstatus, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
    rospy.spin()
