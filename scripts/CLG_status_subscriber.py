#!/usr/bin/env python3
import rospy
from CLG_status.msg import CLG_status

def callback(data):
    rospy.loginfo("%s is age: %d" % (data.name, data.age))

def listener():
    rospy.init_node('custom_listener', anonymous=True)
    rospy.Subscriber("custom_chatter", Person, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()