#!/usr/bin/env python3

import rospy
from cable_loop_gripper.cable_loop_gripper_driver import CableLoopGripperDriver as CLGdriver

def CLG_control():
    gripper = CLGdriver('/dev/serial0',115200)
    rate = rospy.Rate(10) # 100 hz

    while not rospy.is_shutdown():
        gripper.updateDriver()
        rate.sleep()


if __name__ == '__main__':
    try:
        rospy.init_node('CLG_control', anonymous=True)
        CLG_control()
    except rospy.ROSInterruptException: pass
