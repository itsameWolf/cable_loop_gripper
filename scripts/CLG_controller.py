#!/usr/bin/env python3

import rospy
from cable_loop_gripper.cable_loop_gripper_driver import CableLoopGripper as CLG

def talker():
    grippper = CLG()
    while not rospy.is_shutdown():
        grippper.getCLGstatus()
        print(grippper.CLG_status)

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException: pass