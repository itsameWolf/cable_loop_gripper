#!/usr/bin/env python3

import rospy
from cable_loop_gripper.cable_loop_gripper_vision_driver import CableLoopGripperVisionDriver as vision
from cable_loop_gripper.srv import CLGvisionSRV, CLGvisionSRVResponse

def visionServer():
    vs = vision()
    rospy.init_node('CLG_vision_server')
    srv = rospy.Service('CLG_vision',CLGvisionSRV,vs.runVisionRoutine)

    while rospy.is_shutdown() is not True:
        vs.runVision()

if __name__ == "__main__":
    visionServer()