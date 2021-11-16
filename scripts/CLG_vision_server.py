#!/usr/bin/env python3

import rospy
from cable_loop_gripper.cable_loop_gripper_vision_driver import CableLoopGripperVisionDriver as vision
from cable_loop_gripper.srv import CLGvision_vialXYdistance, CLGvision_loopRadius

def visionServer():
    vs = vision()
    rospy.init_node('CLG_vision_server')
    srv_vial = rospy.Service('CLG_vision_vial_xy_distance',CLGvision_vialXYdistance,vs.vialXYDistance)
    srv_loop = rospy.Service('CLG_vision_loop_radius',CLGvision_loopRadius,vs.loopRadius)

    while rospy.is_shutdown() is not True:
        vs.runVision()

if __name__ == "__main__":
    visionServer()
