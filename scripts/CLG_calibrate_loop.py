#!/usr/bin/env python3

import rospy
import sys
from cable_loop_gripper.srv import CLGvision_loopRadius,CLG_offsetLoopRadius

def calibrate_loop(n = 20):
    rospy.wait_for_service('CLG_control_node/CLG_vision_loop_radius')
    vision_proxy = rospy.ServiceProxy('CLG_control_node/CLG_vision_loop_radius',CLGvision_loopRadius)
    offset_proxy = rospy.ServiceProxy("CLG_control_node/CLG_set_radius_offset",CLG_offsetLoopRadius)
    
    print("acquirign loop radius...")
    values = [] 
    for i in range(n):
        loop = vision_proxy(True)
        values.append(float(loop.loop_radius))
        print(i)
    sum_values = 0.0
    sum_values = sum(values)
    loop_rad = sum_values/n
    rospy.loginfo(loop_rad)
    offset_proxy(loop_rad)


if __name__ == "__main__":
    n_iter = int(sys.argv[1]) 
    rospy.init_node('CLG_calibration', anonymous=True)
    rospy.loginfo("start loop calibration")
    calibrate_loop(n_iter)
