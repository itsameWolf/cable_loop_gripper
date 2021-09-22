#!/usr/bin/env python3

import rospy
from panda_robot import PandaArm
from cable_loop_gripper.msg import CLGcommand,CLGstatus
from cable_loop_gripper.srv import CLGvision_loopRadius,CLGvision_vialXYdistance
import numpy

class VialTray:
    def __init__(self, rows=3, cols=3, vial_r=0.012, vial_spacing=0.026, tray_origin=(0.2,0.2)):
        self._rows = rows
        self._cols = cols
        self._n_vials = self._rows*self._cols

        self._vial_r = vial_r
        self._vial_spacing = vial_spacing
        self._tray_origin = tray_origin

def experiment():

    rospy.init_node('panda_arm_experiment')
    pub = rospy.Publisher("/CLG_command", CLGcommand, queue_size=10)
    arm = PandaArm()
    rate = rospy.Rate(10)
    proxy = rospy.ServiceProxy('CLG_control_node/CLG_vision_vial_xy_distance',CLGvision_vialXYdistance)

    def GripRoutine(force=10.0):
        msg = CLGcommand()
        value = float(force)            
        msg.requested_loop_radius= 0
        msg.requested_force = value
        msg.control_mode = False
        pub.publish(msg)

    def MoveLoopRoutine(radius=10.0):
        msg = CLGcommand()
        value = float(radius)            
        msg.requested_loop_radius= value
        msg.requested_force = 0
        msg.control_mode = True
        pub.publish(msg)

    def FindVialError():
        x, y, r = proxy(True)
        return float(x), float(y), float(r)

    arm.move_to_neutral()
    
    
    