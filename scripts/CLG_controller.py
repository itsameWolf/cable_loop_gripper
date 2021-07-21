#!/usr/bin/env python3
#include "cable_loop_gripper/msg/CLGstatus.msg"
#include "cable_loop_gripper/msg/CLGcommand.msg"

import rospy
from cable_loop_gripper.cable_loop_gripper_driver import CableLoopGripper as CLG
from CLGstatus.msg import CLGstatus
from CLGcommand.msg import CLGcommand

grippper = CLG()

def CLG_command_handler():
    lengthcommand = CLGcommand.requested_loop_length
    forceCommand = CLGcommand.requested_force
    controlCommand = CLGcommand.control_mode

    if lengthcommand != grippper.CLG_status.requested_force:
        grippper.requestLoopLength(lengthcommand)
        grippper.CLG_status.requested_loop_length = lengthcommand

    if forceCommand != grippper.CLG_status.requested_force:
        grippper.requestForce(forceCommand)
        grippper.CLG_status.requested_force = forceCommand

    if controlCommand != grippper.CLG_status.control_mode:
        grippper.setControlMode(controlCommand)
        grippper.CLG_status.control_mode = controlCommand

pub = rospy.Publisher('CLG_status', CLGstatus)
sub = rospy.Subscriber('CLG_commands', CLGcommand, CLG_command_handler)

def CLG_control():

    rospy.init_node()

    while not rospy.is_shutdown():
        grippper.getCLGstatus()
        print(grippper.CLG_status)
        grippper.requestLoopLength(20)

if __name__ == '__main__':
    try:
        CLG_control()
    except rospy.ROSInterruptException: pass