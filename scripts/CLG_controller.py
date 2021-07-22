#!/usr/bin/env python3

import rospy
from cable_loop_gripper.cable_loop_gripper_driver import CableLoopGripperDriver as CLGdriver
from cable_loop_gripper.msg import CLGstatus, CLGcommand

'''
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
'''


def CLG_control():
    gripper = CLGdriver('/dev/serial0',115200)
    msg_status = CLGstatus
    pub = rospy.Publisher('CLG_status', msg_status, queue_size=50)
    #sub = rospy.Subscriber('CLG_commands', CLGcommand, CLG_command_handler)
    rate = rospy.Rate(10) # 100 hz

    while not rospy.is_shutdown():
        gripper.updateDriver()
        rate.sleep()


if __name__ == '__main__':
    try:
        rospy.init_node('CLG_control', anonymous=True)
        CLG_control()
    except rospy.ROSInterruptException: pass