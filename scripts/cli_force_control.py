#!/usr/bin/env python3

import rospy
from cable_loop_gripper.msg import CLGcommand

def cli_force_control():

    rospy.init_node('force_input')
    pub = rospy.Publisher("/CLG_command", CLGcommand, queue_size=10)

    while not rospy.is_shutdown():

        msg = CLGcommand()
        force = float(input('input loop target lenght: '))

        msg.requested_loop_length = 0
        msg.requested_force = force
        msg.control_mode = False

        pub.publish(msg)

if __name__ == '__main__':
    cli_force_control()