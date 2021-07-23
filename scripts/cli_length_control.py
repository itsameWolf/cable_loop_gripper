#!/usr/bin/env python3

import rospy
from cable_loop_gripper.msg import CLGcommand

def cli_length_control():

    rospy.init_node('length_input')
    pub = rospy.Publisher("/CLG_command", CLGcommand, queue_size=10)

    while not rospy.is_shutdown():

        msg = CLGcommand()
        length = float(input('input loop target lenght: '))

        msg.requested_loop_length = length
        msg.requested_force = 0
        msg.control_mode = True

        pub.publish(msg)

if __name__ == '__main__':
    cli_length_control()