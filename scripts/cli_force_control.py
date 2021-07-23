#!/usr/bin/env python3

import rospy
from cable_loop_gripper.msg import CLGcommand

def cli_force_control():

    rospy.init_node('force_input')
    pub = rospy.Publisher("/CLG_command", CLGcommand, queue_size=10)

    kill = False

    while not rospy.is_shutdown() and not kill:

        msg = CLGcommand()

        user_in = input('input loop target lenght: ')

        if (user_in == "exit"):
            
            kill = True

        else:
            try:
                value = float(user_in)
                
                msg.requested_loop_length = 0
                msg.requested_force = value
                msg.control_mode = False

                pub.publish(msg)
            except ValueError:
                print("invalid input")
        

if __name__ == '__main__':
    cli_force_control()