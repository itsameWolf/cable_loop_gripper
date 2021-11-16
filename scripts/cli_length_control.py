#!/usr/bin/env python3

import rospy
from cable_loop_gripper.msg import CLGcommand

def cli_length_control():

    rospy.init_node('radius_input')
    pub = rospy.Publisher("/CLG_command", CLGcommand, queue_size=10)

    kill = False

    print("input the desired cable loop radius or press q to exit")

    while not rospy.is_shutdown() and not kill:

        msg = CLGcommand()

        user_in = input('input target loop radius: ')

        if (user_in == "q"):
            
            kill = True

        else:
            try:
                value = float(user_in)
                
                msg.requested_loop_radius = value
                msg.requested_force = 0
                msg.control_mode = True

                pub.publish(msg)
            except ValueError:
                print("invalid input")
        

if __name__ == '__main__':
    cli_length_control()
