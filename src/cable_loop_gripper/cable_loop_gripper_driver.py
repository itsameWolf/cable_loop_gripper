import numpy as np
import rospy
from cable_loop_gripper.msg import CLGstatus, CLGcommand
from .cable_loop_gripper import CableLoopGripper as CLG

class CableLoopGripperDriver:
    def __init__(self, comport='/dev/serial0',baud=115200):
        
        self._comport = comport
        self._baud = baud

        self.gripper = CLG(self._comport, self._baud)

        self._pub = rospy.Publisher("/CLG_status", CLGstatus, queue_size=10)

    def pubblish_gripper_status (self, msg, state):
        msg.current_loop_length = state.current_loop_length
        msg.requested_loop_length = state.requested_loop_length
        msg.loop_length_at_setpoint = state.loop_length_at_setpoint
        msg.current_force = state.current_force
        msg.requested_force = state.requested_force
        msg.force_at_setpoint = state.force_at_setpoint
        return msg

    def updateDriver(self):
        status = self.gripper.getCLGstatus()
        #print(status.current_force)
        cs = CLGstatus
        print(cs.current_force)
        #cs = self.pubblish_gripper_status(cs,status)
        #print(cs)#.current_force)
        rospy.loginfo(cs)
        self._pub.publish(cs)