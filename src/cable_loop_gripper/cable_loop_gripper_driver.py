import numpy as np
import rospy
from cable_loop_gripper.msg import CLGstatus, CLGcommand
from .cable_loop_gripper import CableLoopGripper as CLG
from .cable_loop_gripper_struct import CLG_status_struct as statusStruct

class CableLoopGripperDriver:
    def __init__(self, comport='/dev/serial0',baud=115200):
        
        self._comport = comport
        self._baud = baud

        self.gripper = CLG(self._comport, self._baud)

        self._status_publisher = rospy.Publisher("/CLG_status", CLGstatus, queue_size=10)
        self._command_subscriber = rospy.Subscriber("/CLG_command", CLGcommand, self.CLG_command_handler)

        self._last_length_command = 0
        self._last_force_command = 0
        self._last_control_command = 0

    def pubblish_gripper_status (self, state):
        msg = CLGstatus()

        msg.current_loop_length = state.current_loop_length
        msg.requested_loop_length = state.requested_loop_length
        msg.loop_length_at_setpoint = state.loop_length_at_setpoint
        msg.current_force = state.current_force
        msg.requested_force = state.requested_force
        msg.force_at_setpoint = state.force_at_setpoint

        return msg

    def CLG_command_handler(self, data):
        lengthcommand = data.requested_loop_length
        forceCommand = data.requested_force
        controlCommand = data.control_mode

        if controlCommand != self._last_control_command:
            self.gripper.setControlMode(controlCommand)
            self._last_control_command = controlCommand

        if lengthcommand != self._last_length_command:
            self.gripper.requestLoopLength(lengthcommand)
            self._last_length_command = lengthcommand

        if forceCommand != self._last_force_command:
            self.gripper.requestForce(forceCommand)
            self._last_force_command = forceCommand

    def updateDriver(self):
        status = self.gripper.getCLGstatus()
        cs = CLGstatus
        cs = self.pubblish_gripper_status(status)
        rospy.loginfo(cs)
        self._status_publisher.publish(cs)