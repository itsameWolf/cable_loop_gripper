import numpy as np
import rospy
from cable_loop_gripper.msg import CLGstatus, CLGcommand
from cable_loop_gripper.srv import CLG_offsetLoopRadius
from .cable_loop_gripper import CableLoopGripper as CLG
from .cable_loop_gripper_struct import CLG_status_struct as statusStruct

class CableLoopGripperDriver:
    def __init__(self, comport='/dev/serial0',baud=115200):
        
        self._comport = comport
        self._baud = baud

        self.gripper = CLG(self._comport, self._baud)

        self._status_publisher = rospy.Publisher("/CLG_status", CLGstatus, queue_size=10)
        self._command_subscriber = rospy.Subscriber("/CLG_command", CLGcommand, self.CLG_command_handler)
        self._offset_server = rospy.Service('CLG_set_radius_offset',CLG_offsetLoopRadius,self.radius_offset_callback)
        
        self._last_radius_command = 0
        self._last_force_command = 0
        self._last_control_command = 0

    def pubblish_gripper_status (self, state):
        msg = CLGstatus()

        msg.current_loop_radius = state.current_loop_radius
        msg.requested_loop_radius = state.requested_loop_radius
        msg.loop_radius_at_setpoint = state.loop_radius_at_setpoint
        msg.current_force = state.current_force
        msg.requested_force = state.requested_force
        msg.force_at_setpoint = state.force_at_setpoint

        return msg

    def CLG_command_handler(self, data):
        radiuscommand = data.requested_loop_radius
        forceCommand = data.requested_force
        controlCommand = data.control_mode
        rospy.loginfo(data)

        if controlCommand != self._last_control_command:
            self.gripper.setControlMode(controlCommand)
            self._last_control_command = controlCommand

        if radiuscommand != self._last_radius_command:
            self.gripper.requestLoopRadius(radiuscommand)
            self._last_radius_command = radiuscommand

        if forceCommand != self._last_force_command:
            self.gripper.requestForce(forceCommand)
            self._last_force_command = forceCommand

    def radius_offset_callback(self, req):
        self.gripper.setLoopRadius(req.request_offset)
        return True

    def updateDriver(self):
        status = self.gripper.getCLGstatus()
        cs = CLGstatus
        cs = self.pubblish_gripper_status(status)
        rospy.loginfo(cs)
        self._status_publisher.publish(cs)