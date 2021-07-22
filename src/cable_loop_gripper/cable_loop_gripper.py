from typing import TextIO
import serial
import re
from .cable_loop_gripper_struct import CLG_status_struct as status


class CableLoopGripper:
    def __init__ (self, comport='/dev/serial0',baud=115200):#,camera_id=0):
        
        self.serial_interface = serial.Serial(comport,baud,timeout=1)

        connected = self.serial_interface.isOpen()

        if not connected:
            raise Exception("Communication with gripper on serial port: %s and baud rate: %d not achieved" % (comport, baud))

        self.previous_status = status(0,0,True,0,0,False)
        self.current_status = self.getCLGstatus()


    def getCLGstatus(self):
        if self.serial_interface.in_waiting > 0:
            gripper_status = status
            line = self.serial_interface.readline().decode('ascii').rstrip()
            values = re.findall(r'\d+(?:\.\d+)?',line)
            if len(values) == 7:
                gripper_status.current_loop_length = float(values[0])
                gripper_status.requested_loop_length = float(values[1])
                gripper_status.loop_length_at_setpoint = bool(values[5])
                gripper_status.current_force = float(values[2])
                gripper_status.requested_force = float(values[3])
                gripper_status.force_at_setpoint = bool(values[6])
                return gripper_status
            else:
                return self.previous_status

    def setControlMode (self, control_status): 
        msg = "S"+str(control_status)+"\n"
        self.serial_interface.write(msg.encode())

    def setLoopLength (self, length):
        msg = "P"+str(length)+"\n"
        self.serial_interface.write(msg.encode())

    def requestLoopLength (self, length):
        msg = "P"+str(length)+"\n"
        self.serial_interface.write(msg.encode())

    def requestForce (self, force):
        msg = "P"+str(force)+"\n"
        self.serial_interface.write(msg.encode())