import struct
import serial
import re
from .cable_loop_gripper_struct import CLG_status_struct as status


class CableLoopGripper:
    def __init__ (self, comport='/dev/serial0',baud=115200):#,camera_id=0):
        
        self.serial_interface = serial.Serial(comport,baud,timeout=1)

        connected = self.serial_interface.isOpen()

        if not connected:
            raise Exception("Communication with gripper on serial port: %s and baud rate: %d not achieved" % (comport, baud))

        self.current_status = self.getCLGstatus()


    def getCLGstatus(self):
        values = [0]*7

        if self.serial_interface.in_waiting > 0:
            line = self.serial_interface.readline().decode('ascii').rstrip()
            values = re.findall(r'\d+(?:\.\d+)?',line)

        struct = status(float(values[0]),
                        float(values[1]),
                        bool(values[5]),
                        float(values[2]),
                        float(values[3]),
                        bool(values[6]))

        return struct
            

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