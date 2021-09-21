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


        self._last_line = "0 - 0 - 0 - 0 - 0 - 0 - 0\n"


    def getCLGstatus(self):
        values = [0]*7

        if self.serial_interface.in_waiting > 0:
            try:
                line = self.serial_interface.readline().decode('ascii').rstrip()

            except UnicodeDecodeError:
                line =  self._last_line
            
            values = re.findall(r'\d+(?:\.\d+)?',line)
            if len(values) < 7:
                values = re.findall(r'\d+(?:\.\d+)?',self._last_line)
            else:   
                self._last_line = line 
            
        
        struct = status(float(values[0]),
                        float(values[1]),
                        bool(values[5]),
                        float(values[2]),
                        float(values[3]),
                        bool(values[6]))

        return struct
            

    def setControlMode (self, control_status): 
        msg = "S"+str(int(control_status))+"\n"
        self.serial_interface.write(msg.encode())

    def setLoopRadius (self, radius):
        msg = "O"+str(radius)+"\n"
        self.serial_interface.write(msg.encode())

    def requestLoopRadius (self, radius):
        msg = "P"+str(radius)+"\n"
        self.serial_interface.write(msg.encode())

    def requestForce (self, force):
        msg = "F"+str(force)+"\n"
        self.serial_interface.write(msg.encode())

    