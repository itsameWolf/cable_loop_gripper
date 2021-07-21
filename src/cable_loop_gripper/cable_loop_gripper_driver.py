import cv2
import numpy as np
import serial
import re
from serial.serialutil import Timeout
from .cable_loop_gripper_struct import CLG_status_struct as status

class CableLoopGripper:
    def __init__(self, comport='/dev/serial0',baud=57600,camera_id=0):

        self.CLG_status = status()
        
        self.serial_interface = serial.Serial(comport,baud,timeout=1)

        connected = self.serial_interface.isOpen()
        
        if not connected:
            raise Exception("Communication with gripper on serial port: %s and baud rate: %d not achieved" % (comport, baud))

        self.cap = cv2.VideoCapture(camera_id)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH,640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT,640)
        self.cap.set(cv2.CAP_PROP_FPS,10)

        if self.cap is None or not self.cap.isOpened():
            raise Exception('Warning: unable to open video source: ', camera_id)

        
    def getCLGstatus(self):
        if self.serial_interface.in_waiting > 0:
            line = self.serial_interface.readline().decode('ascii').rstrip()
            values = re.findall(r'\d+(?:\.\d+)?',line)
            self.CLG_status.current_loop_length = values[0]
            self.CLG_status.requested_loop_length = values[1]
            self.CLG_status.current_force = values[2]
            self.CLG_status.requested_force = values[3]
            self.CLG_status.at_setpoint = values[5]
    
    def setControlStatus (self, control_status): 
        msg = "S"+str(control_status)+"\n"
        self.serial_interface.write(msg)

    def setLoopLength (self, length):
        msg = "P"+str(length)+"\n"
        self.serial_interface.write(msg)

    def requestLoopLength (self, length):
        msg = "P"+str(length)+"\n"
        self.serial_interface.write(msg)

    def requestForce (self, force):
        msg = "P"+str(force)+"\n"
        self.serial_interface.write(msg)

    def detectCable(frame, color_channel=2, blur_kernel=(5,5), canny_param_1=190,canny_param_2=30):
        blur = cv2.blur(frame[color_channel],blur_kernel)
        return cv2.HoughCircles(blur, cv2.HOUGH_GRADIENT,dp=1, minDist = 640, param1=canny_param_1, param2=canny_param_1, minRadius=80, maxRadius=1000)

    def detectVials(frame, color_channel=0, blur_kernel=(19,19), threshold_value=190,):
        blur = cv2.blur(frame[color_channel],blur_kernel)
        _, threshold = cv2.threshold(blur, threshold_value, 255, cv2.THRESH_TOZERO)
        _, contours, _ = cv2.findContours(threshold, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        centers = [None]*len(contours)
        radiuses = [None]*len(contours)
        for contour in contours:
            approx = cv2.approxPolyDP(contour,0.01*cv2.arcLength(contour,True),True)
            area = cv2.contourArea(contour)
            if ((len(approx)>8) & (len(approx)<18)&(area>300)):
                centre, radius = cv2.minEnclosingCircle(contour)
                centers.append(centre)
                radiuses.append(radius)
        return centers,radiuses

