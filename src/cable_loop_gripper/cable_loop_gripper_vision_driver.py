import rospy
from cable_loop_gripper.msg import CLGvision
from std_msgs.msg import String
from .cable_loop_gripper_vision import CableLoopGripperVision as CLG_V


class CableLoopGripperVisionDriver:
    def __init__(self, camera_id=0, width=640, height=640, fps=10):
        
        self._camera_id = camera_id
        self._frame_width = width
        self._frame_height = height
        self._fps = fps

        print("initialising camera...")
        self.camera = CLG_V(self._camera_id, self._frame_width, self._frame_height, self._fps)
        print("camerai initialised correctly")

        self._ret,self._frame = self.camera.cap.read()
        self._run_flag = False
        self.vision_results = []

    def runVisionRoutine(self, req):
        cable = self.camera.detectCable(self._frame)
        print(cable)
        vials = self.camera.detectVials(self._frame)
        print(vials)
        closest_vial_id = self.camera.findClosestVial(cable,vials)
        return self.camera.findXYdistanceAndRadius(cable,vials[0][closest_vial_id])

    def  runVision(self):
        self._ret ,self._frame = self.camera.cap.read()

            