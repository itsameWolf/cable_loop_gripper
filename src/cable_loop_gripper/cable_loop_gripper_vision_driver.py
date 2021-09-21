from .cable_loop_gripper_vision import CableLoopGripperVision as CLG_V


class CableLoopGripperVisionDriver:
    def __init__(self, camera_id=0, width=640, height=640, fps=10):
        
        self._camera_id = camera_id
        self._frame_width = width
        self._frame_height = height
        self._fps = fps
        self._pixel_to_mm = 0.12

        print("initialising camera...")
        self.camera = CLG_V(self._camera_id, self._frame_width, self._frame_height, self._fps)
        print("camerai initialised correctly")

        self._ret,self._frame = self.camera.cap.read()
        self._run_flag = False
        self.vision_results = []

    def vialXYDistance(self, req):
        loop = self.camera.detectCable(self._frame)
        print(loop)
        vials = self.camera.detectVials(self._frame)
        print(vials)
        closest_vial_id = self.camera.findClosestVial(loop,vials)
        xy_distance_pixel = self.camera.findXYdistanceAndRadius(loop,vials[0][closest_vial_id])
        return [i*self._pixel_to_mm for i in xy_distance_pixel]

    def loopRadius(self,req):
        loop = self.camera.detectCable(self._frame)
        return loop[2]*self._pixel_to_mm
        
    def runVision(self):
        self._ret ,self._frame = self.camera.cap.read()

            