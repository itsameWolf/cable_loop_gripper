from .cable_loop_gripper_vision import CableLoopGripperVision as CLG_V


class CableLoopGripperVisionDriver:
    def __init__(self, camera_id=0, width=640, height=640, fps=20):
        
        self._camera_id = camera_id
        self._frame_width = width
        self._frame_height = height
        self._fps = fps
        self._pixel_to_mm = 0.12
        self._face_position = [490,330]

        print("initialising camera...")
        self.camera = CLG_V(self._camera_id, self._frame_width, self._frame_height, self._fps)
        print("camera initialised correctly")

        self._ret,self._frame = self.camera.cap.read()
        self._run_flag = False
        self.vision_results = []

    def vialXYDistance(self, req):
        loop = self.camera.detectCable(self._frame)
        centers, radiuses = self.camera.detectVials(self._frame)
        closest_vial_id = self.camera.findClosestVial(loop,centers)
        xy_distance_pixel = self.camera.findXYdistance(loop,centers[closest_vial_id]) + (radiuses[closest_vial_id],)
        return ([i*self._pixel_to_mm for i in xy_distance_pixel])

    def vialXYDistanceFromFace(self,req):
        loop = self.camera.detectCable(self._frame)
        centers, radiuses = self.camera.detectVials(self._frame,method=0)
        closest_vial_id = self.camera.findClosestVial(self._face_position ,centers)
        xy_distance_pixel = self.camera.findXYdistance(self._face_position ,centers[closest_vial_id]) + (radiuses[closest_vial_id],)
        return ([i*self._pixel_to_mm for i in xy_distance_pixel])

    def loopRadius(self,req):
        loop = self.camera.detectCable(self._frame)
        return loop[2]*self._pixel_to_mm
        
    def runVision(self):
        self._ret ,self._frame = self.camera.cap.read()

            
