import cv2
import math

class CableLoopGripperVision:
    def __init__(self, camera_id=0, width=640, height=640, fps=10):

        self.cap = cv2.VideoCapture(camera_id)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH,width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT,height)
        self.cap.set(cv2.CAP_PROP_FPS,fps)

        if self.cap is None or not self.cap.isOpened():
            raise Exception('Warning: unable to open video source: ', self._camera_id)


    def detectCable (self, frame, color_channel=2, blur_kernel=(3,3), canny_param_1=100,canny_param_2=20):
        blur = cv2.blur(frame[:,:,color_channel],blur_kernel)
        loop = cv2.HoughCircles(blur, cv2.HOUGH_GRADIENT,dp=1, minDist = 640, param1=canny_param_1, param2=canny_param_2, minRadius=80, maxRadius=1000)
        return loop[0,0,:]

    def detectVials (self, frame, color_channel=0, blur_kernel=(19,19), threshold_value=170, method=1, canny_param_1=90,canny_param_2=25):
        blur = cv2.blur(frame[:,:,color_channel],blur_kernel)
        _, threshold = cv2.threshold(blur, threshold_value, 255, cv2.THRESH_BINARY)
        if method:
            circles = cv2.HoughCircles(blur, cv2.HOUGH_GRADIENT,dp=1, minDist = 20, param1=canny_param_1, param2=canny_param_2, minRadius=20, maxRadius=500)
            vials = circles[0,:,:]
            centers = vials[:,0:2]
            radiuses = vials[:,2]
        else:
            _, contours, _ = cv2.findContours(threshold, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            centers = []
            radiuses = []
            for contour in contours:
                approx = cv2.approxPolyDP(contour,0.01*cv2.arcLength(contour,True),True)
                area = cv2.contourArea(contour)
                if ((len(approx)>8) & (len(approx)<18)&(area>300)):
                    centre, radius = cv2.minEnclosingCircle(contour)
                    centers.append(centre)
                    radiuses.append(radius)
        return centers,radiuses

    def findClosestVial (self, cable, vials):
        mindist = 0.0
        mindist_index = 0
        x_c = cable[0]
        y_c = cable[1]
        for index, vial in enumerate(vials):
            x_v = vial[0]
            y_v = vial[1]
            dist = math.sqrt((x_v-x_c)**2+(y_v-y_c)**2)
            if index == 0:
                mindist = dist
            elif dist < mindist:
                mindist = dist
                mindist_index = index
        return mindist_index

    def findXYdistance (self, cable, vial):
        return cable[0]-vial[0], cable[1]-vial[1]
