import cv2

class CableLoopGripper:
    def __init__(self, camera_id=0, width=640, height=640, fps=10):
        self._camera_id = camera_id
        self._frame_width = width
        self._frame_height = height
        self._fps = fps

        self.cap = cv2.VideoCapture(self._camera_id)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH,self._frame_width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT,self._frame_height)
        self.cap.set(cv2.CAP_PROP_FPS,self._fps)

        if self.cap is None or not self.cap.isOpened():
            raise Exception('Warning: unable to open video source: ', self._camera_id)

    def detectCable (self, frame, color_channel=2, blur_kernel=(5,5), canny_param_1=190,canny_param_2=30):
            blur = cv2.blur(frame[color_channel],blur_kernel)
            return cv2.HoughCircles(blur, cv2.HOUGH_GRADIENT,dp=1, minDist = 640, param1=canny_param_1, param2=canny_param_1, minRadius=80, maxRadius=1000)

    def detectVials (self, frame, color_channel=0, blur_kernel=(19,19), threshold_value=190,):
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