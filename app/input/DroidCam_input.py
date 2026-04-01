import cv2

from .videoInput import VideoInput
from settings.Slider_setting  import Slider

class DroidCam(VideoInput):
    def __init__(self):
        super().__init__()
        self.cap = cv2.VideoCapture('http://192.168.1.30:4747/mjpegfeed')

    def getFrame(self):
        super().getFrame()
        ret, frame = self.cap.read()
        return frame if ret else None

    def release(self):
        self.cap.release()

    def getSettings(self):
        return [
            Slider("Exposure", "exposure", None, 16, 3468, 64),
            Slider("Gain", "gain", "This is a slider that will update the camera's analogue gain parameter", 1, 64, 4),
        ]

    def onSetting(self, name, value):
        print("DroidCam updating",name,"to",value)

    def isOpen(self):
        return self.cap.isOpened()