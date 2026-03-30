import cv2

from .videoInput import VideoInput

class OV9281(VideoInput):
    def __init__(self, width=1280, height=800, fps=30):
        gstreamer_pipeline = (
            "v4l2src device=/dev/video8 io-mode=2 ! "
            "video/x-raw,format=UYVY,width="+str(width)+",height="+str(height)+",framerate="+str(fps)+"/1 ! "
            "videoconvert ! "
            "video/x-raw,format=GRAY8 ! "
            "appsink drop=true sync=false"
        )

        self.cap = cv2.VideoCapture(gstreamer_pipeline, cv2.CAP_GSTREAMER)

    def getFrame(self):
        ret, frame = self.cap.read()
        return frame if ret else None

    def release(self):
        self.cap.release()

    def getSettings(self):
        return [
            
        ]

    def isOpen(self):
        return self.cap.isOpened()