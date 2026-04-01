import cv2
import subprocess

from settings.Slider_setting import Slider

from .videoInput import VideoInput

def v4l2_ctl(device, *args):
    cmd = ["v4l2-ctl", f"--device={device}"] + list(args)
    result = subprocess.run(cmd, capture_output=True, text=True)
    if result.returncode != 0:
        raise RuntimeError(f"v4l2-ctl error: {result.stderr}")
    return result.stdout

DEVICE = "/dev/video8"

class OV9281(VideoInput):
    def __init__(self, width=1280, height=800, fps=120):
        super().__init__()
        gstreamer_pipeline = (
            "v4l2src device=/dev/video8 io-mode=2 ! "
            f"video/x-raw,format=UYVY,width={width},height={height},framerate={fps}/1 ! "
            "videoconvert ! "
            f"video/x-raw,format=GRAY8,framerate={fps}/1 ! "
            "appsink max-buffers=1 drop=true sync=false async=false"
        )

        self.cap = cv2.VideoCapture(gstreamer_pipeline, cv2.CAP_GSTREAMER)

    def getFrame(self):
        super().getFrame()
        ret, frame = self.cap.read()
        return frame if ret else None

    def release(self):
        self.cap.release()

    def getSettings(self):
        return [
            Slider("Brightness","brightness", "Doesn't really do much on this camera", 1, 228, 200),
            Slider("Exposure", "exposure","Exposure of the camera, Longer exposure is brighter but blurrier", 4, 3652, 800),
            Slider("Analogue Gain", "analogue_gain", "Gain of the video, makes everything brighter", 16, 248, 16)
        ]

    def onSetting(self, name, value):
        v4l2_ctl(DEVICE, "--set-ctrl="+str(name)+"="+str(value))

    def isOpen(self):
        return self.cap.isOpened()