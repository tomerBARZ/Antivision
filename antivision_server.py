import cv2
import numpy as np
from pupil_apriltags import Detector

from antivision_utils import *

cam = cv2.VideoCapture(-1)

video_res_fps = (640,480,210)
cam.set(cv2.CAP_PROP_FRAME_WIDTH, video_res_fps[0])
cam.set(cv2.CAP_PROP_FRAME_HEIGHT, video_res_fps[1])
cam.set(cv2.CAP_PROP_FPS, video_res_fps[2])

tag_detector = Detector("tag16h5",decode_sharpening=10.0)
marker_objp = Marker_OBJP(0.15)

X = np.load('Camera_Calib.npz')
mtx, dist, rvecs, tvecs = [X[i] for i in ('mtx','dist','rvecs','tvecs')]
new_mtx, roi = Calibration_GetOptimalNewCameraMTX(video_res_fps[0],video_res_fps[1],mtx,dist)

from networktables import NetworkTables
NetworkTables.initialize(server='192.168.4.1')
nt = NetworkTables.getTable('antivision')

nt.putBoolean("connected",True)

while cam.isOpened():
    try:
        ret, frame = cam.read()

        if(ret):
            results = FindAprilTags(tag_detector,frame)

            tagIds = []
            tagPitch = []
            tagYaw = []
            tagRoll = []
            relativeX = []
            relativeY = []
            relativeZ = []

            for tag in results:
                tag_transform = GetTagTransform(tag,marker_objp,new_mtx,dist)
                my_transform = GetCameraTransform(tag_transform)

                tagIds.append(tag.tag_id)
                tagPitch.append(tag_transform[1][0])
                tagYaw.append(tag_transform[1][1])
                tagRoll.append(tag_transform[1][2])
                relativeX.append(my_transform[0])
                relativeY.append(my_transform[1])
                relativeZ.append(my_transform[2])

            nt.putNumberArray("tag_ids",tagIds)
            
            nt.putNumberArray("tag_pitch",tagPitch)
            nt.putNumberArray("tag_yaw",tagYaw)
            nt.putNumberArray("tag_roll",tagRoll)

            nt.putNumberArray("tag_relative_x",relativeX)
            nt.putNumberArray("tag_relative_Y",relativeY)
            nt.putNumberArray("tag_relative_Z",relativeZ)
    except KeyboardInterrupt:
        break