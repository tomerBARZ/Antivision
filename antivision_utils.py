import cv2
import math
import numpy as np

def Marker_OBJP(real_marker_size):
    halfsize = real_marker_size/2
    TL = (-halfsize,-halfsize,0)
    TR = (halfsize,-halfsize,0)
    BL = (-halfsize,halfsize,0)
    BR = (halfsize,halfsize,0)

    return np.array([BL,BR,TR,TL])

def PythagoreanTheorem(a,b):
    return math.sqrt(a**2 + b**2)

def GetPointAtDistanceAndAngle(distance,degrees):
    x = distance * math.cos(math.radians(degrees))
    y = distance * math.sin(math.radians(degrees))
    return (x,y)

def FindAprilTags(detector, undistorted):
    gray = cv2.cvtColor(undistorted,cv2.COLOR_BGR2GRAY)
    results = detector.detect(gray)
    final_detections = []
    for tag in results:
        if(tag.tag_id <= 8 and tag.hamming <= 0):
            final_detections.append(tag)
    return final_detections

def GetPitchYawRoll(RotationMatrix):
    Pitch = math.atan2(RotationMatrix[2][1], RotationMatrix[2][2])
    Yaw = math.atan2(-RotationMatrix[2][0], math.sqrt(RotationMatrix[2][1] * RotationMatrix[2][1] + RotationMatrix[2][2] * RotationMatrix[2][2]))
    Roll = math.atan2(RotationMatrix[1][0], RotationMatrix[0][0])
    return math.degrees(Pitch), math.degrees(Yaw)+90, math.degrees(Roll)

def GetTagTransform(tag,objp,new_mtx,dist):
    ret, rvec,tvec = cv2.solvePnP(objp,tag.corners,new_mtx,dist)
    transform = ((-1,-1,-1),(-1,-1,-1))
    if(ret):
        x = tvec[0]
        y = tvec[2]
        z = tvec[1]

        rotation_matrix, _ = (cv2.Rodrigues(rvec))
        pitch, yaw, roll = GetPitchYawRoll(rotation_matrix)

        transform = ((x,y,z),(pitch,yaw,roll))
    return transform

def DrawCameraRelativeMap(frame,coords,multiplier=100):
    x = int(coords[0][0]*multiplier)
    y = int(coords[0][1]*multiplier)
    heading_angle = int(coords[1][1])
    map_target = (x+int(frame.shape[1]/2),int(frame.shape[0])-y)
    map = cv2.circle(np.zeros_like(frame,dtype=np.uint8),map_target,5,(0,0,255),-1)
    map = cv2.arrowedLine(map,(int(frame.shape[1]/2),int(frame.shape[0])),(int(frame.shape[1]/2),0),(255,0,0),2)
    target_heading = GetPointAtDistanceAndAngle(200,heading_angle)
    map = cv2.arrowedLine(map,map_target,(map_target[0]+int(target_heading[0]),map_target[1]+int(target_heading[1])),(0,255,0),3)
    return map

def GetCameraTransform(target_coords):
    target_x = target_coords[0][0]
    target_y = target_coords[0][1]
    target_yaw = target_coords[1][1]
    
    my_transform = (-1,-1,-1)
    distance_to_target = PythagoreanTheorem(target_x,target_y)
    if(distance_to_target > 0):
        if(int(target_x < 0)):
            gamma = math.degrees(math.asin(target_y/distance_to_target))
        else:
            gamma = 90 + (90-math.degrees(math.asin(target_y/distance_to_target)))
        camera_position = GetPointAtDistanceAndAngle(distance_to_target,gamma + 270-target_yaw)
        camera_position = (-(camera_position[0]),-(camera_position[1]))
        my_transform = (camera_position[0],camera_position[1],180-target_yaw)
    return my_transform

def Calibration_GetOptimalNewCameraMTX(width, height, mtx, dist):
    return cv2.getOptimalNewCameraMatrix(mtx, dist, (width,height), 1, (width,height))