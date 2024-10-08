import cv2
import numpy as np

def Calibration_Calibrate(capture,x_size = 9,y_size = 6):
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    objp = np.zeros((y_size*x_size,3), np.float32)
    objp[:,:2] = np.mgrid[0:x_size,0:y_size].T.reshape(-1,2)
    
    objpoints = [] # 3d point in real world space
    imgpoints = [] # 2d points in image plane.
    
    counter = 0
    drop_counter = 0
    while capture.isOpened():
        ret, frame = capture.read()

        if not ret:
            break

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        adjusted = cv2.convertScaleAbs(gray, alpha=1.5,)
        ret, corners = cv2.findChessboardCorners(adjusted, (x_size,y_size), None)
        
        drop_counter += 1
        print(ret)
        if ret == True and drop_counter >= 10:
            drop_counter = 0
            objpoints.append(objp)
            corners2 = cv2.cornerSubPix(gray,corners, (11,11), (-1,-1), criteria)
            imgpoints.append(corners2)

            cv2.drawChessboardCorners(frame, (x_size,y_size), corners2, ret)
            counter += 1
            print("FOUND",counter)
        cv2.imshow('img', frame)
        if cv2.waitKey(1) == ord('q'):
            break

    cv2.destroyAllWindows()
    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)
    np.savez('Camera_Calib.npz',ret=ret,mtx=mtx,dist=dist,rvecs=rvecs,tvecs=tvecs)
    mean_error = 0
    for i in range(len(objpoints)):
        imgpoints2, _ = cv2.projectPoints(objpoints[i], rvecs[i], tvecs[i], mtx, dist)
        error = cv2.norm(imgpoints[i], imgpoints2, cv2.NORM_L2)/len(imgpoints2)
        mean_error += error
    print( "total error: {}".format(mean_error/len(objpoints)) )


if __name__ == '__main__':
    cap = cv2.VideoCapture(2)

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    Calibration_Calibrate(cap)