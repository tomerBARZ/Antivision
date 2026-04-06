import time
import threading

from flask import Flask, Response, jsonify, render_template, request
import cv2
import numpy as np

from pathlib import Path

from input.OV9281_input import OV9281
from input.DroidCam_input import DroidCam

import platform
system = platform.system()

from tinydb import TinyDB, Query
db = TinyDB('settings.json')

from calibration.calibration import detect_charuco, generate_charuco_board, generate_heatmap
from misc.temp import get_cpu_temp

app = Flask(__name__)

cap = DroidCam() if system == "Windows" else OV9281(640,480)
capSettings = {setting.name: setting for setting in cap.getSettings()}

calibrationMap = None
calibration = {}
new_camera_matrix = None

setting_callbacks = {}
settings_store = {}
button_callbacks = {}

if not cap.isOpen:
    print("CRITICAL: Failed to open camera!")
    exit()

frame = None
latest_frame = None
frame_lock = threading.Lock()
frame_event = threading.Event()
capture_running = True

def capture_loop():
    global frame, latest_jpeg

    while capture_running:
        raw_frame = cap.getFrame()
        if raw_frame is None:
            time.sleep(0.01)
            continue

        orientation = settings_store.get('orientation')
        
        if orientation == 'Flipped':
            frame = cv2.rotate(raw_frame, cv2.ROTATE_180)
        elif orientation == 'Rotated Clockwise':
            frame = cv2.rotate(raw_frame, cv2.ROTATE_90_CLOCKWISE)
        elif orientation == 'Rotated Counterclockwise':
            frame = cv2.rotate(raw_frame, cv2.ROTATE_90_COUNTERCLOCKWISE)
        else:
            frame = raw_frame


        undistortion = settings_store.get('undistortion')
        if(undistortion == "Active"):
            if(len(calibration) > 0):
                frame = cv2.undistort(frame, calibration["camera_matrix"], calibration['dist_coeffs'], None, new_camera_matrix)

        ok, buffer = cv2.imencode('.jpg', frame)
        if not ok:
            continue

        with frame_lock:
            latest_jpeg = buffer.tobytes()

        frame_event.set()

        time.sleep(0.01)  # Control frame capture rate

capture_thread = threading.Thread(target=capture_loop, daemon=True)
capture_thread.start()

def generate_frames():
    try:
        while True:
            frame_event.wait(timeout=1.0)

            with frame_lock:
                if latest_jpeg is None:
                    continue
                frame_bytes = latest_jpeg

            yield (
                b'--frame\r\n'
                b'Content-Type: image/jpeg\r\n'
                + f'Content-Length: {len(frame_bytes)}\r\n\r\n'.encode()
                + frame_bytes
                + b'\r\n'
            )

            time.sleep(0.01)

    except GeneratorExit:
        return
    except Exception as e:
        print("Stream client disconnected or errored:", e)
        return

def getBoard():
    settings = get_aruco_settings()
    return generate_charuco_board(settings[0],settings[1],settings[2])

def generate_calibration_frames():
    global frame, calibrationMap

    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)

    try:
        while settings_store.get('calibrating'):
            frame_event.wait(timeout=1.0)
            
            with frame_lock:
                vis = frame.copy()

            board = settings_store.get('aruco_board',getBoard()[1])
            charuco_corners, charuco_ids, marker_corners, marker_ids = detect_charuco(
                vis, board, dictionary
            )

            if marker_ids is None or len(marker_ids) <= 0:
                continue
            


            if len(frame.shape) <= 2:
                vis = cv2.cvtColor(vis, cv2.COLOR_GRAY2BGR)

            cv2.aruco.drawDetectedMarkers(vis, marker_corners, None)
            if(calibrationMap is not None):
                vis = cv2.hconcat([vis, calibrationMap])

            _, buffer = cv2.imencode('.jpg', vis)
            latest_calib = buffer.tobytes()
            if latest_calib is None:
                continue
            frame_bytes = latest_calib

            if(settings_store.get('calibration_capture',False)):
                print("CAPTURING")
                if charuco_corners is not None and len(charuco_ids) >= 8:
                    print("HAS CORNERS")
                    settings_store['calibration_all_corners'].append(charuco_corners)
                    settings_store['calibration_all_ids'].append(charuco_ids)
                    
                    x,y,_ = frame.shape
                    
                    h = (generate_heatmap((x,y), settings_store['calibration_all_corners']) / 2).astype(np.uint8)
                    h = cv2.blur(h,(10,10))
                    s = np.full_like(h, 200)
                    v = np.full_like(h, 200)

                    hsv_image = cv2.merge([h, s, v])
                    calibrationMap = cv2.cvtColor(hsv_image, cv2.COLOR_HSV2BGR)
                else:
                    print(charuco_corners, charuco_ids)

                settings_store['calibration_capture'] = False

            yield (
                b'--frame\r\n'
                b'Content-Type: image/jpeg\r\n'
                + f'Content-Length: {len(frame_bytes)}\r\n\r\n'.encode()
                + frame_bytes
                + b'\r\n'
            )
            
            time.sleep(0.01)

    except GeneratorExit:
        return
    except Exception as e:
        print("Stream client disconnected or errored:", e)
        return

def get_aruco_settings():
    return (
        int(settings_store.get("charuco_x",11)),
        int(settings_store.get("charuco_y",8)),
        float(settings_store.get("charuco_mm",40)/100.0),
        bool(settings_store.get("calibrating",False))
        )

def get_new_matrix():
    global new_camera_matrix
    h, w = frame.shape[:2]
    new_camera_matrix, _ = cv2.getOptimalNewCameraMatrix(calibration["camera_matrix"], calibration['dist_coeffs'], (w, h), 1, (w, h))

def save_calibration():    
    np.savez('calibration.npz',
            camera_matrix = calibration["camera_matrix"],
            dist_coeffs = calibration["dist_coeffs"],
            rvecs = calibration["rvecs"],
            tvecs = calibration["tvecs"])
    get_new_matrix()

def load_calibration():
    global calibration
    file_path = Path('calibration.npz')

    if file_path.is_file():
        data = np.load(file_path)
        calibration = {"camera_matrix": data["camera_matrix"], "dist_coeffs":data["dist_coeffs"], "rvecs":data["rvecs"], "tvecs": data["tvecs"]}
        get_new_matrix()    

def generate_calib_image():
    _, buffer = cv2.imencode('.jpg', getBoard()[0])
    frame_bytes = buffer.tobytes()

    yield (
        b'--frame\r\n'
        b'Content-Type: image/jpeg\r\n'
        + f'Content-Length: {len(frame_bytes)}\r\n\r\n'.encode()
        + frame_bytes
        + b'\r\n'
    )

def on_setting(name):
    def decorator(func):
        setting_callbacks[name] = func
        return func
    return decorator

def on_button(name):
    def decorator(func):
        button_callbacks[name] = func
        return func
    return decorator

@on_setting("marker_size")
def update_marker_size(value):
    print("marker_size callback:", value)

@on_setting("detector_downscale")
def update_downscale(value):
    print("detector_downscale callback:", value)

@on_setting("orientation")
def update_orientation(value):
    pass

@on_setting("undistortion")
def update_undistortion(value):
    pass

def handle_setting_change(name, value):
    Setting = Query()
    db.upsert(({'name': name, 'value': value}), Setting.name == name)
    callback = setting_callbacks.get(name)
    if callback:
        callback(value)
    elif name in capSettings:
        cap.onSetting(capSettings[name].id, value)
        capSettings[name].value = value
    else:
        print(f"No callback registered for {name}, value={value}")

@app.post("/api/settings")
def api_settings():
    data = request.get_json()
    name = data.get("name")
    value = data.get("value")

    if not name:
        return jsonify(ok=False, error="Missing setting name"), 400

    settings_store[name] = value
    handle_setting_change(name, value)

    return jsonify(ok=True, name=name, value=value)

def handle_button_press(name, text):
    callback = button_callbacks.get(name)
    if callback:
        callback(text)
    else:
        print(f"Button pressed: name={name}, text={text}")

@app.post("/api/button-press")
def api_button_press():
    data = request.get_json()
    name = data.get("name")
    text = data.get("text", "")

    if not name:
        return jsonify(ok=False, error="Missing button name"), 400

    handle_button_press(name, text)

    return jsonify(ok=True, name=name, text=text)

@app.route("/api/stats")
def api_stats():
    return {
        "fps": cap.fps,
        "temperature": get_cpu_temp(),
    }

@app.route("/video_feed")
def video_feed():
    return Response(
        generate_frames(),
        content_type='multipart/x-mixed-replace; boundary=frame',
        headers={
            'Cache-Control': 'no-cache',
            'Pragma': 'no-cache'
        }
    )

@app.route("/calibration_feed")
def calibration_feed():
    return Response(
        generate_calibration_frames(),
        content_type='multipart/x-mixed-replace; boundary=frame',
        headers={
            'Cache-Control': 'no-cache',
            'Pragma': 'no-cache'
        }
    )

@app.route("/calib_image.jpg")
def calibartion_image():
    return Response(
        generate_calib_image(),
        content_type='multipart/x-mixed-replace; boundary=frame',
        headers={
            'Cache-Control': 'no-cache',
            'Pragma': 'no-cache'
        }
    )

@on_button("calibration")
def handle_calibration(value):
    global calibration

    if(value == "Capture Detections"):
        settings_store['calibration_capture'] = True
    else:
        if(value == "Begin Calibration"):
            settings_store['calibrating'] = True
            settings_store['aruco_board'] = getBoard()[1]
            calibration = {}
        else:
            if(value == "Finish"):
                if(len(settings_store.get('calibration_all_corners',[[]])) > 0):
                    min_corners = 4

                    filtered = [
                        (c.reshape(-1, 1, 2), i.reshape(-1, 1))
                        for c, i in zip(settings_store.get('calibration_all_corners'), settings_store.get('calibration_all_ids'))
                        if len(c) >= min_corners
                    ]

                    if len(filtered) == 0:
                        raise ValueError("No frames with enough corners for calibration!")

                    charuco_corners_fixed, charuco_ids_fixed = zip(*filtered)

                    ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.aruco.calibrateCameraCharuco(
                            charucoCorners=charuco_corners_fixed,
                            charucoIds=charuco_ids_fixed,
                            board=settings_store.get('aruco_board'),
                            imageSize=  (frame if frame.shape[2] == 1 else cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)).shape[::-1],
                            cameraMatrix=None,
                            distCoeffs=None
                        )
                    if(ret):
                        calibration["camera_matrix"] = camera_matrix
                        calibration["dist_coeffs"] = dist_coeffs
                        calibration["rvecs"] = rvecs
                        calibration["tvecs"] = tvecs
                        save_calibration()
                    else:
                        print("Calibration failed")                
                    settings_store['calibrating'] = False
                else:
                    print("NOT ENOUGH CORNERS:",len(settings_store.get('calibration_all_corners',[[]])),settings_store.get('calibration_all_corners',[[]]))
            else:            
                settings_store['calibrating'] = False
        settings_store['calibration_all_corners'] = []
        settings_store['calibration_all_ids'] = []

@app.route('/')
def index():
    return render_template(
        "vision_config.html",
        inputSettings=capSettings.values(),
        selected_orientation=settings_store.get('orientation'),
        selected_undistortion=settings_store.get('undistortion'),
        aruco_settings = get_aruco_settings(),
        isCalibrated=len(calibration) > 0
    )

if __name__ == '__main__':
    Setting = Query()
    savedSettings = db.all()
    print(savedSettings)
    for setting in savedSettings:
        settings_store[setting['name']] = setting['value']
        handle_setting_change(setting['name'], setting['value'])
    load_calibration()

    app.run(host='0.0.0.0', port=5000, threaded=True)