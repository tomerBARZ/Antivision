import time
import threading

from flask import Flask, Response, jsonify, render_template, request
import cv2

from input.OV9281_input import OV9281
from input.DroidCam_input import DroidCam

import platform
system = platform.system()

from tinydb import TinyDB, Query
db = TinyDB('settings.json')

from misc.temp import get_cpu_temp

app = Flask(__name__)

cap = DroidCam() if system == "Windows" else OV9281(640,480)
capSettings = {setting.name: setting for setting in cap.getSettings()}

setting_callbacks = {}
settings_store = {}

if not cap.isOpen:
    print("CRITICAL: Failed to open camera!")
    exit()

latest_jpeg = None
frame_lock = threading.Lock()
frame_event = threading.Event()
capture_running = True

def capture_loop():
    global latest_jpeg

    while capture_running:
        frame = cap.getFrame()
        if frame is None:
            time.sleep(0.01)
            continue

        if(settings_store.get('orientation') == 'Flipped'):
            frame = cv2.rotate(frame, cv2.ROTATE_180)
        elif(settings_store.get('orientation') == 'Rotated Clockwise'):
            frame = cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE)
        elif(settings_store.get('orientation') == 'Rotated Counterclockwise'):
            frame = cv2.rotate(frame, cv2.ROTATE_90_COUNTERCLOCKWISE)

        ok, buffer = cv2.imencode('.jpg', frame)
        if not ok:
            continue

        with frame_lock:
            latest_jpeg = buffer.tobytes()

        frame_event.set()

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

def on_setting(name):
    def decorator(func):
        setting_callbacks[name] = func
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

@app.route('/')
def testPage():
    return render_template(
        "vision_config.html",
        inputSettings=capSettings.values(),
        selected_orientation=settings_store.get('orientation')
    )

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

if __name__ == '__main__':
    Setting = Query()
    savedSettings = db.all()
    print(savedSettings)
    for setting in savedSettings:
        settings_store[setting['name']] = setting['value']
        handle_setting_change(setting['name'], setting['value'])

    app.run(host='0.0.0.0', port=5000, threaded=True)