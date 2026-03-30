import time

from flask import Flask, Response, jsonify, render_template, request
import cv2

from input.OV9281_input import OV9281
from input.DroidCam_input import DroidCam

app = Flask(__name__)

cap = DroidCam()

setting_callbacks = {}
settings_store = {}

if not cap.isOpen:
    print("CRITICAL: Failed to open camera!")
    exit()

def generate_frames():
    """Generator function that constantly reads frames and encodes them for the web."""
    while True:
        frame = cap.getFrame()
        if frame is None:
            break
        
        frame = cv2.circle(frame,(500,250), 50, 200, 2)

        # Encode the raw grayscale frame into a JPEG memory buffer
        ret, buffer = cv2.imencode('.jpg', frame)
        frame_bytes = buffer.tobytes()
        
        # Yield the frame in the standard multipart/x-mixed-replace MJPEG format
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')


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

def handle_setting_change(name, value):
    callback = setting_callbacks.get(name)
    if callback:
        callback(value)
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
        pipelines=["Pipeline_Name", "AprilTags", "Retroreflective"],
        active_pipeline=0
    )

@app.route("/api/stats")
def api_stats():
    return {
        "fps": 69.5,
        "temperature": 42.1,
        "latency": 12.9
    }

@app.route("/video_feed")
def video_feed():
    """The route that serves the continuous video stream."""
    return Response(generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

if __name__ == '__main__':
    print("Starting video server... Go to http://<Orange_Pi_IP>:5000 in your browser.")
    # host='0.0.0.0' exposes the server to your entire local network, not just localhost
    app.run(host='0.0.0.0', port=5000, threaded=True)