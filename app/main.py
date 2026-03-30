import time

from flask import Flask, Response, render_template
import cv2

from input.OV9281_input import OV9281
from input.DroidCam_input import DroidCam

app = Flask(__name__)

cap = DroidCam()

if not cap.isOpen:
    print("CRITICAL: Failed to open camera!")
    exit()

frame = cap.getFrame()

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

@app.route('/test')
def testPage():
    return render_template('index.html',settings=cap.getSettings())

@app.route('/')
def video_feed():
    """The route that serves the continuous video stream."""
    return Response(generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

if __name__ == '__main__':
    print("Starting video server... Go to http://<Orange_Pi_IP>:5000 in your browser.")
    # host='0.0.0.0' exposes the server to your entire local network, not just localhost
    app.run(host='0.0.0.0', port=5000, threaded=True)