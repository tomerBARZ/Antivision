from flask import Flask, Response
import cv2

app = Flask(__name__)

# Your exact working GStreamer pipeline
gstreamer_pipeline = (
    "v4l2src device=/dev/video8 io-mode=2 ! "
    "video/x-raw,format=UYVY,width=1280,height=800,framerate=30/1 ! "
    "videoconvert ! "
    "video/x-raw,format=GRAY8 ! "
    "appsink drop=true sync=false"
)

# Open the camera globally so it doesn't initialize on every page refresh
cap = cv2.VideoCapture(gstreamer_pipeline, cv2.CAP_GSTREAMER)

if not cap.isOpened():
    print("CRITICAL: Failed to open camera!")
    exit()

def generate_frames():
    """Generator function that constantly reads frames and encodes them for the web."""
    while True:
        success, frame = cap.read()
        if not success:
            break
        
        # Encode the raw grayscale frame into a JPEG memory buffer
        ret, buffer = cv2.imencode('.jpg', frame)
        frame_bytes = buffer.tobytes()
        
        # Yield the frame in the standard multipart/x-mixed-replace MJPEG format
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')

@app.route('/')
def video_feed():
    """The route that serves the continuous video stream."""
    return Response(generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

if __name__ == '__main__':
    print("Starting video server... Go to http://<Orange_Pi_IP>:5000 in your browser.")
    # host='0.0.0.0' exposes the server to your entire local network, not just localhost
    app.run(host='0.0.0.0', port=5000, threaded=True)