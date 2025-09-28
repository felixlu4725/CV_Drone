from flask import Flask, Response
from picamera2 import Picamera2
from ultralytics import YOLO
import cv2
import time

app = Flask(__name__)

# Initialize camera
picam2 = Picamera2()
picam2.preview_configuration.main.size = (800, 800)
picam2.preview_configuration.main.format = "RGB888"
picam2.preview_configuration.align()
picam2.configure("preview")
picam2.start()

# Load YOLO model
model = YOLO("yolov8n.pt")  # Very small, fast model that works well for general objects

def generate_frames():
    while True:
        # Capture a frame
        frame = picam2.capture_array()

        # YOLO inference
        start = time.time()
        results = model.predict(frame, imgsz=320, conf=0.25, verbose=False)
        end = time.time()

        # Annotate frame with bounding boxes
        annotated = results[0].plot(boxes=True, masks=False)

        # Add FPS overlay
        fps = 1 / (end - start)
        cv2.putText(annotated, f'FPS: {fps:.2f}', (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)

        # Encode to JPEG for browser streaming
        _, buffer = cv2.imencode('.jpg', annotated)
        frame_bytes = buffer.tobytes()

        # Yield frame to stream
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')

@app.route('/video')
def video():
    return Response(generate_frames(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/')
def index():
    return '<h1>YOLO Live Stream</h1><p><a href="/video">Click to view stream</a></p>'

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=8000)
