from ultralytics import YOLO
import cv2
import threading
from flask import Flask, render_template, request
import pygame
import time

# Initialize pygame mixer once at the start
pygame.mixer.init()

# Load YOLO model
model = YOLO("yolov8n.pt")

# Video file path (local video instead of ESP32-CAM)
VIDEO_PATH = "videos/cowvid.mp4"

# Manual control flags
pump_on = False
sound_on = False

app = Flask(__name__)

@app.route('/')
def control_page():
    return render_template("control.html", pump=pump_on, sound=sound_on)

@app.route('/control', methods=['POST'])
def manual_control():
    global pump_on, sound_on
    if 'pump' in request.form:
        pump_on = not pump_on
    if 'sound' in request.form:
        sound_on = not sound_on
    return render_template("control.html", pump=pump_on, sound=sound_on)

# Play sound using pygame
def play_sound(file_path):
    try:
        pygame.mixer.music.load(file_path)
        pygame.mixer.music.play()
        while pygame.mixer.music.get_busy():
            time.sleep(0.1)
    except Exception as e:
        print(f"Error playing sound: {e}")

def detect_animals():
    global pump_on, sound_on
    cap = cv2.VideoCapture(VIDEO_PATH)
    while True:
        ret, frame = cap.read()
        if not ret:
            cap.set(cv2.CAP_PROP_POS_FRAMES, 0)  # Restart video
            continue

        results = model.predict(frame, conf=0.5)

        for r in results:
            for box in r.boxes:
                cls = int(box.cls[0])
                name = r.names[cls]
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                width = x2 - x1
                distance = 1000 / width if width > 0 else 9999

                if name in ["cow", "elephant"]:
                    print(f"[ALERT] {name} detected at - {distance:.2f} cm")

                    if distance < 10:
                        sound_file = None
                        if name == "cow":
                            sound_file = "animal_sounds/cow.mp3"
                        elif name == "elephant":
                            sound_file = "animal_sounds/elephant.mp3"
                        if sound_file:
                            threading.Thread(target=play_sound, args=(sound_file,)).start()
                        print("Pump activated due to animal proximity")
                        time.sleep(3)

                    if pump_on:
                        print("Manual Pump ON")

                    if sound_on:
                        threading.Thread(target=play_sound, args=("animal_sounds/cow.mp3",)).start()

# Start detection thread
detect_thread = threading.Thread(target=detect_animals)
detect_thread.daemon = True
detect_thread.start()

if __name__ == '__main__':
    app.run(host="0.0.0.0", port=5000)
