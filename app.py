import cv2
import time
import json
import threading
import pygame
import serial
from ultralytics import YOLO

# Initialize Pygame mixer
pygame.mixer.init()

# Load deterrent sounds
cow_sound = pygame.mixer.Sound("animal_sounds/cow.mp3")
elephant_sound = pygame.mixer.Sound("animal_sounds/elephant.mp3")

# Load YOLO model
model = YOLO("yolov8n.pt")

# Connect to Arduino if available
try:
    arduino = serial.Serial('COM3', 9600, timeout=1)
    time.sleep(2)
    print("✅ Arduino connected")
    arduino_connected = True
except Exception as e:
    print(f"❌ Arduino not connected: {e}")
    arduino_connected = False

# Cooldowns
last_sound_time = 0
last_pump_time = 0
sound_cooldown = 3  # sec
pump_cooldown = 10  # sec

# Video input
VIDEO_PATH = "videos/cowvid.mp4"

def play_sound(animal):
    global last_sound_time
    now = time.time()
    if now - last_sound_time < sound_cooldown:
        print(f"🔇 Cooldown active. Sound skipped.")
        return
    last_sound_time = now

    print(f"🔊 Playing {animal} alert")
    if animal == "cow":
        cow_sound.play()
    elif animal == "elephant":
        elephant_sound.play()

def send_to_arduino(command, animal, distance):
    if not arduino_connected:
        print(f"[SIM] {command} for {animal} at {distance}cm")
        return
    try:
        data = {
            'command': command,
            'animal': animal,
            'distance': distance,
            'timestamp': int(time.time())
        }
        arduino.write((json.dumps(data) + '\n').encode())
        print(f"📤 Sent to Arduino: {command}")
        response = arduino.readline().decode().strip()
        if response:
            print(f"📥 Arduino: {response}")
    except Exception as e:
        print(f"❌ Serial error: {e}")

def activate_pump(animal, distance):
    global last_pump_time
    now = time.time()
    if now - last_pump_time < pump_cooldown:
        print("💧 Pump cooldown active")
        return
    last_pump_time = now
    send_to_arduino("PUMP_ON", animal, distance)
    print(f"💧 Pump activated for {animal}")

def draw_detections(frame, detections):
    for d in detections:
        x1, y1, x2, y2 = d['bbox']
        distance = d['distance']
        label = f"{d['name'].upper()} {distance:.1f}cm"

        color = (0, 255, 0)
        if distance < 100:
            color = (0, 165, 255)
        if distance < 50:
            color = (0, 0, 255)

        cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
        cv2.putText(frame, label, (x1, y1 - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
    return frame

def detect_animals():
    cap = cv2.VideoCapture(VIDEO_PATH)
    if not cap.isOpened():
        print("❌ Could not open video")
        return

    print("🚀 Detection started. Press Q to quit.")

    while True:
        ret, frame = cap.read()
        if not ret:
            cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
            continue

        results = model.predict(frame, conf=0.5, verbose=False)
        detections = []

        for r in results:
            if r.boxes is not None:
                for box in r.boxes:
                    cls = int(box.cls[0])
                    name = r.names[cls]
                    if name not in ["cow", "elephant"]:
                        continue
                    confidence = float(box.conf[0])
                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    h = y2 - y1
                    if h <= 0:
                        continue

                    focal_length = 800
                    real_height = 150 if name == "cow" else 300
                    distance = (real_height * focal_length) / h
                    distance = min(distance, 10000)

                    detections.append({
                        "name": name,
                        "confidence": confidence,
                        "bbox": [x1, y1, x2, y2],
                        "distance": distance
                    })

                    # Actions
                    if distance < 2000:
                        activate_pump(name, distance)
                        threading.Thread(target=play_sound, args=(name,), daemon=True).start()
                    elif distance < 5000:
                        threading.Thread(target=play_sound, args=(name,), daemon=True).start()

        frame = draw_detections(frame, detections)
        cv2.imshow("Animal Detection", frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()
    if arduino_connected:
        arduino.close()
        print("🔌 Arduino disconnected")

if __name__ == '__main__':
    detect_animals()
