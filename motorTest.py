# Animal Detection System - Updated with Pump Control

import cv2
import time
import json
import threading
import pygame
import serial
from ultralytics import YOLO

# Initialize Pygame mixer
pygame.mixer.init()

# Load deterrent animal sounds
cow_sound = pygame.mixer.Sound("animal_sounds/cow.mp3")
elephant_sound = pygame.mixer.Sound("animal_sounds/elephant.mp3")

# Load YOLO model
model = YOLO("yolov8l.pt")

# Connect to Arduino
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
sound_cooldown = 3  # seconds
pump_cooldown = 15  # seconds (increased to account for pump auto-shutoff + cooldown)

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
        
        # Read response with timeout
        start_time = time.time()
        response = ""
        while time.time() - start_time < 0.5:  # 500ms timeout
            if arduino.in_waiting > 0:
                response = arduino.readline().decode().strip()
                break
        
        if response:
            print(f"📥 Arduino: {response}")
    except Exception as e:
        print(f"❌ Serial error: {e}")

def activate_pump(animal, distance):
    global last_pump_time
    now = time.time()
    if now - last_pump_time < pump_cooldown:
        remaining = int(pump_cooldown - (now - last_pump_time))
        print(f"💧 Pump cooldown active ({remaining}s remaining)")
        return
    
    last_pump_time = now
    send_to_arduino("PUMP_ON", animal, distance)
    print(f"💧 Pump activated for {animal} at {distance:.1f}cm")

def draw_detections(frame, detections):
    """Draw bounding boxes and info on detected animals"""
    for d in detections:
        x1, y1, x2, y2 = d['bbox']
        distance = d['distance']
        confidence = d['confidence']
        label = f"{d['name'].upper()} {distance:.1f}cm ({confidence:.2f})"

        # Color coding based on distance
        color = (0, 255, 0)      # Green - Far
        if distance < 500:       # Medium distance
            color = (0, 165, 255) # Orange
        if distance < 200:       # Close - pump activation range
            color = (0, 0, 255)   # Red

        # Draw bounding box
        cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
        
        # Draw label background
        label_size = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)[0]
        cv2.rectangle(frame, (x1, y1 - label_size[1] - 10), 
                     (x1 + label_size[0], y1), color, -1)
        
        # Draw label text
        cv2.putText(frame, label, (x1, y1 - 5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
    
    return frame

def draw_status_info(frame, pump_cooldown_remaining):
    """Draw system status on frame"""
    height, width = frame.shape[:2]
    
    # Status background
    cv2.rectangle(frame, (10, 10), (300, 100), (0, 0, 0), -1)
    cv2.rectangle(frame, (10, 10), (300, 100), (255, 255, 255), 2)
    
    # Status text
    status_texts = [
        f"System: {'ONLINE' if arduino_connected else 'OFFLINE'}",
        f"Pump Cooldown: {pump_cooldown_remaining}s" if pump_cooldown_remaining > 0 else "Pump: READY",
        "Press 'q' to quit"
    ]
    
    for i, text in enumerate(status_texts):
        color = (0, 255, 0) if arduino_connected or i == 2 else (0, 0, 255)
        if i == 1 and pump_cooldown_remaining > 0:
            color = (0, 165, 255)
        cv2.putText(frame, text, (15, 30 + i * 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
    
    return frame

def detect_animals():
    cap = cv2.VideoCapture(VIDEO_PATH)
    if not cap.isOpened():
        print("❌ Could not open video")
        return

    print("🚀 Detection started. Press Q to quit.")
    print("💧 Pump will activate for animals closer than 200cm")
    print("🔊 Sound alerts for all detected animals")

    frame_count = 0
    last_status_time = time.time()

    while True:
        ret, frame = cap.read()
        if not ret:
            # Loop video if using file
            cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
            continue

        frame_count += 1
        current_time = time.time()
        
        # Process every frame for detection
        results = model.predict(frame, conf=0.5, verbose=False)
        detections = []
        animals_detected = False

        for r in results:
            if r.boxes is not None:
                for box in r.boxes:
                    cls = int(box.cls[0])
                    name = r.names[cls]
                    
                    # Only process target animals
                    if name not in ["cow", "elephant"]:
                        continue
                    
                    animals_detected = True
                    confidence = float(box.conf[0])
                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    h = y2 - y1
                    
                    if h <= 0:
                        continue

                    # Calculate distance (improved estimation)
                    focal_length = 800
                    real_height = 150 if name == "cow" else 300  # cm
                    distance = (real_height * focal_length) / h
                    distance = min(distance, 10000)  # Cap max distance

                    detections.append({
                        "name": name,
                        "confidence": confidence,
                        "bbox": [x1, y1, x2, y2],
                        "distance": distance
                    })

                    # Immediate actions for detected animals
                    print(f"🎯 {name.upper()} detected at {distance:.1f}cm (conf: {confidence:.2f})")
                    
                    # Send RED status to Arduino
                    send_to_arduino("RED", name, distance)
                    
                    # Play deterrent sound
                    threading.Thread(target=play_sound, args=(name,), daemon=True).start()
                    
                    # Activate pump if animal is close enough
                    if distance < 2000:  # Reduced threshold for better effectiveness
                        activate_pump(name, distance)

        # Send GREEN status if no animals detected
        if not animals_detected:
            # Only send GREEN status every 2 seconds to reduce serial traffic
            if current_time - last_status_time >= 2:
                send_to_arduino("GREEN", "none", 9999)
                last_status_time = current_time

        # Calculate pump cooldown remaining
        pump_cooldown_remaining = max(0, int(pump_cooldown - (current_time - last_pump_time)))

        # Draw all visual elements
        frame = draw_detections(frame, detections)
        frame = draw_status_info(frame, pump_cooldown_remaining)

        # Display frame
        cv2.imshow("Animal Deterrent System", frame)

        # Handle quit
        if cv2.waitKey(1) & 0xFF == ord('q'):
            print("🛑 Shutting down system...")
            break

    # Cleanup
    cap.release()
    cv2.destroyAllWindows()
    
    if arduino_connected:
        # Send final GREEN status and close connection
        send_to_arduino("GREEN", "none", 9999)
        time.sleep(0.5)
        arduino.close()
        print("🔌 Arduino disconnected")
    
    print("✅ System shutdown complete")

if __name__ == '__main__':
    detect_animals()