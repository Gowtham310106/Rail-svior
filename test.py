from ultralytics import YOLO
import cv2

# Load model
model = YOLO("yolov8n.pt")

# Run prediction
results = model.predict("sample.jpg")

# Display the image with OpenCV
for r in results:
    img = r.plot()  # this gives you the image with bounding boxes
    cv2.imshow("Prediction", img)
    cv2.waitKey(0)  # waits until you press a key
    cv2.destroyAllWindows()
