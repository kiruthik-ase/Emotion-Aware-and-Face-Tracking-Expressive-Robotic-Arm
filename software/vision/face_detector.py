import cv2
import time
import torch
from ultralytics import YOLO

# ==============================
# CONFIG
# ==============================
VIDEO_SOURCE = 0  # 0 = webcam, change to 1 if using USB cam
MODEL_PATH = "yolov8n-face.pt"
CONF_THRESHOLD = 0.5
DEVICE = "cuda" if torch.cuda.is_available() else "cpu"

print("[INFO] Using device:", DEVICE)

# ==============================
# LOAD MODEL
# ==============================
face_detector = YOLO(MODEL_PATH).to(DEVICE)

cap = cv2.VideoCapture(VIDEO_SOURCE)

# Optional: set resolution
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

# OpenCV window (resizable)
WINDOW_NAME = "Face Detection | YOLOv8-FACE"
cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_AUTOSIZE)
cap.set(cv2.CAP_PROP_FRAME_WIDTH,  960)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)


prev_time = time.time()

# ==============================
# MAIN LOOP
# ==============================
while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        break
    frame = cv2.flip(frame, 1)  # horizontal flip (mirror correction)

    # ---------- FACE DETECTION ----------
    results = face_detector(
        frame,
        device=DEVICE,
        conf=CONF_THRESHOLD,
        verbose=False
    )

    if results and len(results[0].boxes) > 0:
        # pick most confident face
        box = results[0].boxes[0].xyxy[0].cpu().numpy().astype(int)
        x1, y1, x2, y2 = box

        # clamp to frame
        h, w, _ = frame.shape
        x1, y1 = max(0, x1), max(0, y1)
        x2, y2 = min(w, x2), min(h, y2)

        # draw face box
        cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
        cv2.putText(frame, "FACE",
                    (x1, y1 - 10),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.8,
                    (0, 255, 0),
                    2)

    # ---------- FPS ----------
    now = time.time()
    fps = int(1 / (now - prev_time))
    prev_time = now

    cv2.putText(frame,
                f"FPS: {fps}",
                (20, 40),
                cv2.FONT_HERSHEY_SIMPLEX,
                1,
                (255, 255, 0),
                2)

    cv2.imshow(WINDOW_NAME, frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
    
    if cv2.getWindowProperty(WINDOW_NAME, cv2.WND_PROP_VISIBLE) < 1:
        break

cap.release()
cv2.destroyAllWindows()
