import cv2
import time
import torch
import serial
from ultralytics import YOLO

# -------------------------------
# IMPORT YOUR MODULES
# -------------------------------
from vision.emotion_detector import EmotionDetector
from perception.filters import EmotionFilter
from emotion_engine.derived import DerivedEmotionEngine
from emotion_engine.state_machine import EmotionStateMachine

# ==============================
# CONFIG
# ==============================
VIDEO_SOURCE = 0
MODEL_PATH = "models/yolov8n-face.pt"

DEVICE = "cuda" if torch.cuda.is_available() else "cpu"

DISPLAY_WIDTH = 1280
DISPLAY_HEIGHT = 720

# -------------------------------
# ARDUINO SERIAL
# -------------------------------
SERIAL_PORT = "COM4"
BAUD_RATE = 115200

# ==============================
# INIT SERIAL
# ==============================
ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
time.sleep(2)
print("[INFO] Connected to Arduino on", SERIAL_PORT)

# ==============================
# SERVO POSES (SAFE + EXPRESSIVE)
# ==============================
POSES = {
    "HAPPY":    (100, 20, 120),
    "SAD":      (80, 50, 60),
    "ANGRY":    (110, 35, 40),
    "SURPRISE": (90, 5, 150),
    "NEUTRAL":  (90, 0, 90),
}

last_sent_emotion = "NEUTRAL"

def send_pose(emotion):
    global last_sent_emotion
    if emotion == last_sent_emotion:
        return

    if emotion not in POSES:
        emotion = "NEUTRAL"

    b, s, e = POSES[emotion]
    cmd = f"B:{b} S:{s} E:{e}\n"
    ser.write(cmd.encode())
    last_sent_emotion = emotion
    print("[ARM]", emotion, cmd.strip())

# ==============================
# INIT VISION + EMOTION
# ==============================
print("[INFO] Device:", DEVICE)

face_detector = YOLO(MODEL_PATH).to(DEVICE)
emotion_detector = EmotionDetector()

emotion_filter = EmotionFilter(
    window_size=12,
    min_confidence=0.55,
    dominance_margin=0.15
)

derived_engine = DerivedEmotionEngine(bored_time_sec=5.0)
state_machine = EmotionStateMachine(window_size=12, threshold=7)

cap = cv2.VideoCapture(VIDEO_SOURCE)

WINDOW_NAME = "Live Face + Emotion + Arm"
cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_AUTOSIZE)

prev_time = time.time()
final_emotion = "NEUTRAL"

# ==============================
# MAIN LOOP
# ==============================
while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        break

    frame = cv2.flip(frame, 1)

    results = face_detector(
        frame,
        device=DEVICE,
        conf=0.5,
        verbose=False
    )

    if results and len(results[0].boxes) > 0:
        box = results[0].boxes[0].xyxy[0].cpu().numpy().astype(int)
        x1, y1, x2, y2 = box

        h, w, _ = frame.shape
        x1, y1 = max(0, x1), max(0, y1)
        x2, y2 = min(w, x2), min(h, y2)

        face = frame[y1:y2, x1:x2]
        cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)

        # -------- EMOTION PIPELINE --------
        raw_emotions = emotion_detector.predict(face)
        derived_emotion = derived_engine.update(raw_emotions)
        base_emotion, _ = emotion_filter.update(raw_emotions)

        if derived_emotion:
            final_emotion = state_machine.update(derived_emotion)
        elif base_emotion:
            final_emotion = state_machine.update(base_emotion.upper())
        else:
            final_emotion = state_machine.update(None)

    else:
        final_emotion = state_machine.update(None)

    # -------- ARM CONTROL --------
    send_pose(final_emotion)

    # -------- FPS --------
    now = time.time()
    fps = int(1 / (now - prev_time))
    prev_time = now

    # -------- DISPLAY --------
    cv2.putText(
        frame,
        f"Emotion: {final_emotion}",
        (20, 40),
        cv2.FONT_HERSHEY_SIMPLEX,
        1,
        (255, 255, 255),
        2
    )

    cv2.putText(
        frame,
        f"FPS: {fps}",
        (20, 80),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.9,
        (0, 255, 255),
        2
    )

    frame = cv2.resize(frame, (DISPLAY_WIDTH, DISPLAY_HEIGHT))
    cv2.imshow(WINDOW_NAME, frame)

    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

    if cv2.getWindowProperty(WINDOW_NAME, cv2.WND_PROP_VISIBLE) < 1:
        break

# ==============================
# CLEAN EXIT
# ==============================
send_pose("NEUTRAL")
cap.release()
cv2.destroyAllWindows()
ser.close()
