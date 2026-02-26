import cv2
import time
import torch
from ultralytics import YOLO

from vision.emotion_detector import EmotionDetector
from perception.filters import EmotionFilter
from emotion_engine.derived import DerivedEmotionEngine
from emotion_engine.state_machine import EmotionStateMachine

from control.tracker_controller import TrackerController


# ==============================
# CONFIG
# ==============================
VIDEO_SOURCE = 0
MODEL_PATH = r"C:\Users\LENOVO\robo sem 4\software\models\yolov8n-face.pt"

DEVICE = "cuda" if torch.cuda.is_available() else "cpu"

print("[INFO] Device:", DEVICE)

DISPLAY_WIDTH = 1280
DISPLAY_HEIGHT = 720


# ==============================
# INIT MODULES
# ==============================
face_detector = YOLO(MODEL_PATH).to(DEVICE)
emotion_detector = EmotionDetector()

emotion_filter = EmotionFilter(window_size=12,
                               min_confidence=0.55,
                               dominance_margin=0.15)

derived_engine = DerivedEmotionEngine(bored_time_sec=5.0)
state_machine = EmotionStateMachine(window_size=12, threshold=7)


controller = TrackerController(port="COM4")


cap = cv2.VideoCapture(VIDEO_SOURCE)
WINDOW_NAME = "Desk Assistant Robot"
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

    h, w, _ = frame.shape

    # ---------- FACE DETECT ----------
    results = face_detector(frame, conf=0.5, verbose=False)

    if results and len(results[0].boxes) > 0:
        box = results[0].boxes[0].xyxy[0].cpu().numpy().astype(int)
        x1, y1, x2, y2 = box

        face_x = (x1 + x2) // 2
        face_y = (y1 + y2) // 2

        face_crop = frame[y1:y2, x1:x2]

        # ---------- EMOTION ----------
        raw = emotion_detector.predict(face_crop)

        derived = derived_engine.update(raw)
        base_emotion, _ = emotion_filter.update(raw)

        if derived:
            final_emotion = state_machine.update(derived)
        elif base_emotion:
            final_emotion = state_machine.update(base_emotion.upper())
        else:
            final_emotion = state_machine.update(None)

        # ---------- TRACKING ANGLES ----------
        base_angle, shoulder_angle = controller.compute_tracking_angles(
            face_x, face_y, w, h
        )

        # ---------- ELBOW FROM EMOTION ----------
        elbow_angle = controller.emotion_to_elbow(final_emotion)

        # ---------- SEND ----------
        controller.send_angles(base_angle, shoulder_angle, elbow_angle)
        controller.send_emotion(final_emotion) 
        # Draw box
        cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)

    else:
        controller.neutral()

    # ---------- DISPLAY ----------
    fps = int(1 / (time.time() - prev_time))
    prev_time = time.time()

    cv2.putText(frame, f"Emotion: {final_emotion}",
                (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1,
                (255, 255, 255), 2)

    cv2.putText(frame, f"FPS: {fps}",
                (20, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.9,
                (0, 255, 255), 2)

    frame = cv2.resize(frame, (DISPLAY_WIDTH, DISPLAY_HEIGHT))
    cv2.imshow(WINDOW_NAME, frame)

    # Exit
    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

    if cv2.getWindowProperty(WINDOW_NAME, cv2.WND_PROP_VISIBLE) < 1:
        break


# ==============================
# CLEAN EXIT
# ==============================
cap.release()
cv2.destroyAllWindows()
controller.close()
print("[DONE]")
