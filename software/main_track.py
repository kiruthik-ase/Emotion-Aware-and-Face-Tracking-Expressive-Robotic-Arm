import cv2
import time
import signal
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
MODEL_PATH = "models/yolov8n-face.pt"

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

# Demo mode state
demo_mode = False
demo_index = 0
demo_last_switch = 0
DEMO_HOLD_SEC = 3.0   # seconds per emotion in demo


# ==============================
# CLEAN EXIT (handles Q, window close, Ctrl+C, and crashes)
# ==============================
def clean_exit():
    """Guaranteed neutral + close, called from any exit path."""
    print("[EXIT] Cleaning up...")
    try:
        cap.release()
    except Exception:
        pass
    try:
        cv2.destroyAllWindows()
    except Exception:
        pass
    try:
        controller.close()
    except Exception:
        pass
    print("[DONE]")

def signal_handler(sig, frame):
    clean_exit()
    raise SystemExit(0)

signal.signal(signal.SIGINT, signal_handler)
signal.signal(signal.SIGTERM, signal_handler)

try:
    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            break

        frame = cv2.flip(frame, 1)
        h, w, _ = frame.shape

        key = cv2.waitKey(1) & 0xFF

        # ---------- DEMO MODE TOGGLE (D key) ----------
        if key == ord("d"):
            demo_mode = not demo_mode
            if demo_mode:
                demo_index = 0
                demo_last_switch = time.time()
                print("[DEMO] Demo mode ON — cycling all emotions")
            else:
                print("[DEMO] Demo mode OFF — back to live tracking")

        if demo_mode:
            # Cycle to next emotion every DEMO_HOLD_SEC seconds
            if time.time() - demo_last_switch >= DEMO_HOLD_SEC:
                demo_index = (demo_index + 1) % len(controller.DEMO_EMOTIONS)
                demo_last_switch = time.time()

            demo_emo = controller.DEMO_EMOTIONS[demo_index]
            controller.demo_pose(demo_emo)

            # OSD for demo
            cv2.putText(frame, "DEMO MODE",
                        (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1.0,
                        (0, 0, 255), 2)
            cv2.putText(frame, f"Pose: {demo_emo}  [{demo_index+1}/{len(controller.DEMO_EMOTIONS)}]",
                        (20, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.9,
                        (0, 255, 255), 2)
            cv2.putText(frame, "Press D to exit demo | N = next",
                        (20, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                        (180, 180, 180), 1)

            # N key = skip to next emotion immediately
            if key == ord("n"):
                demo_index = (demo_index + 1) % len(controller.DEMO_EMOTIONS)
                demo_last_switch = time.time()

            frame = cv2.resize(frame, (DISPLAY_WIDTH, DISPLAY_HEIGHT))
            cv2.imshow(WINDOW_NAME, frame)

            if key == ord("q"):
                break
            if cv2.getWindowProperty(WINDOW_NAME, cv2.WND_PROP_VISIBLE) < 1:
                break
            continue   # skip normal pipeline in demo mode

        # ---------- FACE DETECT ----------
        results = face_detector(frame, conf=0.5, verbose=False)

        if results and len(results[0].boxes) > 0:
            box = results[0].boxes[0].xyxy[0].cpu().numpy().astype(int)
            x1, y1, x2, y2 = box

            # Clamp to frame bounds
            x1, y1 = max(0, x1), max(0, y1)
            x2, y2 = min(w, x2), min(h, y2)

            # Face center for tracking
            face_x = (x1 + x2) // 2
            face_y = (y1 + y2) // 2

            face = frame[y1:y2, x1:x2]
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)

            # ---------- EMOTION PIPELINE ----------
            raw = emotion_detector.predict(face)
            derived = derived_engine.update(raw)
            base_emotion, _ = emotion_filter.update(raw)

            if derived:
                final_emotion = state_machine.update(derived)
            elif base_emotion:
                final_emotion = state_machine.update(base_emotion.upper())
            else:
                final_emotion = state_machine.update(None)

            # ---------- ALWAYS-TRACKING + EMOTION CONTROL ----------
            controller.update(face_x, face_y, w, h, final_emotion)

        else:
            final_emotion = state_machine.update(None)

            # ---------- GRADUAL NEUTRAL RETURN ----------
            controller.no_face()

        # ---------- FPS ----------
        now = time.time()
        fps = int(1 / (now - prev_time)) if (now - prev_time) > 0 else 0
        prev_time = now

        # ---------- DISPLAY ----------
        cv2.putText(frame, f"Emotion: {final_emotion}",
                    (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1,
                    (255, 255, 255), 2)

        cv2.putText(frame, f"FPS: {fps}",
                    (20, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.9,
                    (0, 255, 255), 2)

        cv2.putText(frame, "D=Demo  Q=Quit",
                    (20, h - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                    (120, 120, 120), 1)

        frame = cv2.resize(frame, (DISPLAY_WIDTH, DISPLAY_HEIGHT))
        cv2.imshow(WINDOW_NAME, frame)

        if key == ord("q"):
            break
        if cv2.getWindowProperty(WINDOW_NAME, cv2.WND_PROP_VISIBLE) < 1:
            break

except Exception as e:
    print(f"[ERROR] {e}")
finally:
    clean_exit()
