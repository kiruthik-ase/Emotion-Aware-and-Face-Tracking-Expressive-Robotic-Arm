import cv2
import time
import torch
import multiprocessing as mp
import queue
import numpy as np
from typing import Any
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

# Emotion inference is expensive (DeepFace/TensorFlow). Running it every frame can
# stall/freeze after a short time. Track every frame, but update emotion at a
# lower rate and reuse the last result in-between.
EMOTION_INTERVAL_SEC = 0.50  # 2 Hz

# Smooth face center to reduce jitter while allowing higher tracking gains.
FACE_SMOOTH_ALPHA = 0.40  # 0..1 (higher = more responsive, lower = smoother)

# Optional: set True to print timing once per second
DEBUG_TIMING = False


def _emotion_worker(req_q: Any, resp_q: Any, stop_event: Any):
    """Runs DeepFace emotion inference isolated from the main loop.

    If DeepFace/TensorFlow stalls, the main process stays responsive.
    """

    detector = EmotionDetector()

    while not stop_event.is_set():
        try:
            payload = req_q.get(timeout=0.1)
        except queue.Empty:
            continue

        if payload is None:
            continue

        try:
            buf = np.frombuffer(payload, dtype=np.uint8)
            face_bgr = cv2.imdecode(buf, cv2.IMREAD_COLOR)
            emotions = detector.predict(face_bgr)
        except Exception:
            emotions = {}

        # Keep only the latest response
        try:
            while True:
                resp_q.get_nowait()
        except queue.Empty:
            pass

        try:
            resp_q.put_nowait(emotions)
        except Exception:
            pass


class SafeEmotionInferencer:
    """Manages a DeepFace worker process with watchdog restart."""

    def __init__(self):
        self._ctx = mp.get_context("spawn")
        self._req_q: Any = None
        self._resp_q: Any = None
        self._stop_event: Any = None
        self._proc: Any = None
        self._last_ok_time = 0.0
        self._last_emotions: dict[str, float] = {}

    def start(self):
        self.stop()
        self._req_q = self._ctx.Queue(maxsize=1)
        self._resp_q = self._ctx.Queue(maxsize=1)
        self._stop_event = self._ctx.Event()
        self._proc = self._ctx.Process(
            target=_emotion_worker,
            args=(self._req_q, self._resp_q, self._stop_event),
            daemon=True,
        )
        self._proc.start()
        self._last_ok_time = time.time()

    def stop(self):
        if self._stop_event is not None:
            try:
                self._stop_event.set()
            except Exception:
                pass
        if self._proc is not None:
            try:
                self._proc.join(timeout=0.5)
            except Exception:
                pass
        self._proc = None
        self._stop_event = None
        self._req_q = None
        self._resp_q = None

    def submit_face(self, face_bgr) -> None:
        """Submit a face crop (BGR numpy image). Non-blocking; drops if busy."""

        if self._req_q is None:
            return

        try:
            ok, encoded = cv2.imencode(".jpg", face_bgr, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
            if not ok:
                return
            payload = encoded.tobytes()
        except Exception:
            return

        # Keep only latest request
        try:
            while True:
                self._req_q.get_nowait()
        except Exception:
            pass

        try:
            self._req_q.put_nowait(payload)
        except Exception:
            pass

    def poll_latest(self) -> dict:
        """Return latest emotions if available; otherwise last known."""

        if self._resp_q is None:
            return self._last_emotions

        got_any = False
        try:
            while True:
                emotions = self._resp_q.get_nowait()
                if isinstance(emotions, dict):
                    self._last_emotions = emotions
                    self._last_ok_time = time.time()
                    got_any = True
        except Exception:
            pass

        # Watchdog: if no response for too long, restart worker.
        if not got_any:
            if (time.time() - self._last_ok_time) > 6.0:
                self.start()

        return self._last_emotions


def main():
    # ==============================
    # INIT MODULES
    # ==============================
    face_detector = YOLO(MODEL_PATH).to(DEVICE)

    emotion_filter = EmotionFilter(window_size=12, min_confidence=0.55, dominance_margin=0.15)
    derived_engine = DerivedEmotionEngine(bored_time_sec=5.0)
    state_machine = EmotionStateMachine(window_size=12, threshold=7)

    controller = TrackerController(port="COM4")

    # Presentation stability: keep camera buffer tiny so it can't lag/freeze.
    cap = cv2.VideoCapture(VIDEO_SOURCE)
    try:
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    except Exception:
        pass

    WINDOW_NAME = "Desk Assistant Robot"
    cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_AUTOSIZE)

    # Emotion worker
    emo_worker = SafeEmotionInferencer()
    emo_worker.start()

    prev_time = time.time()
    final_emotion = "NEUTRAL"

    last_emotion_update = 0.0
    smooth_face_x: float | None = None
    smooth_face_y: float | None = None

    t_last_report = time.time()
    t_yolo_ms = 0.0
    t_emotion_ms = 0.0

    try:
        # ==============================
        # MAIN LOOP
        # ==============================
        while cap.isOpened():
            ret, frame = cap.read()
            if not ret:
                time.sleep(0.05)
                continue

            frame = cv2.flip(frame, 1)
            h, w, _ = frame.shape

            # ---------- FACE DETECT ----------
            t0 = time.time()
            try:
                results = face_detector(frame, conf=0.5, verbose=False)
            except Exception:
                results = None
            t_yolo_ms = (time.time() - t0) * 1000.0

            if results and len(results[0].boxes) > 0:
                box = results[0].boxes[0].xyxy[0].cpu().numpy().astype(int)
                x1, y1, x2, y2 = box

                x1, y1 = max(0, x1), max(0, y1)
                x2, y2 = min(w, x2), min(h, y2)

                face_x = (x1 + x2) // 2
                face_y = (y1 + y2) // 2

                if smooth_face_x is None:
                    smooth_face_x = float(face_x)
                    smooth_face_y = float(face_y)
                else:
                    assert smooth_face_y is not None
                    smooth_face_x = (1.0 - FACE_SMOOTH_ALPHA) * smooth_face_x + FACE_SMOOTH_ALPHA * float(face_x)
                    smooth_face_y = (1.0 - FACE_SMOOTH_ALPHA) * smooth_face_y + FACE_SMOOTH_ALPHA * float(face_y)

                face = frame[y1:y2, x1:x2]
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)

                # Submit for emotion inference (non-blocking)
                emo_worker.submit_face(face)

                # ---------- EMOTION PIPELINE (rate-limited, non-blocking) ----------
                now_ts = time.time()
                if (now_ts - last_emotion_update) >= EMOTION_INTERVAL_SEC:
                    last_emotion_update = now_ts
                    te0 = time.time()
                    raw = emo_worker.poll_latest()
                    t_emotion_ms = (time.time() - te0) * 1000.0

                    derived = derived_engine.update(raw)
                    base_emotion, _ = emotion_filter.update(raw)

                    if derived:
                        final_emotion = state_machine.update(derived)
                    elif base_emotion:
                        final_emotion = state_machine.update(base_emotion.upper())
                    else:
                        final_emotion = state_machine.update(None)

                controller.update(
                    face_x=int(smooth_face_x) if smooth_face_x is not None else face_x,
                    face_y=int(smooth_face_y) if smooth_face_y is not None else face_y,
                    frame_w=w,
                    frame_h=h,
                    emotion=final_emotion,
                    face_detected=True,
                )
            else:
                final_emotion = state_machine.update(None)
                smooth_face_x = None
                smooth_face_y = None
                controller.update(
                    face_x=0,
                    face_y=0,
                    frame_w=w,
                    frame_h=h,
                    emotion=final_emotion,
                    face_detected=False,
                )

            # ---------- FPS ----------
            now = time.time()
            fps = int(1 / (now - prev_time)) if (now - prev_time) > 0 else 0
            prev_time = now

            # ---------- DISPLAY ----------
            cv2.putText(frame, f"Emotion: {final_emotion}", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
            cv2.putText(frame, f"FPS: {fps}", (20, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 255), 2)

            frame = cv2.resize(frame, (DISPLAY_WIDTH, DISPLAY_HEIGHT))
            cv2.imshow(WINDOW_NAME, frame)

            if DEBUG_TIMING:
                t_now = time.time()
                if (t_now - t_last_report) >= 1.0:
                    t_last_report = t_now
                    print(f"[TIMING] yolo={t_yolo_ms:.1f}ms emotion_poll={t_emotion_ms:.1f}ms emo={final_emotion}")

            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
            if cv2.getWindowProperty(WINDOW_NAME, cv2.WND_PROP_VISIBLE) < 1:
                break
    finally:
        # ==============================
        # CLEAN EXIT
        # ==============================
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
        try:
            emo_worker.stop()
        except Exception:
            pass

    print("[DONE]")


if __name__ == "__main__":
    mp.freeze_support()
    main()
