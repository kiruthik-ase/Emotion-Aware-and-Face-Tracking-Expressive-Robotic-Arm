import time
import serial
import math


class TrackerController:
    """Combined face tracking + emotion arm controller.

    ALWAYS_TRACK mode:
      - base follows face left/right
      - shoulder follows face up/down
      - elbow is emotion-driven
      - OLED emotion is sent via EMO:<emotion>

    Anti-jitter measures:
      - deadzone
      - integer-degree quantization
      - command rate limiting

        Sweet-spot behavior (tracking + dramatic poses):
            - base+shoulder always track the face
            - emotion provides a full pose (B,S,E) that is BLENDED into tracking targets
            - when emotion changes, a short "gesture burst" increases the pose blend for drama
    """

    def __init__(self, port: str = "COM4", baud: int = 115200):
        print("[INFO] Connecting to Arduino...")
        # Important: set a write_timeout so serial.write() cannot block forever.
        # Blocking writes are a common cause of the Python loop "freezing".
        self.ser = serial.Serial(port, baud, timeout=0.2, write_timeout=0.05)
        time.sleep(2)
        print("[INFO] Arduino connected on", port)

        # Calibrated centers (tweak if robot is placed aside)
        self.BASE_CENTER = 90
        self.SHOULDER_CENTER = 20

        # Tracking parameters (more responsive)
        # "More obsessed" tracking: higher gains + smaller deadzone.
        # Pump vertical (Y) more because shoulder motion felt weak.
        # (jitter is handled by face smoothing in main + command throttling here)
        self.TRACK_GAIN_X = 0.14
        self.TRACK_GAIN_Y = 0.20
        self.DEADZONE = 10

        # Safe servo limits
        self.BASE_MIN, self.BASE_MAX = 60, 120
        self.SHOULDER_MIN, self.SHOULDER_MAX = 0, 60
        self.ELBOW_MIN, self.ELBOW_MAX = 30, 150

        # Emotion → elbow mapping
        self.EMOTION_ELBOW = {
            "HAPPY": 120,
            "SAD": 60,
            "ANGRY": 40,
            "SURPRISE": 150,
            "NEUTRAL": 90,
            "BORED": 80,
            "EXCITED": 140,
        }

        # Full emotion poses (from your slide table) + derived emotions
        self.EMOTION_POSES = {
            "HAPPY": (100, 20, 120),
            "SAD": (80, 50, 60),
            "ANGRY": (110, 35, 40),
            "SURPRISE": (90, 5, 150),
            "NEUTRAL": (90, 0, 90),
            "BORED": (90, 10, 80),
            "EXCITED": (100, 10, 140),
        }

        # Personality tuning: always-on pose influence (0..1)
        self.POSE_BIAS_ALPHA = {
            "NEUTRAL": 0.00,
            "HAPPY": 0.08,
            "SAD": 0.10,
            "ANGRY": 0.10,
            "SURPRISE": 0.06,
            "BORED": 0.06,
            "EXCITED": 0.08,
        }

        # Dramatic gesture burst on emotion change
        # (keep it, but cap it so tracking stays dominant)
        self.GESTURE_PEAK_ALPHA = 0.45
        self.GESTURE_HOLD_SEC = 0.40
        self.GESTURE_RELEASE_SEC = 0.80

        # Hard cap: max influence of emotion pose on base+shoulder
        # Keep tracking dominant (especially after raising gains)
        self.MAX_POSE_ALPHA = 0.25

        # Small animation strength during gesture burst
        self.WAGGLE_ELBOW_DEG = 10
        self.WAGGLE_SHOULDER_DEG = 5

        # Internal state
        self._current_emotion = "NEUTRAL"
        self._gesture_start_time = 0.0
        self._face_present = True

        # Send throttling
        self._last_cmd: str | None = None
        self._last_emo: str | None = None
        self._last_emo_time = 0.0
        self._last_send_time = 0.0
        self._last_angles: tuple[int, int, int] | None = None
        self.SEND_INTERVAL_SEC = 0.04  # ~25 Hz (less serial/OLED stress)

        # Only send new angles if movement is meaningful.
        self.MIN_ANGLE_DELTA_DEG = 2

        # OLED refresh can be slow; rate-limit emotion text updates.
        self.EMO_MIN_INTERVAL_SEC = 0.50

    def clamp(self, val, vmin, vmax):
        return max(vmin, min(val, vmax))

    def lerp(self, a: float, b: float, t: float) -> float:
        return a + (b - a) * t

    def _pose_alpha(self, emotion: str, now: float) -> float:
        bias = self.POSE_BIAS_ALPHA.get(emotion, 0.0)

        # transient burst
        dt = now - self._gesture_start_time
        transient = 0.0
        if dt < 0:
            transient = 0.0
        elif dt <= self.GESTURE_HOLD_SEC:
            transient = self.GESTURE_PEAK_ALPHA
        elif dt <= (self.GESTURE_HOLD_SEC + self.GESTURE_RELEASE_SEC):
            k = (dt - self.GESTURE_HOLD_SEC) / max(1e-6, self.GESTURE_RELEASE_SEC)
            transient = self.GESTURE_PEAK_ALPHA * (1.0 - k)

        return self.clamp(bias + transient, 0.0, self.MAX_POSE_ALPHA)

    def _gesture_wiggle(self, emotion: str, now: float):
        """Returns small (base_delta, shoulder_delta, elbow_delta) during gesture burst."""

        dt = now - self._gesture_start_time
        if dt < 0 or dt > (self.GESTURE_HOLD_SEC + self.GESTURE_RELEASE_SEC):
            return 0.0, 0.0, 0.0

        # 3 Hz wiggle
        phase = 2.0 * math.pi * 3.0 * dt

        if emotion in ("HAPPY", "EXCITED"):
            elbow = math.sin(phase) * self.WAGGLE_ELBOW_DEG
            shoulder = math.sin(phase * 0.5) * (self.WAGGLE_SHOULDER_DEG * 0.6)
            return 0.0, shoulder, elbow

        if emotion == "SURPRISE":
            # quick shoulder bounce
            shoulder = abs(math.sin(phase)) * self.WAGGLE_SHOULDER_DEG
            return 0.0, shoulder, 0.0

        return 0.0, 0.0, 0.0

    def compute_tracking_angles(self, face_x: int, face_y: int, frame_w: int, frame_h: int):
        cx = frame_w // 2
        cy = frame_h // 2

        error_x = face_x - cx
        error_y = face_y - cy

        if abs(error_x) < self.DEADZONE:
            error_x = 0
        if abs(error_y) < self.DEADZONE:
            error_y = 0

        base = self.BASE_CENTER + error_x * self.TRACK_GAIN_X
        shoulder = self.SHOULDER_CENTER - error_y * self.TRACK_GAIN_Y

        base = self.clamp(base, self.BASE_MIN, self.BASE_MAX)
        shoulder = self.clamp(shoulder, self.SHOULDER_MIN, self.SHOULDER_MAX)

        return base, shoulder

    def emotion_to_elbow(self, emotion: str):
        elbow = self.EMOTION_ELBOW.get(emotion, 90)
        return self.clamp(elbow, self.ELBOW_MIN, self.ELBOW_MAX)

    def update(self, face_x: int, face_y: int, frame_w: int, frame_h: int, emotion: str, face_detected: bool):
        """One call per frame: blends tracking with full emotion pose + OLED."""

        now = time.time()

        if not face_detected:
            # When face is lost: go neutral once, then stay still.
            if self._face_present:
                self._face_present = False
                self._current_emotion = "NEUTRAL"
                self._gesture_start_time = now
                self.neutral()
            return

        if not self._face_present:
            # Face just returned
            self._face_present = True

        if emotion != self._current_emotion:
            self._current_emotion = emotion
            self._gesture_start_time = now
            self.send_emotion(emotion)

        # Tracking targets
        track_base, track_shoulder = self.compute_tracking_angles(face_x, face_y, frame_w, frame_h)

        # Pose targets
        pose_base, pose_shoulder, pose_elbow = self.EMOTION_POSES.get(emotion, self.EMOTION_POSES["NEUTRAL"])

        alpha = self._pose_alpha(emotion, now)

        base = self.lerp(track_base, pose_base, alpha)
        shoulder = self.lerp(track_shoulder, pose_shoulder, alpha)

        # Elbow stays expressive (pose) but still clamped
        elbow = pose_elbow

        # Small wiggle during the gesture burst for personality
        db, ds, de = self._gesture_wiggle(emotion, now)
        base += db
        shoulder += ds
        elbow += de

        self.send_angles(base, shoulder, elbow)

    def send_angles(self, base, shoulder, elbow):
        now = time.time()
        if (now - self._last_send_time) < self.SEND_INTERVAL_SEC:
            return

        base_i = int(round(base))
        shoulder_i = int(round(shoulder))
        elbow_i = int(round(elbow))

        base_i = self.clamp(base_i, self.BASE_MIN, self.BASE_MAX)
        shoulder_i = self.clamp(shoulder_i, self.SHOULDER_MIN, self.SHOULDER_MAX)
        elbow_i = self.clamp(elbow_i, self.ELBOW_MIN, self.ELBOW_MAX)

        cmd = f"B:{base_i} S:{shoulder_i} E:{elbow_i}"
        if cmd == self._last_cmd:
            return

        if self._last_angles is not None:
            lb, ls, le = self._last_angles
            if (
                abs(base_i - lb) < self.MIN_ANGLE_DELTA_DEG
                and abs(shoulder_i - ls) < self.MIN_ANGLE_DELTA_DEG
                and abs(elbow_i - le) < self.MIN_ANGLE_DELTA_DEG
            ):
                return

        try:
            self.ser.write((cmd + "\n").encode())
        except (serial.SerialTimeoutException, serial.SerialException, OSError):
            return

        self._last_cmd = cmd
        self._last_angles = (base_i, shoulder_i, elbow_i)
        self._last_send_time = now

    def send_emotion(self, emotion: str):
        if not emotion:
            return

        if emotion == self._last_emo:
            return

        now = time.time()
        if (now - self._last_emo_time) < self.EMO_MIN_INTERVAL_SEC:
            return

        try:
            self.ser.write(f"EMO:{emotion}\n".encode())
        except (serial.SerialTimeoutException, serial.SerialException, OSError):
            return

        self._last_emo = emotion
        self._last_emo_time = now

    def neutral(self):
        self.send_angles(self.BASE_CENTER, self.SHOULDER_CENTER, 90)
        self.send_emotion("NEUTRAL")

    def close(self):
        print("[INFO] Closing Serial + Neutral Pose")
        self.neutral()
        self.ser.close()
