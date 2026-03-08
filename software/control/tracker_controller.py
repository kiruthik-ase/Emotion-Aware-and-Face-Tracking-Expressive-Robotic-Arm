import time
import math
import serial


class TrackerController:
    """
    Always-Tracking + Emotion-Posed Robotic Arm Controller

    Architecture:
      - Base: face tracking (horizontal) + emotion offset
      - Shoulder: blended face tracking (vertical) + emotion pose
      - Elbow: purely emotion-driven (gesture intensity)
      - All outputs smoothed via EMA to prevent jitter
      - Serial rate-limited with buffer reset to prevent Arduino freeze
      - OLED updates throttled to prevent I2C blocking
    """

    def __init__(self, port="COM4", baud=115200):

        # ── SERIAL CONNECTION ──
        print("[INFO] Connecting to Arduino...")
        try:
            self.ser = serial.Serial(port, baud, timeout=1)
            time.sleep(2)
            self.connected = True
            print("[INFO] Arduino connected on", port)
        except serial.SerialException:
            self.ser = None
            self.connected = False
            print("[WARN] Arduino not found — running in offline mode")

        # ── TRACKING PARAMETERS ──
        self.BASE_CENTER = 90
        self.SHOULDER_CENTER = 20

        self.TRACK_GAIN_X = 0.25        # base horizontal responsiveness
        self.TRACK_GAIN_Y = 0.25        # shoulder vertical responsiveness
        self.DEADZONE = 8               # pixels — ignore movement below this

        # How much shoulder follows emotion vs tracking
        # 0.0 = pure tracking, 1.0 = pure emotion pose
        self.SHOULDER_EMOTION_BLEND = 0.80

        # ── OUTPUT SMOOTHING ──
        # EMA alpha: lower = smoother/slower, higher = snappier/jerkier
        self.SMOOTH_ALPHA = 0.30
        self.MIN_ANGLE_CHANGE = 2       # degrees — skip tiny updates

        # ── SERVO LIMITS (match Arduino firmware) ──
        self.BASE_MIN, self.BASE_MAX = 60, 120
        self.SHOULDER_MIN, self.SHOULDER_MAX = 0, 60
        self.ELBOW_MIN, self.ELBOW_MAX = 30, 150

        # ── EMOTION POSE TABLE ──
        # (base_offset, shoulder_absolute, elbow_absolute)
        # Resulting absolute base = BASE_CENTER + offset
        # HAPPY→B:100 SAD→B:80 ANGRY→B:110 SURPRISE→B:90 NEUTRAL→B:90
        self.POSES = {
            "HAPPY":    ( 10,  20, 120),   # cheerful raised arm, extended
            "SAD":      (-10,  50,  60),   # droopy low arm, folded
            "ANGRY":    ( 20,  35,  40),   # tense, folded tight
            "SURPRISE": (  0,   5, 150),   # arm shoots out fully
            "NEUTRAL":  (  0,   0,  90),   # relaxed mid-range
            "BORED":    (  5,  10,  80),   # slight lean, reaching
            "EXCITED":  ( 10,  10, 140),   # lean + big extension
            "FEAR":     (  0,   0,  90),   # same as neutral
            "DISGUST":  (  0,   0,  90),   # same as neutral
        }

        # ── RATE LIMITING ──
        self.MIN_SEND_INTERVAL = 0.08   # 80ms → max ~12 commands/sec
        self.last_send_time = 0

        # ── OLED COOLDOWN ──
        self.OLED_COOLDOWN = 1.5
        self.last_oled_time = 0

        # ── STATE ──
        self.current_emotion = "NEUTRAL"
        self.last_cmd = None
        self.last_emo_sent = None

        # Smoothed output angles — start at neutral
        self.smooth_base = float(self.BASE_CENTER)
        self.smooth_shoulder = float(self.SHOULDER_CENTER)
        self.smooth_elbow = 90.0

        # Last sent integer angles
        self.sent_base = self.BASE_CENTER
        self.sent_shoulder = self.SHOULDER_CENTER
        self.sent_elbow = 90

        # Idle state
        self.idle_start_time = None

        # Demo mode emotion list
        self.DEMO_EMOTIONS = ["HAPPY", "SAD", "ANGRY", "SURPRISE", "EXCITED", "BORED", "NEUTRAL"]

    # ── UTILITIES ──

    def clamp(self, val, lo, hi):
        return max(lo, min(val, hi))

    def lerp(self, a, b, t):
        return a * (1.0 - t) + b * t

    # ── MAIN UPDATE — CALL EVERY FRAME ──

    def update(self, face_x, face_y, frame_w, frame_h, emotion):
        """
        face_x, face_y: face center in pixels
        frame_w, frame_h: camera resolution
        emotion: stable emotion string from state machine
        """

        self.idle_start_time = None   # face visible → not idle

        # ── Emotion pose lookup ──
        base_offset, emo_shoulder, emo_elbow = self.POSES.get(
            emotion, (0, 0, 90)
        )

        # ── Raw tracking angles ──
        cx = frame_w // 2
        cy = frame_h // 2

        error_x = face_x - cx
        error_y = face_y - cy

        if abs(error_x) < self.DEADZONE:
            error_x = 0
        if abs(error_y) < self.DEADZONE:
            error_y = 0

        # Base: tracking + emotion offset
        raw_base = self.BASE_CENTER + error_x * self.TRACK_GAIN_X + base_offset

        # Shoulder: blend tracking with emotion pose
        track_shoulder = self.SHOULDER_CENTER - error_y * self.TRACK_GAIN_Y
        raw_shoulder = self.lerp(
            track_shoulder, emo_shoulder, self.SHOULDER_EMOTION_BLEND
        )

        # Elbow: purely emotion-driven
        raw_elbow = float(emo_elbow)

        # ── EMA smoothing ──
        a = self.SMOOTH_ALPHA
        self.smooth_base     = a * raw_base     + (1 - a) * self.smooth_base
        self.smooth_shoulder = a * raw_shoulder  + (1 - a) * self.smooth_shoulder
        self.smooth_elbow    = a * raw_elbow     + (1 - a) * self.smooth_elbow

        # ── Clamp to safe limits ──
        base     = self.clamp(int(round(self.smooth_base)),     self.BASE_MIN,     self.BASE_MAX)
        shoulder = self.clamp(int(round(self.smooth_shoulder)), self.SHOULDER_MIN, self.SHOULDER_MAX)
        elbow    = self.clamp(int(round(self.smooth_elbow)),    self.ELBOW_MIN,    self.ELBOW_MAX)

        # ── Send if changed enough OR heartbeat (prevents freeze) ──
        changed = (
            abs(base - self.sent_base)         >= self.MIN_ANGLE_CHANGE or
            abs(shoulder - self.sent_shoulder)  >= self.MIN_ANGLE_CHANGE or
            abs(elbow - self.sent_elbow)        >= self.MIN_ANGLE_CHANGE
        )
        heartbeat = (time.time() - self.last_send_time) > 2.0

        if changed or heartbeat:
            self._send_angles(base, shoulder, elbow, force=heartbeat)

        # ── OLED update on emotion change ──
        if emotion != self.current_emotion:
            self.current_emotion = emotion
            now = time.time()
            if (now - self.last_oled_time) >= self.OLED_COOLDOWN:
                self._send_emotion(emotion)

    # ── NO FACE — GENTLE RETURN TO NEUTRAL ──

    def no_face(self):
        """Smooth drift back to neutral; gentle scan after 2s idle."""

        if self.idle_start_time is None:
            self.idle_start_time = time.time()

        idle_elapsed = time.time() - self.idle_start_time

        # Gentle drift (slower alpha than normal tracking)
        a = self.SMOOTH_ALPHA * 0.4

        if idle_elapsed > 2.0:
            # After 2s: gentle breathing scan so robot looks alive
            t = time.time()
            target_base     = self.BASE_CENTER + 10 * math.sin(t * 0.4)
            target_shoulder = self.SHOULDER_CENTER + 4 * math.sin(t * 0.5)
            target_elbow    = 90.0 + 8 * math.sin(t * 0.3)
        else:
            # First 2s: settle toward neutral
            target_base     = float(self.BASE_CENTER)
            target_shoulder = float(self.SHOULDER_CENTER)
            target_elbow    = 90.0

        self.smooth_base     = a * target_base     + (1 - a) * self.smooth_base
        self.smooth_shoulder = a * target_shoulder  + (1 - a) * self.smooth_shoulder
        self.smooth_elbow    = a * target_elbow     + (1 - a) * self.smooth_elbow

        base     = self.clamp(int(round(self.smooth_base)),     self.BASE_MIN,     self.BASE_MAX)
        shoulder = self.clamp(int(round(self.smooth_shoulder)), self.SHOULDER_MIN, self.SHOULDER_MAX)
        elbow    = self.clamp(int(round(self.smooth_elbow)),    self.ELBOW_MIN,    self.ELBOW_MAX)

        changed = (
            abs(base - self.sent_base)         >= self.MIN_ANGLE_CHANGE or
            abs(shoulder - self.sent_shoulder)  >= self.MIN_ANGLE_CHANGE or
            abs(elbow - self.sent_elbow)        >= self.MIN_ANGLE_CHANGE
        )
        heartbeat = (time.time() - self.last_send_time) > 2.0

        if changed or heartbeat:
            self._send_angles(base, shoulder, elbow, force=heartbeat)

        # Update OLED to neutral
        if self.current_emotion != "NEUTRAL":
            self.current_emotion = "NEUTRAL"
            now = time.time()
            if (now - self.last_oled_time) >= self.OLED_COOLDOWN:
                self._send_emotion("NEUTRAL")

    # ── SERIAL: SEND SERVO ANGLES ──

    def _send_angles(self, base, shoulder, elbow, force=False):
        if not self.connected:
            return

        now = time.time()
        if not force and (now - self.last_send_time) < self.MIN_SEND_INTERVAL:
            return

        cmd = f"B:{base} S:{shoulder} E:{elbow}"

        if not force and cmd == self.last_cmd:
            return

        try:
            self.ser.reset_output_buffer()
            self.ser.write((cmd + "\n").encode())
            self.last_cmd = cmd
            self.last_send_time = now
            self.sent_base = base
            self.sent_shoulder = shoulder
            self.sent_elbow = elbow
            print("[SEND]", cmd)
        except (serial.SerialException, serial.SerialTimeoutException):
            print("[WARN] Serial write failed")

    # ── SERIAL: OLED EMOTION LABEL ──

    def _send_emotion(self, emotion):
        if not self.connected:
            return

        if emotion == self.last_emo_sent:
            return

        try:
            time.sleep(0.02)
            self.ser.write(f"EMO:{emotion}\n".encode())
            self.last_emo_sent = emotion
            self.last_oled_time = time.time()
            print("[OLED]", emotion)
        except (serial.SerialException, serial.SerialTimeoutException):
            print("[WARN] OLED write failed")

    # ── DEMO MODE ──
    # Absolute angles matching user's target values
    DEMO_POSES = {
        "HAPPY":    (100, 20, 120),
        "SAD":      ( 80, 50,  60),
        "ANGRY":    (110, 35,  40),
        "SURPRISE": ( 90,  5, 150),
        "NEUTRAL":  ( 90,  0,  90),
        "BORED":    ( 95, 10,  80),
        "EXCITED":  (100, 10, 140),
    }

    def demo_pose(self, emotion):
        """Send demo angles with smooth transition via EMA."""
        base, shoulder, elbow = self.DEMO_POSES.get(emotion, (90, 0, 90))

        # Smooth even in demo for clean transitions
        a = self.SMOOTH_ALPHA
        self.smooth_base     = a * float(base)     + (1 - a) * self.smooth_base
        self.smooth_shoulder = a * float(shoulder)  + (1 - a) * self.smooth_shoulder
        self.smooth_elbow    = a * float(elbow)     + (1 - a) * self.smooth_elbow

        b = self.clamp(int(round(self.smooth_base)),     self.BASE_MIN,     self.BASE_MAX)
        s = self.clamp(int(round(self.smooth_shoulder)), self.SHOULDER_MIN, self.SHOULDER_MAX)
        e = self.clamp(int(round(self.smooth_elbow)),    self.ELBOW_MIN,    self.ELBOW_MAX)

        if emotion != self.current_emotion:
            self.current_emotion = emotion
            self._send_emotion(emotion)

        changed = (
            abs(b - self.sent_base)     >= self.MIN_ANGLE_CHANGE or
            abs(s - self.sent_shoulder) >= self.MIN_ANGLE_CHANGE or
            abs(e - self.sent_elbow)    >= self.MIN_ANGLE_CHANGE
        )
        heartbeat = (time.time() - self.last_send_time) > 1.5

        if changed or heartbeat:
            self._send_angles(b, s, e, force=heartbeat)

    # ── FORCE NEUTRAL (for shutdown) ──

    def neutral(self):
        """Force neutral — bypasses all smoothing and rate limiting.
        Retries up to 3 times to guarantee the arm returns home."""

        if not self.connected:
            return

        cmd = f"B:{self.BASE_CENTER} S:{self.SHOULDER_CENTER} E:90"

        for attempt in range(3):
            try:
                self.ser.reset_output_buffer()
                self.ser.write((cmd + "\n").encode())
                time.sleep(0.15)
                self.ser.write("EMO:NEUTRAL\n".encode())
                time.sleep(0.05)
                self.last_cmd = cmd
                self.last_emo_sent = "NEUTRAL"
                self.current_emotion = "NEUTRAL"
                self.smooth_base = float(self.BASE_CENTER)
                self.smooth_shoulder = float(self.SHOULDER_CENTER)
                self.smooth_elbow = 90.0
                print(f"[NEUTRAL] {cmd} (attempt {attempt + 1})")
                return
            except (serial.SerialException, serial.SerialTimeoutException):
                print(f"[WARN] Neutral attempt {attempt + 1} failed")
                time.sleep(0.1)

        print("[ERROR] Could not send neutral after 3 attempts")

    # ── CLOSE ──

    def close(self):
        """Return to neutral and close serial port cleanly."""

        print("[INFO] Shutting down controller...")
        self.neutral()

        if self.connected and self.ser and self.ser.is_open:
            time.sleep(0.3)
            self.ser.close()
            print("[INFO] Serial port closed")
