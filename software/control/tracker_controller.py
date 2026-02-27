import time
import serial


class TrackerController:
    """
    Always-Tracking + Emotion-Posed Robotic Arm Controller

    Behavior:
      - Base ALWAYS follows face left/right (+ subtle emotion bias)
      - Shoulder BLENDS face tracking with emotion shoulder pose
      - Elbow is purely emotion-driven (gesture intensity)
      - OUTPUT SMOOTHING prevents wild servo oscillations
      - Serial rate-limited to prevent Arduino freeze
      - OLED has cooldown to prevent display freeze
    """

    def __init__(self, port="COM4", baud=115200):

        # =============================
        # SERIAL CONNECTION
        # =============================
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

        # =============================
        # TRACKING PARAMETERS
        # =============================
        self.BASE_CENTER = 90
        self.SHOULDER_CENTER = 20

        # Proportional gains
        self.TRACK_GAIN_X = 0.25     # base left/right
        self.TRACK_GAIN_Y = 0.20     # shoulder up/down

        # Deadzone (pixels) — face movement below this is ignored
        self.DEADZONE = 8

        # Blend factor: how much emotion influences shoulder vs tracking
        # 0.0 = pure tracking, 1.0 = pure emotion pose
        self.SHOULDER_EMOTION_BLEND = 0.4

        # =============================
        # OUTPUT SMOOTHING (CRITICAL — prevents Arduino freeze)
        # =============================
        # Exponential moving average: smoothed = alpha * new + (1-alpha) * old
        # Lower alpha = smoother but slower response
        # Higher alpha = faster but can oscillate
        self.SMOOTH_ALPHA = 0.3

        # Minimum angle change to actually send a command (prevents micro-jitter)
        self.MIN_ANGLE_CHANGE = 2

        # =============================
        # SAFE SERVO LIMITS (match Arduino firmware)
        # =============================
        self.BASE_MIN, self.BASE_MAX = 60, 120
        self.SHOULDER_MIN, self.SHOULDER_MAX = 0, 60
        self.ELBOW_MIN, self.ELBOW_MAX = 30, 150

        # =============================
        # EMOTION POSE TABLE
        # (base_offset, shoulder_pose, elbow_pose)
        # =============================
        self.POSES = {
            "HAPPY":    (5,   20,  120),
            "SAD":      (-5,  50,  60),
            "ANGRY":    (10,  35,  40),
            "SURPRISE": (0,   5,   150),
            "NEUTRAL":  (0,   0,   90),
            "BORED":    (0,   10,  80),
            "EXCITED":  (0,   10,  140),
            "FEAR":     (0,   0,   90),
            "DISGUST":  (0,   0,   90),
        }

        # =============================
        # RATE LIMITING (prevents Arduino serial buffer overflow)
        # =============================
        self.MIN_SEND_INTERVAL = 0.08   # 80ms = max ~12 commands/sec
        self.last_send_time = 0

        # =============================
        # OLED COOLDOWN (prevents OLED freeze)
        # =============================
        self.OLED_COOLDOWN = 1.5        # min seconds between OLED updates
        self.last_oled_time = 0

        # =============================
        # STATE
        # =============================
        self.current_emotion = "NEUTRAL"
        self.last_cmd = None
        self.last_emo_sent = None

        # Smoothed output angles (EMA state)
        self.smooth_base = float(self.BASE_CENTER)
        self.smooth_shoulder = float(self.SHOULDER_CENTER)
        self.smooth_elbow = 90.0

        # Last sent integer angles (for change detection)
        self.sent_base = self.BASE_CENTER
        self.sent_shoulder = self.SHOULDER_CENTER
        self.sent_elbow = 90

    # =============================
    # UTILITIES
    # =============================
    def clamp(self, val, lo, hi):
        return max(lo, min(val, hi))

    def lerp(self, a, b, t):
        """Linear interpolation: a*(1-t) + b*t"""
        return a * (1.0 - t) + b * t

    # =============================
    # MAIN UPDATE — CALL EVERY FRAME
    # =============================
    def update(self, face_x, face_y, frame_w, frame_h, emotion):
        """
        Call this every frame when a face is detected.

        face_x, face_y: center of detected face in pixels
        frame_w, frame_h: camera frame dimensions
        emotion: stable emotion string (from state machine)
        """

        # --- Get emotion pose values ---
        base_offset, emo_shoulder, emo_elbow = self.POSES.get(
            emotion, (0, 0, 90)
        )

        # --- Compute raw tracking angles ---
        cx = frame_w // 2
        cy = frame_h // 2

        error_x = face_x - cx
        error_y = face_y - cy

        # Deadzone removes micro-jitter
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

        # --- EXPONENTIAL SMOOTHING (prevents wild oscillations) ---
        alpha = self.SMOOTH_ALPHA
        self.smooth_base = alpha * raw_base + (1 - alpha) * self.smooth_base
        self.smooth_shoulder = alpha * raw_shoulder + (1 - alpha) * self.smooth_shoulder
        self.smooth_elbow = alpha * raw_elbow + (1 - alpha) * self.smooth_elbow

        # --- Clamp to safe limits ---
        base = self.clamp(int(round(self.smooth_base)), self.BASE_MIN, self.BASE_MAX)
        shoulder = self.clamp(int(round(self.smooth_shoulder)), self.SHOULDER_MIN, self.SHOULDER_MAX)
        elbow = self.clamp(int(round(self.smooth_elbow)), self.ELBOW_MIN, self.ELBOW_MAX)

        # --- Only send if angles changed enough ---
        changed = (
            abs(base - self.sent_base) >= self.MIN_ANGLE_CHANGE or
            abs(shoulder - self.sent_shoulder) >= self.MIN_ANGLE_CHANGE or
            abs(elbow - self.sent_elbow) >= self.MIN_ANGLE_CHANGE
        )

        if changed:
            self._send_angles(base, shoulder, elbow)

        # --- Update OLED if emotion changed (with cooldown) ---
        if emotion != self.current_emotion:
            self.current_emotion = emotion
            now = time.time()
            if (now - self.last_oled_time) >= self.OLED_COOLDOWN:
                self._send_emotion(emotion)

    # =============================
    # NO FACE — GRADUAL NEUTRAL RETURN
    # =============================
    def no_face(self):
        """
        Call when no face is detected.
        Smoothly returns toward neutral pose using the same EMA.
        """

        alpha = self.SMOOTH_ALPHA * 0.5  # slower return to neutral

        self.smooth_base = alpha * self.BASE_CENTER + (1 - alpha) * self.smooth_base
        self.smooth_shoulder = alpha * self.SHOULDER_CENTER + (1 - alpha) * self.smooth_shoulder
        self.smooth_elbow = alpha * 90.0 + (1 - alpha) * self.smooth_elbow

        base = self.clamp(int(round(self.smooth_base)), self.BASE_MIN, self.BASE_MAX)
        shoulder = self.clamp(int(round(self.smooth_shoulder)), self.SHOULDER_MIN, self.SHOULDER_MAX)
        elbow = self.clamp(int(round(self.smooth_elbow)), self.ELBOW_MIN, self.ELBOW_MAX)

        changed = (
            abs(base - self.sent_base) >= self.MIN_ANGLE_CHANGE or
            abs(shoulder - self.sent_shoulder) >= self.MIN_ANGLE_CHANGE or
            abs(elbow - self.sent_elbow) >= self.MIN_ANGLE_CHANGE
        )

        if changed:
            self._send_angles(base, shoulder, elbow)

        # Update OLED to neutral
        if self.current_emotion != "NEUTRAL":
            self.current_emotion = "NEUTRAL"
            now = time.time()
            if (now - self.last_oled_time) >= self.OLED_COOLDOWN:
                self._send_emotion("NEUTRAL")

    # =============================
    # SERIAL: SEND SERVO ANGLES (RATE LIMITED)
    # =============================
    def _send_angles(self, base, shoulder, elbow):
        """
        Send: B:90 S:20 E:120
        Rate-limited and change-gated to prevent Arduino freeze.
        """

        if not self.connected:
            return

        # Rate limit
        now = time.time()
        if (now - self.last_send_time) < self.MIN_SEND_INTERVAL:
            return

        cmd = f"B:{base} S:{shoulder} E:{elbow}"

        # Skip exact duplicate commands
        if cmd == self.last_cmd:
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

    # =============================
    # SERIAL: OLED EMOTION LABEL (WITH COOLDOWN)
    # =============================
    def _send_emotion(self, emotion):
        """Send: EMO:HAPPY  (min 1.5s between sends)"""

        if not self.connected:
            return

        if emotion == self.last_emo_sent:
            return

        try:
            # Small delay to let Arduino finish any servo movement first
            time.sleep(0.02)
            self.ser.write(f"EMO:{emotion}\n".encode())
            self.last_emo_sent = emotion
            self.last_oled_time = time.time()
            print("[OLED]", emotion)
        except (serial.SerialException, serial.SerialTimeoutException):
            print("[WARN] OLED write failed")

    # =============================
    # FORCE NEUTRAL (for shutdown)
    # =============================
    def neutral(self):
        """Force neutral — bypasses rate limiting, smoothing, and dedup."""

        if not self.connected:
            return

        cmd = f"B:{self.BASE_CENTER} S:{self.SHOULDER_CENTER} E:90"

        try:
            self.ser.reset_output_buffer()
            self.ser.write((cmd + "\n").encode())
            time.sleep(0.15)
            self.ser.write(f"EMO:NEUTRAL\n".encode())
            time.sleep(0.05)
            self.last_cmd = cmd
            self.last_emo_sent = "NEUTRAL"
            self.current_emotion = "NEUTRAL"
            self.smooth_base = float(self.BASE_CENTER)
            self.smooth_shoulder = float(self.SHOULDER_CENTER)
            self.smooth_elbow = 90.0
            print("[NEUTRAL]", cmd)
        except (serial.SerialException, serial.SerialTimeoutException):
            print("[WARN] Neutral send failed")

    # =============================
    # CLOSE — CLEAN SHUTDOWN
    # =============================
    def close(self):
        """Return to neutral and close serial port cleanly."""

        print("[INFO] Shutting down controller...")
        self.neutral()

        if self.connected and self.ser and self.ser.is_open:
            time.sleep(0.3)
            self.ser.close()
            print("[INFO] Serial port closed")
