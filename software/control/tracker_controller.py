import time
import serial


class TrackerController:
    """
    Always-Tracking + Emotion-Posed Robotic Arm Controller

    Behavior:
      - Base ALWAYS follows face left/right (+ subtle emotion bias)
      - Shoulder BLENDS face tracking with emotion shoulder pose
      - Elbow is purely emotion-driven (gesture intensity)
      - Serial commands are rate-limited to prevent Arduino freeze
      - OLED updated on emotion change

    The robot never stops watching you.
    Its posture shifts with your mood.
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

        # Proportional gains (how aggressively arm follows face)
        self.TRACK_GAIN_X = 0.12     # base left/right
        self.TRACK_GAIN_Y = 0.10     # shoulder up/down

        # Deadzone — pixels of face movement ignored (removes jitter)
        self.DEADZONE = 15

        # Blend factor: how much emotion influences shoulder vs tracking
        # 0.0 = pure tracking, 1.0 = pure emotion pose
        self.SHOULDER_EMOTION_BLEND = 0.6

        # =============================
        # SAFE SERVO LIMITS (match Arduino firmware)
        # =============================
        self.BASE_MIN, self.BASE_MAX = 60, 120
        self.SHOULDER_MIN, self.SHOULDER_MAX = 0, 60
        self.ELBOW_MIN, self.ELBOW_MAX = 30, 150

        # =============================
        # EMOTION POSE TABLE
        # (base_offset, shoulder_pose, elbow_pose)
        #
        # base_offset: added to tracking base angle
        #   positive = lean right, negative = lean left
        # shoulder_pose: blended with tracking shoulder
        # elbow_pose: direct control
        # =============================
        self.POSES = {
            "HAPPY":    (5,   20,  120),
            "SAD":      (-5,  50,  60),
            "ANGRY":    (10,  35,  40),
            "SURPRISE": (0,   5,   150),
            "NEUTRAL":  (0,   0,   90),
            "BORED":    (0,   10,  80),
            "EXCITED":  (0,   10,  140),
        }

        # =============================
        # RATE LIMITING (prevents Arduino freeze)
        # =============================
        self.MIN_SEND_INTERVAL = 0.05   # 50ms = max ~20 commands/sec
        self.last_send_time = 0

        # =============================
        # STATE
        # =============================
        self.current_emotion = "NEUTRAL"
        self.last_cmd = None
        self.last_emo_sent = None

        # Last known tracking angles (for smooth no-face fallback)
        self.last_base = self.BASE_CENTER
        self.last_shoulder = self.SHOULDER_CENTER
        self.last_elbow = 90

    # =============================
    # CLAMP UTILITY
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

        # --- Compute tracking angles ---
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
        track_base = self.BASE_CENTER + error_x * self.TRACK_GAIN_X
        base = track_base + base_offset

        # Shoulder: blend tracking with emotion pose
        track_shoulder = self.SHOULDER_CENTER - error_y * self.TRACK_GAIN_Y
        shoulder = self.lerp(
            track_shoulder, emo_shoulder, self.SHOULDER_EMOTION_BLEND
        )

        # Elbow: purely emotion-driven
        elbow = emo_elbow

        # --- Clamp to safe limits ---
        base = self.clamp(int(base), self.BASE_MIN, self.BASE_MAX)
        shoulder = self.clamp(int(shoulder), self.SHOULDER_MIN, self.SHOULDER_MAX)
        elbow = self.clamp(int(elbow), self.ELBOW_MIN, self.ELBOW_MAX)

        # Store for fallback
        self.last_base = base
        self.last_shoulder = shoulder
        self.last_elbow = elbow

        # --- Send to Arduino (rate limited) ---
        self._send_angles(base, shoulder, elbow)

        # --- Update OLED if emotion changed ---
        if emotion != self.current_emotion:
            self.current_emotion = emotion
            self._send_emotion(emotion)

    # =============================
    # NO FACE — GRADUAL NEUTRAL RETURN
    # =============================
    def no_face(self):
        """
        Call when no face is detected.
        Smoothly returns toward neutral pose.
        """

        # Gradually move toward neutral (1 degree per call)
        step = 2

        if self.last_base < self.BASE_CENTER:
            self.last_base = min(self.last_base + step, self.BASE_CENTER)
        elif self.last_base > self.BASE_CENTER:
            self.last_base = max(self.last_base - step, self.BASE_CENTER)

        if self.last_shoulder < self.SHOULDER_CENTER:
            self.last_shoulder = min(self.last_shoulder + step, self.SHOULDER_CENTER)
        elif self.last_shoulder > self.SHOULDER_CENTER:
            self.last_shoulder = max(self.last_shoulder - step, self.SHOULDER_CENTER)

        neutral_elbow = 90
        if self.last_elbow < neutral_elbow:
            self.last_elbow = min(self.last_elbow + step, neutral_elbow)
        elif self.last_elbow > neutral_elbow:
            self.last_elbow = max(self.last_elbow - step, neutral_elbow)

        base = self.clamp(int(self.last_base), self.BASE_MIN, self.BASE_MAX)
        shoulder = self.clamp(int(self.last_shoulder), self.SHOULDER_MIN, self.SHOULDER_MAX)
        elbow = self.clamp(int(self.last_elbow), self.ELBOW_MIN, self.ELBOW_MAX)

        self._send_angles(base, shoulder, elbow)

        # Update OLED to neutral
        if self.current_emotion != "NEUTRAL":
            self.current_emotion = "NEUTRAL"
            self._send_emotion("NEUTRAL")

    # =============================
    # SERIAL: SEND SERVO ANGLES (RATE LIMITED)
    # =============================
    def _send_angles(self, base, shoulder, elbow):
        """
        Send: B:90 S:20 E:120
        Rate-limited to prevent Arduino serial buffer overflow.
        """

        if not self.connected:
            return

        # Rate limit — skip if too soon
        now = time.time()
        if (now - self.last_send_time) < self.MIN_SEND_INTERVAL:
            return

        cmd = f"B:{base} S:{shoulder} E:{elbow}"

        # Skip duplicate commands
        if cmd == self.last_cmd:
            return

        try:
            # Flush any pending output to prevent buffer buildup
            self.ser.reset_output_buffer()
            self.ser.write((cmd + "\n").encode())
            self.last_cmd = cmd
            self.last_send_time = now
            print("[SEND]", cmd)
        except (serial.SerialException, serial.SerialTimeoutException):
            print("[WARN] Serial write failed")

    # =============================
    # SERIAL: OLED EMOTION LABEL
    # =============================
    def _send_emotion(self, emotion):
        """Send: EMO:HAPPY"""

        if not self.connected:
            return

        if emotion == self.last_emo_sent:
            return

        try:
            self.ser.write(f"EMO:{emotion}\n".encode())
            self.last_emo_sent = emotion
            print("[OLED]", emotion)
        except (serial.SerialException, serial.SerialTimeoutException):
            print("[WARN] OLED write failed")

    # =============================
    # FORCE NEUTRAL (for shutdown)
    # =============================
    def neutral(self):
        """Force return to neutral — bypasses rate limiting and dedup."""

        if not self.connected:
            return

        cmd = f"B:{self.BASE_CENTER} S:{self.SHOULDER_CENTER} E:90"

        try:
            self.ser.reset_output_buffer()
            self.ser.write((cmd + "\n").encode())
            time.sleep(0.1)  # ensure Arduino processes it
            self.ser.write(f"EMO:NEUTRAL\n".encode())
            self.last_cmd = cmd
            self.last_emo_sent = "NEUTRAL"
            self.current_emotion = "NEUTRAL"
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
            time.sleep(0.2)  # let Arduino finish moving
            self.ser.close()
            print("[INFO] Serial port closed")
