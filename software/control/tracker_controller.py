import time
import serial


class TrackerController:
    """
    FINAL TRACKER CONTROLLER (Stable + OLED)

    ✅ Servo movement stays smooth (Arduino handles smoothing)
    ✅ Face tracking aggressive but safe
    ✅ Emotion is sent independently to OLED
    ✅ Works perfectly with:

        B:90 S:20 E:120
        EMO:HAPPY
    """

    def __init__(self, port="COM8", baud=115200):

        # =============================
        # SERIAL CONNECTION
        # =============================
        print("[INFO] Connecting to Arduino...")

        self.ser = serial.Serial(port, baud, timeout=1)
        time.sleep(2)

        print("[INFO] Arduino connected successfully on", port)

        # =============================
        # TRACKING PARAMETERS
        # =============================
        self.BASE_CENTER = 90
        self.SHOULDER_CENTER = 20

        # Aggressive tracking (obsessed mode)
        self.TRACK_GAIN_X = 0.20
        self.TRACK_GAIN_Y = 0.16
        self.DEADZONE = 10

        # =============================
        # SAFE SERVO LIMITS (YOUR REAL)
        # =============================
        self.BASE_MIN, self.BASE_MAX = 60, 120
        self.SHOULDER_MIN, self.SHOULDER_MAX = 0, 60
        self.ELBOW_MIN, self.ELBOW_MAX = 30, 150

        # =============================
        # EMOTION → ELBOW MAPPING
        # =============================
        self.EMOTION_ELBOW = {
            "HAPPY": 130,
            "SAD": 60,
            "ANGRY": 40,
            "SURPRISE": 150,
            "NEUTRAL": 90,
            "BORED": 80
        }

        # Avoid repeating same servo command
        self.last_cmd = None
        self.last_emo = None

    # =============================
    # UTILS
    # =============================
    def clamp(self, val, vmin, vmax):
        return max(vmin, min(val, vmax))

    # =============================
    # FACE TRACKING ANGLES
    # =============================
    def compute_tracking_angles(self, face_x, face_y, frame_w, frame_h):
        """
        Compute base + shoulder angles from face position
        """

        cx = frame_w // 2
        cy = frame_h // 2

        error_x = face_x - cx
        error_y = face_y - cy

        # Deadzone removes jitter
        if abs(error_x) < self.DEADZONE:
            error_x = 0
        if abs(error_y) < self.DEADZONE:
            error_y = 0

        # Aggressive tracking response
        base = self.BASE_CENTER + error_x * self.TRACK_GAIN_X
        shoulder = self.SHOULDER_CENTER - error_y * self.TRACK_GAIN_Y

        # Clamp safe limits
        base = self.clamp(base, self.BASE_MIN, self.BASE_MAX)
        shoulder = self.clamp(shoulder, self.SHOULDER_MIN, self.SHOULDER_MAX)

        return base, shoulder

    # =============================
    # EMOTION → ELBOW ANGLE
    # =============================
    def emotion_to_elbow(self, emotion):
        """
        Convert emotion label to elbow servo target
        """

        elbow = self.EMOTION_ELBOW.get(emotion, 90)
        elbow = self.clamp(elbow, self.ELBOW_MIN, self.ELBOW_MAX)

        return elbow

    # =============================
    # SEND SERVO ANGLES ONLY
    # =============================
    def send_angles(self, base, shoulder, elbow):
        """
        Send ONLY servo angles:

        B:90 S:20 E:120
        """

        cmd = f"B:{base:.0f} S:{shoulder:.0f} E:{elbow:.0f}"

        # Avoid sending duplicates
        if cmd == self.last_cmd:
            return

        try:
            self.ser.write((cmd + "\n").encode())
        except serial.SerialTimeoutException:
            return
        self.last_cmd = cmd

        print("[SEND SERVO]", cmd)

    # =============================
    # SEND EMOTION TO OLED ALWAYS
    # =============================
    def send_emotion(self, emotion):
        """
        Send emotion separately:

        EMO:HAPPY
        """

        if emotion == self.last_emo:
            return

        emo_cmd = f"EMO:{emotion}\n"
        try:
            self.ser.write(emo_cmd.encode())
        except serial.SerialTimeoutException:
            return
        self.last_emo = emotion

        print("[SEND EMO]", emotion)

    # =============================
    # NEUTRAL SAFE RESET
    # =============================
    def neutral(self):
        """
        Return robot to neutral pose + neutral face
        """

        self.send_angles(self.BASE_CENTER, self.SHOULDER_CENTER, 90)
        self.send_emotion("NEUTRAL")

    # =============================
    # CLEAN EXIT
    # =============================
    def close(self):
        """
        Close serial safely
        """

        print("[INFO] Closing Serial + Reset Neutral Pose")
        self.neutral()
        self.ser.close()
