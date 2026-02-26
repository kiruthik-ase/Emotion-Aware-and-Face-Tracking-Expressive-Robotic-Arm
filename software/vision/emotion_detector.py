from deepface import DeepFace
import cv2

class EmotionDetector:
    def __init__(self):
        """
        Initializes DeepFace emotion model.
        Model is loaded lazily on first inference.
        """
        pass

    def predict(self, face_bgr):
        """
        Predict emotions from a cropped face image.

        Args:
            face_bgr (numpy array): BGR face image (OpenCV format)

        Returns:
            dict: emotion -> probability (0–1 range)
        """

        if face_bgr is None or face_bgr.size == 0:
            return {}

        try:
            # Convert BGR to RGB (DeepFace expects RGB)
            face_rgb = cv2.cvtColor(face_bgr, cv2.COLOR_BGR2RGB)

            result = DeepFace.analyze(
                face_rgb,
                actions=["emotion"],
                enforce_detection=False,
                detector_backend="skip",  # face already detected
                silent=True
            )

            emotions = result[0]["emotion"]

            # Convert percentages to 0–1 range
            emotions = {k: v / 100.0 for k, v in emotions.items()}

            return emotions

        except Exception:
            return {}
