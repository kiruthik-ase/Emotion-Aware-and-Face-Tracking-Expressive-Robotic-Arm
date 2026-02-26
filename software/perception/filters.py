from collections import deque
import time

class EmotionFilter:
    def __init__(
        self,
        window_size=12,
        min_confidence=0.55,
        dominance_margin=0.15
    ):
        self.window = deque(maxlen=window_size)
        self.min_confidence = min_confidence
        self.dominance_margin = dominance_margin

    def update(self, emotions: dict):
        """
        emotions: dict[str, float]
        returns: (emotion_name | None, confidence)
        """

        if not emotions:
            return None, 0.0

        # Sort by confidence
        sorted_emotions = sorted(
            emotions.items(),
            key=lambda x: x[1],
            reverse=True
        )

        top_emotion, top_conf = sorted_emotions[0]
        second_emotion, second_conf = sorted_emotions[1]

        # Confidence gate
        if top_conf < self.min_confidence:
            return None, 0.0

        # Neutral suppression
        if top_emotion == "neutral":
            if second_conf > (top_conf - self.dominance_margin):
                top_emotion, top_conf = second_emotion, second_conf

        # Dominance check
        if (top_conf - second_conf) < self.dominance_margin:
            return None, 0.0

        self.window.append(top_emotion)

        return top_emotion, top_conf
