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
        sorted_emotions = sorted(emotions.items(), key=lambda x: x[1], reverse=True)

        top_emotion, top_conf = sorted_emotions[0]
        if len(sorted_emotions) >= 2:
            _, second_conf = sorted_emotions[1]
        else:
            second_conf = 0.0

        # Confidence gate
        if top_conf < self.min_confidence:
            return None, 0.0

        # Dominance check
        if (top_conf - second_conf) < self.dominance_margin:
            return None, 0.0

        self.window.append(top_emotion)

        return top_emotion, top_conf
