from collections import deque

class EmotionStateMachine:
    def __init__(self, window_size=12, threshold=7):
        self.history = deque(maxlen=window_size)
        self.current_emotion = "NEUTRAL"
        self.threshold = threshold

    def update(self, emotion):
        """
        emotion: str | None
        returns: stable emotion
        """

        if emotion is None:
            self.history.append(self.current_emotion)
        else:
            self.history.append(emotion)

        # Count votes
        counts = {}
        for e in self.history:
            counts[e] = counts.get(e, 0) + 1

        # Find dominant emotion
        dominant = max(counts, key=counts.get)

        if counts[dominant] >= self.threshold:
            self.current_emotion = dominant

        return self.current_emotion
