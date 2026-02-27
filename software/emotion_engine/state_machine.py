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

        # If the model is uncertain (None), vote for NEUTRAL rather than
        # repeating the last emotion. Repeating makes the system "stick" (e.g.
        # once HAPPY becomes stable, it can stay HAPPY forever).
        if emotion is None:
            self.history.append("NEUTRAL")
        else:
            self.history.append(emotion)

        # Count votes
        counts: dict[str, int] = {}
        for e in self.history:
            counts[e] = counts.get(e, 0) + 1

        # Find dominant emotion
        dominant = max(counts, key=lambda k: counts[k])

        if counts[dominant] >= self.threshold:
            self.current_emotion = dominant

        return self.current_emotion
