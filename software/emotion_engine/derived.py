import time

class DerivedEmotionEngine:
    def __init__(self, bored_time_sec=5.0):
        self.last_non_neutral_time = time.time()
        self.bored_time_sec = bored_time_sec

    def update(self, emotions: dict):
        """
        Returns: derived_emotion or None
        """

        now = time.time()

        happy = emotions.get("happy", 0)
        surprise = emotions.get("surprise", 0)
        neutral = emotions.get("neutral", 0)

        # EXCITED
        if happy > 0.6 and surprise > 0.2:
            return "EXCITED"

        # Track neutral time for BORED
        if neutral < 0.9:
            self.last_non_neutral_time = now

        if neutral > 0.9 and (now - self.last_non_neutral_time) > self.bored_time_sec:
            return "BORED"

        return None
