import threading
import time

class TelemetryState():
    def __init__(self):
        self._lock = threading.Lock()
        self.state = {}
        self._last_update = None

    def update(self, params):
        with self._lock:
            self.state.update(params)
            self._last_update = time.monotonic()

    def get(self):
        with self._lock:
            return dict(self.state)

    def age(self) -> float:
        """Seconds since the last message from the gimbal (inf if none yet)."""
        with self._lock:
            return float("inf") if self._last_update is None else time.monotonic() - self._last_update
