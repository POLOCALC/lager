import threading

class TelemetryState():
    def __init__(self):
        self._lock = threading.Lock()
        self.state = {}

    def update(self, params):
        with self._lock:
            self.state.update(params)   

    def get(self):
        with self._lock:
            return dict(self.state)