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

    def age(self):
        return time.monotonic() - self._last_update if self._last_update else float('inf')
        
    def get_attitude(self):
        with self._lock:
            result = {'yaw': self.state.get('yaw'),
                      'pitch': self.state.get('pitch'),
                      'roll': self.state.get('roll'),
                      'latitude': self.state.get('rtk_lat_deg'),
                      'longitude': self.state.get('rtk_lon_deg'),
                      'altitude': self.state.get('rtk_hfsl_m'),
                      'heading': self.state.get('rtk_yaw_deg')
            }
            return result