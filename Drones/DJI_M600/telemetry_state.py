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
        
    def get_attitude(self):
        with self._lock:
            dict = {'yaw': self.state.get('yaw'),
                    'pitch': self.state.get('pitch'),
                    'roll': self.state.get('roll'),
                    'latitude': self.state.get('rtk_lat_deg'),
                    'longitude': self.state.get('rtk_lon_deg'),
                    'altitude': self.state.get('alt_m'),
                    'heading': self.state.get('rtk_yaw_deg')
            }
            return dict