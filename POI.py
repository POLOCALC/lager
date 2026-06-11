import logging
import threading
import time
import pymap3d as pm

logger = logging.getLogger(__name__)

class POI():
    def __init__(self, name, latitude, longitude, altitude, max_distance):
        self.name = name
        self.latitude = latitude
        self.longitude = longitude
        self.altitude = altitude
        self.max_distance = max_distance
        self.current_distance = None
        self.is_tracking = False
        self.tracking_thread = None
        self._stop_tracking_event = threading.Event()
        self.poi_ecef = pm.geodetic2ecef(self.latitude, self.longitude, self.altitude)

    def get_coordinates(self):
        return (self.latitude, self.longitude, self.altitude)
    
    def start_tracking(self, uav, gimbal, forward_heading=True):
        logger.info(f"Starting tracking of POI: {self.name}")
        self._stop_tracking_event.clear()
        self.is_tracking = True
        self.tracking_thread = threading.Thread(target=self._track_poi, args=(uav, gimbal, forward_heading), daemon=True)
        self.tracking_thread.start()

    def _track_poi(self, uav, gimbal, forward_heading):
        while not self._stop_tracking_event.is_set():
            uav_attitude = uav.telemetry_state.get_attitude()
            uav_lat = uav_attitude.get('latitude')
            uav_lon = uav_attitude.get('longitude')
            uav_alt = uav_attitude.get('altitude')
            uav_heading = uav_attitude.get('heading')
            uav_pitch = uav_attitude.get('pitch')
            uav_roll = uav_attitude.get('roll')
            uav_yaw = uav_attitude.get('yaw')

            if None in [uav_lat, uav_lon, uav_alt, uav_heading, uav_pitch, uav_roll, uav_yaw]:
                logger.warning(f"Missing UAV telemetry data. Cannot track POI {self.name}.")
                self._stop_tracking_event.wait(timeout=1)
                continue

            self.current_distance = self.distance(uav_lat, uav_lon, uav_alt)
            if self.current_distance > self.max_distance:
                logger.warning(f"POI {self.name} is out of range (distance: {self.current_distance:.2f} m). Not tracking.")
                self.is_tracking = False
                # wait up to 5 s but wake immediately if stop() is called
                self._stop_tracking_event.wait(timeout=5)
                continue
            else:
                self.is_tracking = True

                # compute gimbal corrections to point at the POI
                if forward_heading:
                    gimbal_yaw = 0.0
                else:
                    # TODO
                    gimbal_yaw = 0.0
                    pass

                # roll is not used for pointing, but we can set it to 0 to keep the horizon level
                gimbal_roll = 0.0

                # compute pitch correction based on uav and POI world coordinates
                az, el, rng = pm.geodetic2aer(self.latitude, self.longitude, self.altitude,
                                              uav_lat, uav_lon, uav_alt,
                                              deg=True)
                gimbal_pitch = el

                # set the gimbal orientation to point at the POI
                gimbal.goto(yaw=gimbal_yaw, pitch=gimbal_pitch, roll=gimbal_roll, wait=False)

                self._stop_tracking_event.wait(timeout=0.1)  # update at ~10 Hz

    def stop_tracking(self):
        logger.info(f"Stopping tracking of POI: {self.name}")
        self.is_tracking = False
        self._stop_tracking_event.set()  # wakes any wait() in the tracking loop immediately
        if self.tracking_thread:
            self.tracking_thread.join()
            self.tracking_thread = None
            logger.info(f"Tracking of POI {self.name} stopped.")

    def distance(self, uav_lat, uav_lon, uav_alt):
        # calculate distance between uav and POI using pymap3d
        uav_ecef = pm.geodetic2ecef(uav_lat, uav_lon, uav_alt)
        distance = ((self.poi_ecef[0] - uav_ecef[0]) ** 2 + (self.poi_ecef[1] - uav_ecef[1]) ** 2 + (self.poi_ecef[2] - uav_ecef[2]) ** 2) ** 0.5
        return distance
    
    def get_data(self):
        return {
            'name': self.name,
            'latitude': self.latitude,
            'longitude': self.longitude,
            'altitude': self.altitude,
            'max_distance': self.max_distance,
            'current_distance': self.current_distance,
            'is_tracking': self.is_tracking
        }