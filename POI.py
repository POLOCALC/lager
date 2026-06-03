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
        self.is_tracking = False
        self.tracking_thread = None

    def get_coordinates(self):
        return (self.latitude, self.longitude, self.altitude)
    
    def start_tracking(self, uav, gimbal, forward_heading=True):
        logger.info(f"Starting tracking of POI: {self.name}")
        self.is_tracking = True
        self.tracking_thread = threading.Thread(target=self._track_poi, args=(uav, gimbal, forward_heading))
        self.tracking_thread.start()

    def _track_poi(self, uav, gimbal, forward_heading):
        while True:
            uav_attitude = uav.telemetry_state.get_attitude()
            uav_lat = uav_attitude.get('latitude')
            uav_lon = uav_attitude.get('longitude')
            uav_alt = uav_attitude.get('altitude')
            uav_heading = uav_attitude.get('heading')
            uav_pitch = uav_attitude.get('pitch')
            uav_roll = uav_attitude.get('roll')
            uav_yaw = uav_attitude.get('yaw')

            distance = self.distance(uav_lat, uav_lon, uav_alt)
            if distance > self.max_distance:
                logger.debug(f"POI {self.name} is out of range (distance: {distance:.2f} m). Not tracking.")
                time.sleep(1)  # check again in 1 second
                continue
            else:
                # compute gimbal corrections to point at the POI
                if forward_heading:
                    gimbal_yaw = 0.0
                else:
                    # hehehe
                    gimbal_yaw = 0.0
                    pass

                # roll is not used for pointing, but we can set it to 0 to keep the horizon level
                gimbal_roll = 0.0

                # compute pitch correction based on uav and POI world coordinates
                az, el, rng = pm.geodetic2aer(self.latitude, self.longitude, self.altitude,
                                              uav_lat, uav_lon, uav_alt,
                                              deg=True)
                gimbal_pitch = -el

                # set the gimbal orientation to point at the POI
                gimbal.goto(yaw=gimbal_yaw, pitch=gimbal_pitch, roll=gimbal_roll, wait=False)

                time.sleep(0.1)  # update at 10 Hz

    def stop_tracking(self):
        logger.info(f"Stopping tracking of POI: {self.name}")
        self.is_tracking = False
        if self.tracking_thread:
            self.tracking_thread.join()
            self.tracking_thread = None
            logger.info(f"Tracking of POI {self.name} stopped.")

    def distance(self, uav_lat, uav_lon, uav_alt):
        # calculate distance between uav and POI using pymap3d
        poi_ecef = pm.geodetic2ecef(self.latitude, self.longitude, self.altitude)
        uav_ecef = pm.geodetic2ecef(uav_lat, uav_lon, uav_alt)
        distance = ((poi_ecef[0] - uav_ecef[0]) ** 2 + (poi_ecef[1] - uav_ecef[1]) ** 2 + (poi_ecef[2] - uav_ecef[2]) ** 2) ** 0.5
        return distance