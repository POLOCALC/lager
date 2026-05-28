

class POI():
    def __init__(self, name, latitude, longitude, altitude):
        self.name = name
        self.latitude = latitude
        self.longitude = longitude
        self.altitude = altitude

    def get_coordinates(self):
        return (self.latitude, self.longitude, self.altitude)