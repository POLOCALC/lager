import serial
import logging

logger = logging.getLogger(__name__)

class DroneConnection:
    def __init__(self, port, baudrate, timeout):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.serial = None

    def connect(self):
        try:
            self.serial = serial.Serial(self.port, self.baudrate, timeout=self.timeout)
            logger.info(f"Trying to connect to drone on {self.port} at {self.baudrate} baud")
        except Exception as e:
            logger.error(f"Failed to connect to drone: {e}")
            self.serial = None

    def disconnect(self):
        if self.serial and self.serial.is_open:
            self.serial.close()
            logger.info("Disconnected from drone.")

    def is_connected(self):
        return self.serial is not None and self.serial.is_open