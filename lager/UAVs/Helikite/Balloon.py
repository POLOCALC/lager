import logging

# use the Controller.py logging configuration
logger = logging.getLogger(__name__)

class Balloon:
    def __init__(self, gnss_source=None, keys=None):
        """
        Initialize the Balloon class.
        gnss_source: Optional [str]
            Path to the GNSS source json file
        keys: Optional [list]
            List of keys for reading the GNSS source json file. If None, all keys will be read.
        """
        self.gnss_source = gnss_source
        self.keys = keys
        if self.gnss_source is not None:
            logger.info(f"Balloon initialized with GNSS source: {self.gnss_source}.")
        else:
            logger.info(f"Balloon initialized without GNSS source.")

    def connect(self):
        """
        Connect to the balloon's GNSS source.
        """
        logger.info("Connecting to balloon GNSS source: to be done")
        # Implement connection logic here
        # For example, open a serial port or establish a network connection

    def disconnect(self):
        """
        Disconnect from the balloon's GNSS source.
        """
        logger.info("Disconnecting from balloon GNSS source: to be done")
        # Implement disconnection logic here
        # For example, close a serial port or terminate a network connection

    def start_telemetry(self):
        """
        Start telemetry logging for the balloon.
        """
        logger.info("Starting balloon telemetry logging...")
        # Implement telemetry logging logic here
        # For example, read GNSS data from the source and log it
        if self.gnss_source is not None:
            logger.info(f"Reading GNSS data from {self.gnss_source}...")
            # Add logic to read and log GNSS data
        else:
            logger.warning("No GNSS source provided. Telemetry logging may be limited.")

    def stop_telemetry(self):
        """
        Stop telemetry logging for the balloon.
        """
        logger.info("Stopping balloon telemetry logging: to be done")
        # Implement logic to stop telemetry logging here

    def render_panel(self):
        """
        Render the balloon's telemetry panel.
        """
        logger.info("Rendering balloon telemetry panel: to be done")
        # Implement logic to render the telemetry panel here