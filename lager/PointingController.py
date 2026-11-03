import yaml
import logging
import sys
import time
import os

from lager.UAVs import DJI_M600, DJI_M400

from lager.Gimbals import Gremsy_T7
import lager.Gimbals.Gremsy_T7.parameters as gimbal_params

import lager.POI as POI

import lager.parameters as params
from lager.GPIO import LED

# logging configuration with millisecond precision
LOGGING_LEVEL = logging.INFO

class PointingController:
    def __init__(self, configuration, data_folder=None, verbose=False):
        self.config = None
        self.verbose = verbose

        # initialise status LED early so it can signal progress
        if params.ENABLE_LED_INDICATOR:
            self.led = LED(pin=params.LED_INDICATOR_PIN)
            self.led.set(params.LED_STATE_ON)   # solid on = initialising
        else:
            self.led = None

        # create timestamp
        timestamp = time.strftime("%Y%m%d_%H%M%S", time.gmtime())

        # create data folder
        if data_folder is not None:
            self.data_folder = data_folder
        else:
            self.data_folder = f"{params.DATA_FOLDER_PREFIX}_{timestamp}"
            self.data_folder = os.path.join(params.DATA_PATH, self.data_folder)
        
        # check data path
        if not os.path.exists(self.data_folder):
            os.makedirs(self.data_folder)

        # configure logging only if data_folder is not externally specified (to avoid overwriting existing logs)
        logging_file = os.path.join(self.data_folder, params.LOGGING_FILE_NAME)
        if not logging.getLogger().hasHandlers():
            logger = logging.getLogger()
            logger.setLevel(getattr(logging, params.LOGGING_LEVEL))
            if self.verbose:
                handlers = [logging.StreamHandler(sys.stdout), 
                            logging.FileHandler(logging_file)]
            else:
                handlers = [logging.FileHandler(logging_file)]
            for handler in handlers:
                handler.setFormatter(logging.Formatter(params.LOGGING_FORMAT, params.LOGGING_DATEFMT))
                logger.addHandler(handler)

        self.logger = logging.getLogger(__name__)

        self.logger.info(f"Controller initialized at {timestamp}")
        # create/update "current" symlink pointing to the new data folder
        # symlink is externally created if data_folder is specified
        if data_folder is None:
            current_link = os.path.join(params.DATA_PATH, params.CURRENT_DATA_FOLDER)
            if os.path.islink(current_link) or os.path.exists(current_link):
                os.remove(current_link)
                self.logger.info(f"Previous 'current' symlink removed")
            os.symlink(self.data_folder, current_link)
            self.logger.info(f"'current' symlink updated to point to: {self.data_folder}")

        # load configuration from the specified YAML file or dictionary
        try:
            if isinstance(configuration, dict):
                self.config = configuration
                self.logger.info("Configuration loaded from provided dictionary")
            else:
                with open(configuration, 'r') as file:
                    self.config = yaml.safe_load(file)
                self.logger.info(f"Configuration loaded successfully from {configuration}")
            
                # copy the configuration file to the data folder for reference
                config_copy_path = os.path.join(self.data_folder, os.path.basename(configuration))
                with open(config_copy_path, 'w') as file:
                    yaml.dump(self.config, file)
                self.logger.info(f"Configuration copied to {config_copy_path}")

        except Exception as e:
            self.logger.error(f"Failed to load configuration: {e}")

        # check if the configuration is valid
        if 'name' not in self.config:
            self.logger.info("Configuration file is missing 'name' field.")
            name = "Default Pointing Controller"

        # get uav type and configuration
        self.uav = self.get_uav()

        # get gimbal type and configuration
        self.gimbal = self.get_gimbal()

        # get POI object if configured
        self.poi = self.get_POI()

    def get_uav(self) -> object:
        # check uav configuration
        if 'uav' not in self.config:
            self.logger.warning("Configuration file is missing 'uav' section.")
            return
        
        if 'name' in self.config['uav']:
            name = self.config['uav']['name']
            self.logger.info(f"UAV type specified in configuration: {name}")
        else:
            self.logger.warning("UAV configuration is missing 'name' field.")
            return
        
        if 'simulator' in self.config['uav']:
            simulator = self.config['uav']['simulator']
        else:
            simulator = False

        if 'connection' in self.config['uav']:
            connection_type = self.config['uav']['connection'].get('type', 'unknown')
            connection_protocol = self.config['uav']['connection'].get('protocol', 'unknown')
            connection_port = self.config['uav']['connection'].get('port', None)
            connection_baudrate = self.config['uav']['connection'].get('baudrate', None)
            connection_timeout = self.config['uav']['connection'].get('timeout', None)

            self.logger.info(f"UAV connection type: {connection_type}")
            self.logger.info(f"UAV connection protocol: {connection_protocol}")
            self.logger.info(f"UAV connection port: {connection_port}")
            self.logger.info(f"UAV connection baudrate: {connection_baudrate}")
            self.logger.info(f"UAV connection timeout: {connection_timeout}")
        else:
            self.logger.warning("UAV configuration is missing 'connection' section, which is required for UAV connection.")
            return

        if 'telemetry' in self.config['uav']:
            telemetry_frequency = self.config['uav']['telemetry'].get('frequency', None)
            if telemetry_frequency is not None:
                self.logger.info(f"UAV telemetry frequency: {telemetry_frequency} Hz")

            telemetry_frequency_slow = self.config['uav']['telemetry'].get('slow_frequency', None)
            if telemetry_frequency_slow is not None:
                self.logger.info(f"UAV telemetry slow frequency: {telemetry_frequency_slow} Hz")

            telemetry_frequency_imu = self.config['uav']['telemetry'].get('imu_frequency', None)
            if telemetry_frequency_imu is not None:
                self.logger.info(f"UAV telemetry IMU frequency: {telemetry_frequency_imu} Hz")

            telemetry_frequency_status = self.config['uav']['telemetry'].get('status_frequency', None)
            if telemetry_frequency_status is not None:
                self.logger.info(f"UAV telemetry status frequency: {telemetry_frequency_status} Hz")

            telemetry_stats_s = self.config['uav']['telemetry'].get('stats_s', None)
            if telemetry_stats_s is not None:
                self.logger.info(f"UAV telemetry stats interval: {telemetry_stats_s} seconds")

            telemetry_binary = self.config['uav']['telemetry'].get('binary', None)
            if telemetry_binary is not None:
                self.logger.info(f"UAV telemetry binary: {telemetry_binary}")
        else:
            self.logger.warning("UAV configuration is missing 'telemetry' section, which is required for telemetry settings.")
            return

        # check if the drone type is supported
        if name == 'DJI M600':
            self.logger.info(f"UAV telemetry frequency: {telemetry_frequency} Hz")
            uav = DJI_M600.Drone(device=connection_port,
                                 baudrate=connection_baudrate,
                                 timeout=connection_timeout,
                                 telemetry_frequency=telemetry_frequency,
                                 simulator=simulator,
                                 output_dir=self.data_folder)
            return uav
        elif name == 'DJI M400':
            uav = DJI_M400.Drone(device=connection_port,
                                 binary=telemetry_binary,
                                 fast_hz=telemetry_frequency,
                                 slow_hz=telemetry_frequency_slow,
                                 imu_hz=telemetry_frequency_imu,
                                 status_hz=telemetry_frequency_status,
                                 socket_path=self.config['uav']['connection'].get('socket', None),
                                 stats_s=telemetry_stats_s,
                                 connect_timeout=connection_timeout,
                                 simulator=simulator,
                                 output_dir=self.data_folder)
            return uav
        else:
            self.logger.warning("Unknown UAV type found in configuration.")
            return None
        
    def get_gimbal(self) -> object:
        # check gimbal configuration
        if 'gimbal' not in self.config:
            self.logger.error("Configuration file is missing 'gimbal' section. A gimbal is required.")
            return
        
        if 'name' in self.config['gimbal']:
            name = self.config['gimbal']['name']
            self.logger.info(f"Gimbal type specified in configuration: {name}")
        else:
            self.logger.error("Gimbal configuration is missing 'name' field. A gimbal name is required.")
            return
        
        if 'simulator' in self.config['gimbal']:
            simulator = self.config['gimbal']['simulator']
        else:
            simulator = False

        if 'connection' in self.config['gimbal']:
            connection_type = self.config['gimbal']['connection'].get('type', 'unknown')
            connection_protocol = self.config['gimbal']['connection'].get('protocol', 'unknown')
            connection_port = self.config['gimbal']['connection'].get('port', 'unknown')
            connection_baudrate = self.config['gimbal']['connection'].get('baudrate', 'unknown')
            connection_timeout = self.config['gimbal']['connection'].get('timeout', 'unknown')

            self.logger.info(f"Gimbal connection type: {connection_type}")
            self.logger.info(f"Gimbal connection protocol: {connection_protocol}")
            self.logger.info(f"Gimbal connection port: {connection_port}")
            self.logger.info(f"Gimbal connection baudrate: {connection_baudrate}")
            self.logger.info(f"Gimbal connection timeout: {connection_timeout}")
        else:
            self.logger.error("Gimbal configuration is missing 'connection' section. A gimbal connection is required.")
            return
        
        if 'telemetry' in self.config['gimbal']:
            telemetry_frequency = self.config['gimbal']['telemetry'].get('frequency', None)
            heartbeat_frequency = self.config['gimbal']['telemetry'].get('heartbeat_frequency', gimbal_params.HEARTBEAT_FREQUENCY)
            self.logger.info(f"Gimbal telemetry frequency: {telemetry_frequency} Hz")
            self.logger.info(f"Gimbal heartbeat frequency: {heartbeat_frequency} Hz")
        else:
            telemetry_frequency = None
            heartbeat_frequency = gimbal_params.HEARTBEAT_FREQUENCY

        
        # check if the gimbal type is supported
        if name == 'Gremsy T7':
            gimbal = Gremsy_T7.Gimbal(port=connection_port,
                                      baudrate=connection_baudrate,
                                      timeout=connection_timeout,
                                      heartbeat_frequency=heartbeat_frequency,
                                      simulator=simulator,
                                      output_dir=self.data_folder)
            return gimbal
        
        else:
            self.logger.error("Unknown gimbal type found in configuration. A supported gimbal is required.")
            pass

    def get_config(self):
        return self.config
    

    def connect(self):
        if self.uav is not None:
            self.logger.info("Connecting to UAV...")
            self.uav.connect()
        else:
            self.logger.warning("No UAV configured, skipping connect.")
        
        if self.gimbal is not None:
            self.logger.info("Connecting to gimbal...")
            self.gimbal.connect()
        else:
            self.logger.warning("No gimbal configured, skipping connect.")

        if self.led is not None:
            self.led.set(params.LED_STATE_ON)   # solid on = connected, ready

    
    def disconnect(self):
        if self.uav is not None:
            self.logger.info("Disconnecting from UAV...")
            self.uav.disconnect()
        else:
            self.logger.warning("No UAV configured, skipping disconnect.")
        
        if self.gimbal is not None:
            self.logger.info("Disconnecting from gimbal...")
            self.gimbal.disconnect()
        else:
            self.logger.warning("No gimbal configured, skipping disconnect.")

        if self.led is not None:
            self.led.set(params.LED_STATE_OFF)  # off = disconnected
            self.led.cleanup()

        self.logger.info("Disconnected.")


    def start_telemetry(self):
        if self.uav is not None:
            self.logger.info("Starting UAV telemetry logging...")
            self.uav.start_telemetry()
        else:
            self.logger.warning("No UAV configured, skipping start telemetry.")

        if self.gimbal is not None:
            self.logger.info("Starting gimbal telemetry logging...")
            self.gimbal.start_telemetry()
        else:
            self.logger.warning("No gimbal configured, skipping start telemetry.")

        if self.led is not None:
            self.led.set(params.LED_STATE_BLINK)  # blinking = telemetry acquiring
        
    
    def stop_telemetry(self):
        if self.uav is not None:
            self.logger.info("Stopping UAV telemetry logging...")
            self.uav.stop_telemetry()
        else:
            self.logger.warning("No UAV configured, skipping stop telemetry.")

        if self.gimbal is not None:
            self.logger.info("Stopping gimbal telemetry logging...")
            self.gimbal.stop_telemetry()
        else:
            self.logger.warning("No gimbal configured, skipping stop telemetry.")

        if self.led is not None:
            self.led.set(params.LED_STATE_ON)   # solid on = connected, telemetry stopped

        self.logger.info("Telemetry logging stopped. Data saved to: " + self.data_folder)


    def get_POI(self):
        # check if POI is configured
        if 'poi' not in self.config:
            self.logger.warning("No 'poi' section found in configuration. POI tracking will be disabled.")
            self.poi = None
            return None
        
        if 'name' in self.config['poi']:
            name = self.config['poi']['name']
            self.logger.info(f"POI type specified in configuration: {name}")
        else:
            name = "unknown"

        if 'latitude' in self.config['poi'] and 'longitude' in self.config['poi'] and 'altitude' in self.config['poi']:
            latitude = self.config['poi']['latitude']
            longitude = self.config['poi']['longitude']
            altitude = self.config['poi']['altitude']
            self.logger.info(f"POI coordinates: lat={latitude}, lon={longitude}, alt={altitude}")
        else:
            self.logger.warning("POI configuration is missing 'latitude', 'longitude', or 'altitude' fields.")
            self.poi = None
            return None
        if 'max_distance' in self.config['poi']:
            max_distance = self.config['poi']['max_distance']
            self.logger.info(f"POI max distance: {max_distance} meters")
        else:
            self.logger.warning("POI configuration is missing 'max_distance' field.")
            max_distance = None
        if 'roll_deg' in self.config['poi']:
            try:
                roll_deg = float(self.config['poi']['roll_deg'])
                self.logger.info(f"POI roll: {roll_deg}°")
            except (TypeError, ValueError):
                self.logger.warning(f"Invalid POI 'roll_deg' value: {self.config['poi']['roll_deg']!r}; defaulting to 0°")
                roll_deg = 0.
        else:
            self.logger.info("POI configuration is missing 'roll_deg' field; defaulting to 0°")
            roll_deg = 0.

        poi = POI.POI(name, latitude, longitude, altitude, max_distance, roll_deg)
        return poi