import yaml
import logging
import sys
import time
import os
import threading

from lager.UAVs import DJI_M600
from lager.UAVs import Helikite
from lager.Gimbals import Gremsy_T7
import lager.UAVs.DJI_M600.parameters as drone_params
import lager.Gimbals.Gremsy_T7.parameters as gimbal_params
import lager.cli as cli
import lager.POI as POI

import lager.parameters as params
from lager.GPIO import LED

# logging configuration with millisecond precision
LOGGING_LEVEL = logging.INFO

class PointingController:
    def __init__(self, config_file, data_folder=None):
        self.config = None

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

        # configure logging
        logging_file = os.path.join(self.data_folder, params.LOGGING_FILE_NAME)
        logging.basicConfig(level=LOGGING_LEVEL, format=params.LOGGING_FORMAT, datefmt=params.LOGGING_DATEFMT, handlers=[
            logging.FileHandler(logging_file)
        ])
        logging.info(f"Controller initialized at {timestamp}")

        # create/update "current" symlink pointing to the new data folder
        # symlink is externally created if data_folder is specified
        if data_folder is None:
            current_link = os.path.join(params.DATA_PATH, params.CURRENT_DATA_FOLDER)
            if os.path.islink(current_link) or os.path.exists(current_link):
                os.remove(current_link)
                logging.info(f"Previous 'current' symlink removed")
            os.symlink(self.data_folder, current_link)
            logging.info(f"'current' symlink updated to point to: {self.data_folder}")

        # load configuration from the specified YAML file
        try:
            with open(config_file, 'r') as file:
                self.config = yaml.safe_load(file)
            
            # copy the configuration file to the data folder for reference
            config_copy_path = os.path.join(self.data_folder, os.path.basename(config_file))
            with open(config_copy_path, 'w') as file:
                yaml.dump(self.config, file)
            logging.info(f"Configuration loaded successfully from {config_file}")
            logging.info(f"Configuration copied to {config_copy_path}")

        except Exception as e:
            logging.error(f"Failed to load configuration: {e}")
            sys.exit(1)

        # check if the configuration is valid
        if 'pointing_controller' not in self.config:
            logging.warning("Configuration file is missing 'pointing_controller' section. Display will be disabled.")
            name = "Default Pointing Controller"
            display_enabled = False
            refresh_rate    = 1
        else:
            if 'name' in self.config['pointing_controller']:
                name = self.config['pointing_controller']['name']
                logging.info(f"Pointing controller name: {name}")
            
            display_enabled = False
            refresh_rate    = 1
            if 'display' in self.config['pointing_controller']:
                if 'enable' in self.config['pointing_controller']['display']:
                    display_enabled = self.config['pointing_controller']['display']['enable']
                    if display_enabled:
                        logging.info("Display enabled in configuration.")
                        if 'refresh_rate' in self.config['pointing_controller']['display']:
                            refresh_rate = self.config['pointing_controller']['display']['refresh_rate']
                            logging.info(f"Display refresh rate: {refresh_rate} Hz")
                    else:
                        logging.info("Display disabled in configuration.")

        # get uav type and configuration
        self.uav = self.get_uav()

        # get gimbal type and configuration
        self.gimbal = self.get_gimbal()

        # get POI object if configured
        self.poi = self.get_POI()

        # start display if enabled in configuration
        if display_enabled:
            if self.uav is not None:
                self.uav_panel = self.uav.render_panel
            else:
                self.uav_panel = lambda: None
            
            if self.gimbal is not None:
                self.gimbal_panel = self.gimbal.render_panel
            else:
                self.gimbal_panel = lambda: None

            if self.poi is not None:
                poi_data = self.poi.get_data
            else:
                poi_data = lambda: None

            display_thread = threading.Thread(target=cli.live_display, args=(self.uav_panel, self.gimbal_panel, logging_file, poi_data, refresh_rate), daemon=True)
            display_thread.start()


    def get_uav(self) -> object:
        # check uav configuration
        if 'uav' not in self.config:
            logging.warning("Configuration file is missing 'uav' section.")
            return None
        
        if 'name' in self.config['uav']:
            name = self.config['uav']['name']
            logging.info(f"UAV type specified in configuration: {name}")
        else:
            logging.warning("UAV configuration is missing 'name' field.")
            return None
        
        if 'simulator' in self.config['uav']:
            simulator = self.config['uav']['simulator']
        else:
            simulator = False

        if 'connection' in self.config['uav']:
            connection_type = self.config['uav']['connection'].get('type', 'unknown')
            connection_protocol = self.config['uav']['connection'].get('protocol', 'unknown')
            connection_port = self.config['uav']['connection'].get('port', 'unknown')
            connection_baudrate = self.config['uav']['connection'].get('baudrate', 'unknown')
            connection_timeout = self.config['uav']['connection'].get('timeout', 'unknown')

            logging.info(f"UAV connection type: {connection_type}")
            logging.info(f"UAV connection protocol: {connection_protocol}")
            logging.info(f"UAV connection port: {connection_port}")
            logging.info(f"UAV connection baudrate: {connection_baudrate}")
            logging.info(f"UAV connection timeout: {connection_timeout}")
        else:
            logging.warning("UAV configuration is missing 'connection' section, which is required for UAV connection.")
            return None
        
        if 'telemetry' in self.config['uav']:
            telemetry_frequency = self.config['uav']['telemetry'].get('frequency', drone_params.TELEMETRY_FREQ)
            logging.info(f"UAV telemetry frequency: {telemetry_frequency} Hz")
        else:
            telemetry_frequency = drone_params.TELEMETRY_FREQ

        
        # check if the drone type is supported
        if name == 'DJI M600':
            uav = DJI_M600.Drone(port=connection_port,
                                 baudrate=connection_baudrate,
                                 timeout=connection_timeout,
                                 telemetry_frequency=telemetry_frequency,
                                 simulator=simulator,
                                 output_dir=self.data_folder)
            return uav
        
        elif name == 'Helikite':
            uav = Helikite.Balloon(gnss_source=None, 
                                   keys=None)
            return uav
        else:
            logging.warning("Unknown UAV type found in configuration.")
            return None
        
    def get_gimbal(self) -> object:
        # check gimbal configuration
        if 'gimbal' not in self.config:
            logging.error("Configuration file is missing 'gimbal' section. A gimbal is required.")
            sys.exit(1)
        
        if 'name' in self.config['gimbal']:
            name = self.config['gimbal']['name']
            logging.info(f"Gimbal type specified in configuration: {name}")
        else:
            logging.error("Gimbal configuration is missing 'name' field. A gimbal name is required.")
            sys.exit(1)
        
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

            logging.info(f"Gimbal connection type: {connection_type}")
            logging.info(f"Gimbal connection protocol: {connection_protocol}")
            logging.info(f"Gimbal connection port: {connection_port}")
            logging.info(f"Gimbal connection baudrate: {connection_baudrate}")
            logging.info(f"Gimbal connection timeout: {connection_timeout}")
        else:
            logging.error("Gimbal configuration is missing 'connection' section. A gimbal connection is required.")
            sys.exit(1)
        
        if 'telemetry' in self.config['gimbal']:
            telemetry_frequency = self.config['gimbal']['telemetry'].get('frequency', None)
            heartbeat_frequency = self.config['gimbal']['telemetry'].get('heartbeat_frequency', gimbal_params.HEARTBEAT_FREQUENCY)
            logging.info(f"Gimbal telemetry frequency: {telemetry_frequency} Hz")
            logging.info(f"Gimbal heartbeat frequency: {heartbeat_frequency} Hz")
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
            logging.error("Unknown gimbal type found in configuration. A supported gimbal is required.")
            sys.exit(1)

    def get_config(self):
        return self.config
    

    def connect(self):
        if self.uav is not None:
            logging.info("Connecting to UAV...")
            self.uav.connect()
        else:
            logging.warning("No UAV configured, skipping connect.")
        
        if self.gimbal is not None:
            logging.info("Connecting to gimbal...")
            self.gimbal.connect()
        else:
            logging.warning("No gimbal configured, skipping connect.")

        if self.led is not None:
            self.led.set(params.LED_STATE_ON)   # solid on = connected, ready

    
    def disconnect(self):
        if self.uav is not None:
            logging.info("Disconnecting from UAV...")
            self.uav.disconnect()
        else:
            logging.warning("No UAV configured, skipping disconnect.")
        
        if self.gimbal is not None:
            logging.info("Disconnecting from gimbal...")
            self.gimbal.disconnect()
        else:
            logging.warning("No gimbal configured, skipping disconnect.")

        if self.led is not None:
            self.led.set(params.LED_STATE_OFF)  # off = disconnected
            self.led.cleanup()

        logging.info("Disconnected.")


    def start_telemetry(self):
        if self.uav is not None:
            logging.info("Starting UAV telemetry logging...")
            self.uav.start_telemetry()
        else:
            logging.warning("No UAV configured, skipping start telemetry.")

        if self.gimbal is not None:
            logging.info("Starting gimbal telemetry logging...")
            self.gimbal.start_telemetry()
        else:
            logging.warning("No gimbal configured, skipping start telemetry.")

        if self.led is not None:
            self.led.set(params.LED_STATE_BLINK)  # blinking = telemetry acquiring
        
    
    def stop_telemetry(self):
        if self.uav is not None:
            logging.info("Stopping UAV telemetry logging...")
            self.uav.stop_telemetry()
        else:
            logging.warning("No UAV configured, skipping stop telemetry.")

        if self.gimbal is not None:
            logging.info("Stopping gimbal telemetry logging...")
            self.gimbal.stop_telemetry()
        else:
            logging.warning("No gimbal configured, skipping stop telemetry.")

        if self.led is not None:
            self.led.set(params.LED_STATE_ON)   # solid on = connected, telemetry stopped

        logging.info("Telemetry logging stopped. Data saved to: " + self.data_folder)


    def get_POI(self):
        # check if POI is configured
        if 'poi' not in self.config:
            logging.warning("No 'poi' section found in configuration. POI tracking will be disabled.")
            self.poi = None
            return None
        
        if 'name' in self.config['poi']:
            name = self.config['poi']['name']
            logging.info(f"POI type specified in configuration: {name}")
        else:
            name = "unknown"

        if 'latitude' in self.config['poi'] and 'longitude' in self.config['poi'] and 'altitude' in self.config['poi']:
            latitude = self.config['poi']['latitude']
            longitude = self.config['poi']['longitude']
            altitude = self.config['poi']['altitude']
            logging.info(f"POI coordinates: lat={latitude}, lon={longitude}, alt={altitude}")
        else:
            logging.warning("POI configuration is missing 'latitude', 'longitude', or 'altitude' fields.")
            self.poi = None
            return None
        if 'max_distance' in self.config['poi']:
            max_distance = self.config['poi']['max_distance']
            logging.info(f"POI max distance: {max_distance} meters")
        else:
            logging.warning("POI configuration is missing 'max_distance' field.")
            max_distance = None
        
        poi = POI.POI(name, latitude, longitude, altitude, max_distance)
        return poi