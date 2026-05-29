import yaml
import logging
import sys
import time
import os
import threading

from Drones import DJI_M600
from Gimbals import Gremsy_T7
import cli
import POI

DATA_PATH = 'data'

# logging configuration with millisecond precision
LOGGING_LEVEL = logging.INFO
LOGGING_FORMAT = '[%(asctime)s.%(msecs)03d %(levelname)s] (%(name)s): %(message)s'
LOGGING_DATEFMT = '%Y-%m-%d %H:%M:%S'

class Controller:
    def __init__(self, config_file):
        self.config = None

        # create timestamp
        timestamp = time.strftime("%Y%m%d_%H%M%S")

        # create data folder
        self.data_folder = f"flight_controller_{timestamp}"
        self.data_folder = os.path.join(DATA_PATH, self.data_folder)
        # check data path
        if not os.path.exists(self.data_folder):
            os.makedirs(self.data_folder)

        # create logging file with timestamp
        timestamp = time.strftime("%Y%m%d_%H%M%S")
        logging_file = os.path.join(self.data_folder, f"controller_{timestamp}.log")
        logging.basicConfig(level=LOGGING_LEVEL, format=LOGGING_FORMAT, datefmt=LOGGING_DATEFMT, handlers=[
            logging.FileHandler(logging_file)
        ])

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
        if 'Controller' not in self.config:
            logging.error("Configuration file is missing 'Controller' section.")
            sys.exit(1)

        if 'name' in self.config['Controller']:
            name = self.config['Controller']['name']
            logging.info(f"Controller name: {name}")
        
        if 'display' in self.config['Controller']:
            if 'enable' in self.config['Controller']['display']:
                display_enabled = self.config['Controller']['display']['enable']
                if display_enabled:
                    logging.info("Display enabled in configuration.")
                    if 'refresh_rate' in self.config['Controller']['display']:
                        refresh_rate = self.config['Controller']['display']['refresh_rate']
                        logging.info(f"Display refresh rate: {refresh_rate} Hz")
                else:
                    logging.info("Display disabled in configuration.")

        # get drone type and configuration
        self.drone = self.get_drone()

        # get gimbal type and configuration
        self.gimbal = self.get_gimbal()

        # get POI object if configured
        self.poi = None

        # start display if enabled in configuration
        if display_enabled:
            if self.drone is not None:
                self.drone_panel = self.drone.render_panel
            else:
                self.drone_panel = lambda: None
            
            if self.gimbal is not None:
                self.gimbal_panel = self.gimbal.render_panel
            else:
                self.gimbal_panel = lambda: None

            display_thread = threading.Thread(target=cli.live_display, args=(self.drone_panel, self.gimbal_panel, logging_file, refresh_rate))
            display_thread.start()


    def get_drone(self) -> object:
        # check drone configuration
        if 'Drone' not in self.config:
            logging.warning("Configuration file is missing 'Drone' section.")
            return None
        
        if 'name' in self.config['Drone']:
            name = self.config['Drone']['name']
            logging.info(f"Drone type specified in configuration: {name}")
        else:
            logging.warning("Drone configuration is missing 'name' field.")
            return None
        
        if 'simulator' in self.config['Drone']:
            simulator = self.config['Drone']['simulator']

        if 'connection' in self.config['Drone']:
            connection_type = self.config['Drone']['connection'].get('type', 'unknown')
            connection_protocol = self.config['Drone']['connection'].get('protocol', 'unknown')
            connection_port = self.config['Drone']['connection'].get('port', 'unknown')
            connection_baudrate = self.config['Drone']['connection'].get('baudrate', 'unknown')
            connection_timeout = self.config['Drone']['connection'].get('timeout', 'unknown')

            logging.info(f"Drone connection type: {connection_type}")
            logging.info(f"Drone connection protocol: {connection_protocol}")
            logging.info(f"Drone connection port: {connection_port}")
            logging.info(f"Drone connection baudrate: {connection_baudrate}")
            logging.info(f"Drone connection timeout: {connection_timeout}")
        else:
            logging.warning("Drone configuration is missing 'connection' section, which is required for drone connection.")
            return None
        
        if 'telemetry' in self.config['Drone']:
            telemetry_frequency = self.config['Drone']['telemetry'].get('frequency', None)
            logging.info(f"Drone telemetry frequency: {telemetry_frequency} Hz")

        
        # check if the drone type is supported
        if name == 'DJI M600':
            drone = DJI_M600.Drone(port=connection_port,
                                   baudrate=connection_baudrate,
                                   timeout=connection_timeout,
                                   telemetry_frequency=telemetry_frequency,
                                   simulator=simulator,
                                   output_dir=self.data_folder)
            return drone
        
        else:
            logging.warning("Unknown drone type found in configuration.")
            return None
        
    def get_gimbal(self) -> object:
        # check gimbal configuration
        if 'Gimbal' not in self.config:
            logging.warning("Configuration file is missing 'Gimbal' section.")
            return None
        
        if 'name' in self.config['Gimbal']:
            name = self.config['Gimbal']['name']
            logging.info(f"Gimbal type specified in configuration: {name}")
        else:
            logging.warning("Gimbal configuration is missing 'name' field.")
            return None
        
        if 'simulator' in self.config['Gimbal']:
            simulator = self.config['Gimbal']['simulator']
        
        if 'connection' in self.config['Gimbal']:
            connection_type = self.config['Gimbal']['connection'].get('type', 'unknown')
            connection_protocol = self.config['Gimbal']['connection'].get('protocol', 'unknown')
            connection_port = self.config['Gimbal']['connection'].get('port', 'unknown')
            connection_baudrate = self.config['Gimbal']['connection'].get('baudrate', 'unknown')
            connection_timeout = self.config['Gimbal']['connection'].get('timeout', 'unknown')

            logging.info(f"Gimbal connection type: {connection_type}")
            logging.info(f"Gimbal connection protocol: {connection_protocol}")
            logging.info(f"Gimbal connection port: {connection_port}")
            logging.info(f"Gimbal connection baudrate: {connection_baudrate}")
            logging.info(f"Gimbal connection timeout: {connection_timeout}")
        else:
            logging.warning("Gimbal configuration is missing 'connection' section, which is required for gimbal connection.")
            return None
        
        if 'telemetry' in self.config['Gimbal']:
            telemetry_frequency = self.config['Gimbal']['telemetry'].get('frequency', None)
            heartbeat_frequency = self.config['Gimbal']['telemetry'].get('heartbeat_frequency', None)
            
            logging.info(f"Gimbal telemetry frequency: {telemetry_frequency} Hz")
            logging.info(f"Gimbal heartbeat frequency: {heartbeat_frequency} Hz")

        
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
            logging.warning("Unknown gimbal type found in configuration.")
            return None

    def get_config(self):
        return self.config
    

    def connect(self):
        if self.drone is not None:
            logging.info("Connecting to drone...")
            self.drone.connect()
        else:
            logging.warning("No drone configured, cannot connect.")
        
        if self.gimbal is not None:
            logging.info("Connecting to gimbal...")
            self.gimbal.connect()
        else:
            logging.warning("No gimbal configured, cannot connect.")

    
    def disconnect(self):
        if self.drone is not None:
            logging.info("Disconnecting from drone...")
            self.drone.disconnect()
        else:
            logging.warning("No drone configured, cannot disconnect.")
        
        if self.gimbal is not None:
            logging.info("Disconnecting from gimbal...")
            self.gimbal.disconnect()
        else:
            logging.warning("No gimbal configured, cannot disconnect.")

        logging.info("Disconnected.")


    def start_telemetry(self):
        if self.drone is not None:
            logging.info("Starting drone telemetry logging...")
            self.drone.start_telemetry()
        else:
            logging.warning("No drone configured, cannot start telemetry.")

        if self.gimbal is not None:
            logging.info("Starting gimbal telemetry logging...")
            self.gimbal.start_telemetry()
        else:
            logging.warning("No gimbal configured, cannot start telemetry.")
        
    
    def stop_telemetry(self):
        if self.drone is not None:
            logging.info("Stopping drone telemetry logging...")
            self.drone.stop_telemetry()
        else:
            logging.warning("No drone configured, cannot stop telemetry.")

        if self.gimbal is not None:
            logging.info("Stopping gimbal telemetry logging...")
            self.gimbal.stop_telemetry()
        else:
            logging.warning("No gimbal configured, cannot stop telemetry.")

        logging.info("Telemetry logging stopped. Data saved to: " + self.data_folder)


    def set_POI(self):
        # check if POI is configured
        if 'POI' not in self.config:
            logging.warning("Configuration file is missing 'POI' section.")
            self.poi = None
            return None
        
        if 'name' in self.config['POI']:
            name = self.config['POI']['name']
            logging.info(f"POI type specified in configuration: {name}")
        else:
            name = "unknown"

        if 'latitude' in self.config['POI'] and 'longitude' in self.config['POI'] and 'altitude' in self.config['POI']:
            latitude = self.config['POI']['latitude']
            longitude = self.config['POI']['longitude']
            altitude = self.config['POI']['altitude']
            logging.info(f"POI coordinates: lat={latitude}, lon={longitude}, alt={altitude}")
        else:
            logging.warning("POI configuration is missing 'latitude', 'longitude', or 'altitude' fields.")
            self.poi = None
            return None
        
        self.poi = POI.POI(name, latitude, longitude, altitude)
    
    def get_drone_data(self):
        return None
    
    def get_gimbal_data(self):
        return None
    
    def get_logs(self):
        return None