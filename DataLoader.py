import os
import yaml
import pickle

from Drones import DJI_M600
from Gimbals import Gremsy_T7

decoded_data_filename = 'decoded_telemetry_data.pkl'

class DataLoader:

    def __init__(self, data_folder):
        self.data_folder = data_folder
        self.drone_data_loader = None
        self.gimbal_data_loader = None

        # look for a yaml configuration file in the data folder
        config_file = None
        for file in os.listdir(data_folder):
            if file.endswith('.yaml') or file.endswith('.yml'):
                config_file = os.path.join(data_folder, file)
                break

        if config_file is None:
            raise ValueError("No YAML configuration file found in the data folder")

        with open(config_file, 'r') as f:
            self.config = yaml.safe_load(f)
        print(f"Configuration loaded from {config_file}:")

        # check if the configuration contains drone key
        if 'Drone' in self.config:
            if 'name' in self.config['Drone']:
                print(f"  Drone: {self.config['Drone']['name']}")

                if self.config['Drone']['name'] == 'DJI M600':
                    self.drone_data_loader = DJI_M600.DataLoader(data_folder)
            else:
                print("  Warning: 'name' key not found in 'drone' configuration")
        else:
            print("  No 'drone' key not found in configuration")


        # check if the configuration contains gimbal key
        if 'Gimbal' in self.config:
            if 'name' in self.config['Gimbal']:
                print(f"  Gimbal: {self.config['Gimbal']['name']}")

                if self.config['Gimbal']['name'] == 'Gremsy T7':
                    self.gimbal_data_loader = Gremsy_T7.DataLoader(data_folder)
            else:
                print("  Warning: 'name' key not found in 'gimbal' configuration")
        else:
            print("  No 'gimbal' key not found in configuration")


    def get_data(self, force_decode=False):
        # check if the decoded data file already exists, if so load it and return it
        path_to_decoded_file = os.path.join(self.data_folder, decoded_data_filename)
        if os.path.exists(path_to_decoded_file) and not force_decode:
            try:
                with open(path_to_decoded_file, 'rb') as f:
                    data = pickle.load(f)
                print(f"Decoded data already exists and is loaded from {path_to_decoded_file}")
                print(f"To force re-decoding the raw data, set force_decode=True when calling get_data()")
                return data
            except Exception as e:
                print(f"Error loading decoded data from {path_to_decoded_file}: {e}")
                # if there is an error loading the decoded data, we will try to load the raw data and decode it again
                pass

        data = {}

        if self.drone_data_loader is not None:
            data['drone'] = self.drone_data_loader.get_data()

        if self.gimbal_data_loader is not None:
            data['gimbal'] = self.gimbal_data_loader.get_data()

        return data
    
    def save_data(self, data, force_overwrite=False):
        """
        Save telemetry data to a binary file in the data folder.
        """
        # check if the decoded data file already exists, if so print a warning that it will be overwritten
        path_to_decoded_file = os.path.join(self.data_folder, decoded_data_filename)
        if os.path.exists(path_to_decoded_file):
            print(f"Warning: Decoded data file already exists at {path_to_decoded_file}.")
            print(f"To overwrite it, set force_overwrite=True when calling save_data()")
            if not force_overwrite:
                return

        path_to_file = os.path.join(self.data_folder, decoded_data_filename)

        try:
            with open(path_to_file, 'wb') as f:
                pickle.dump(data, f)
            print(f"Parsed data saved to {path_to_file}")
        except Exception as e:
            print(f"Error saving parsed data to {path_to_file}: {e}")