import os
import yaml

from Drones import DJI_M600
from Gimbals import Gremsy_T7

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


    def get_data(self):
        data = {}

        if self.drone_data_loader is not None:
            data['drone'] = self.drone_data_loader.get_data()

        if self.gimbal_data_loader is not None:
            data['gimbal'] = self.gimbal_data_loader.get_data()

        return data