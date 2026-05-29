import pickle
import json
import os

import Gimbals.Gremsy_T7.parameters as params

class DataLoader:
    def __init__(self, data_folder):
        self.data_folder = data_folder


    def load_raw_data(self, filename=params.TELEMETRY_FILENAME):
        path_to_file = os.path.join(self.data_folder, filename)

        try:
            # read until the end of the file to load all telemetry data
            with open(path_to_file, 'rb') as f:
                data = []
                while True:
                    try:
                        data.append(pickle.load(f))
                    except EOFError:
                        break
        except FileNotFoundError:
            print(f"File not found: {path_to_file}")
            return None
        except Exception as e:
            print(f"Error loading telemetry data from {path_to_file}: {e}")
            return None
        
        # flatten the list of lists into a single list of dictionaries
        if isinstance(data, list) and all(isinstance(item, list) for item in data):
            data = [item for sublist in data for item in sublist]
        
        print(f"Telemetry data loaded from {path_to_file}. Found {len(data)} entries.")
        return data
    

    def parse_raw_data(self, raw_data):
        # raw data is a list of dictionaries with different commands keywords and telemetry data
        parsed_data = {}

        for item in raw_data:
            keyword = item.get("keyword")
            if keyword not in parsed_data:
                parsed_data[keyword] = {}
                parsed_data[keyword]["timestamp"] = []
                for key in item["data"].keys():
                    parsed_data[keyword][key] = []

            parsed_data[keyword]["timestamp"].append(item["timestamp"])
            for key in item["data"].keys():
                try:
                    parsed_data[keyword][key].append(item["data"][key])
                except KeyError:
                    print(f"Key {key} not found in item: {keyword} at timestamp {item['timestamp']}")

        return parsed_data

    def get_data(self):
        raw_data = self.load_raw_data()
        if raw_data is None:
            return None
        
        parsed_data = self.parse_raw_data(raw_data)
        return parsed_data