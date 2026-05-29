import json
import struct
import os

import Drones.DJI_M600.parameters as params
import Drones.DJI_M600.utils as utils

class DataLoader:
    def __init__(self, data_folder):
        self.data_folder = data_folder


    def load_raw_data(self, filename=params.TELEMETRY_FILENAME):
        path_to_file = os.path.join(self.data_folder, filename)

        try:
            # read until the end of the file to load all telemetry data
            with open(path_to_file, 'rb') as f:
                frames = []
                while True:
                    try:
                        # read 8 bytes to get the timestamp (float64)
                        timestamp_bytes = f.read(8)
                        if len(timestamp_bytes) < 8:
                            print("Reached end of file while reading timestamp.")
                            break
                        timestamp = struct.unpack('d', timestamp_bytes)[0]

                        # read 4 bytes to get the frame length (uint32)
                        frame_length_bytes = f.read(4)
                        if len(frame_length_bytes) < 4:
                            print("Reached end of file while reading frame length.")
                            break
                        frame_length = struct.unpack('I', frame_length_bytes)[0]

                        # read the frame data based on the frame length
                        frame_data = f.read(frame_length)
                        if len(frame_data) < frame_length:
                            print("Reached end of file while reading frame data.")
                            break

                        frames.append([timestamp, utils.parse_frame(frame_data)])
                    except EOFError:
                        break
                print(f"Telemetry data loaded from {path_to_file}. Found {len(frames)} frames.")

                
        except FileNotFoundError:
            print(f"File not found: {path_to_file}")
            return None
        except Exception as e:
            print(f"Error loading telemetry data from {path_to_file}: {e}")
            return None
        
        return frames
    

    def parse_raw_data(self, raw_data):
        # raw data is a list of dictionaries with different commands keywords and telemetry data
        parsed_data = {"timestamp": [],
                       "data": []}

        for item in raw_data:
            parsed_data["timestamp"].append(item[0])
            parsed_data["data"].append(item[1])

        return parsed_data

    def get_data(self):
        raw_data = self.load_raw_data()
        if raw_data is None:
            return None
        
        parsed_data = self.parse_raw_data(raw_data)

        return parsed_data