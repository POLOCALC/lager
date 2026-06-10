from yaml import loader

from DataLoader import DataLoader
import argparse
import parameters as params
import os

parser = argparse.ArgumentParser(description='Decode telemetry data')
parser.add_argument('--data-folder', 
                    default=os.path.join(params.DATA_PATH, params.CURRENT_DATA_FOLDER), 
                    help='Folder containing the data')
parser.add_argument('--force-decode', default=False, action='store_true', help='Force decoding of data files')
parser.add_argument('--overwrite', default=False, action='store_true', help='Overwrite decoded data file if it already exists')

def main():
    args = parser.parse_args()
    data_folder = args.data_folder
    loader = DataLoader(data_folder)
    data = loader.get_data(force_decode=args.force_decode)

    # get the drone and gimbal data from the loaded data as pandas dataframes
    drone_data = data['drone']
    gimbal_data = data['gimbal']
    
    print("Drone data:")
    print(drone_data)
    print("Gimbal data:")
    print(gimbal_data)

    loader.save_data(data, force_overwrite=args.overwrite)

if __name__ == "__main__":
    main()