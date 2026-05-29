import sys
import numpy as np

sys.path.append('..')
from DataLoader import DataLoader

def main():
    data_folder = 'data/gimbal_motion/'
    loader = DataLoader(data_folder)
    data = loader.get_data()

    drone_data = data['drone']
    gimbal_data = data['gimbal']

    print("Drone telemetry frequency:")
    if drone_data is not None and 'timestamp' in drone_data:
        timestamps = drone_data['timestamp']
        if len(timestamps) > 1:
            frequency = 1.0 / np.diff(timestamps).mean()
            print(f"  {frequency:.2f} Hz")
        else:
            print("  Not enough data to calculate frequency")
    else:
        print("  No drone telemetry data found")

    print("Gimbal telemetry frequency for different commands:")
    for keyword in gimbal_data.keys():
        timestamps = gimbal_data[keyword]['timestamp']
        if len(timestamps) > 1:
            frequency = 1.0 / np.diff(timestamps).mean()
            print(f"  {keyword}: {frequency:.2f} Hz")
        else:
            print(f"  {keyword}: Not enough data to calculate frequency")


if __name__ == "__main__":
    main()