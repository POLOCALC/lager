import sys

sys.path.append('..')
from DataLoader import DataLoader

def main():
    data_folder = 'data/gimbal_motion/'
    loader = DataLoader(data_folder)
    data = loader.get_data()

    # get the drone and gimbal data from the loaded data as pandas dataframes
    drone_data = data['drone']
    gimbal_data = data['gimbal']
    
    print("Drone data:")
    print(drone_data)
    print("Gimbal data:")
    print(gimbal_data)

    loader.save_data(data)

if __name__ == "__main__":
    main()