import sys
from matplotlib import pyplot as plt

sys.path.append('..')
from DataLoader import DataLoader

def main():
    data_folder = 'data/gimbal_motion/'
    loader = DataLoader(data_folder)
    data = loader.get_data()
    

    # get the drone and gimbal data from the loaded data as pandas dataframes
    drone_data = data['drone']
    gimbal_data = data['gimbal']

    fig, axs = plt.subplots(3, 1, figsize=(10, 10), sharex=True)

    # gimbal data
    clean_data = gimbal_data.dropna(subset=['yaw', 'pitch', 'roll'])
    axs[0].plot(clean_data['timestamp'], clean_data['roll'], label='Roll', color='blue')
    axs[0].plot(clean_data['timestamp'], clean_data['pitch'], label='Pitch', color='green')
    axs[0].plot(clean_data['timestamp'], clean_data['yaw'], label='Yaw', color='red')

    axs[0].set_title('Gimbal Yaw, Pitch and Roll')
    axs[0].set_xlabel('Timestamp [s]')
    axs[0].set_ylabel('Angle [°]')
    axs[0].legend()

    # rc data
    clean_data = drone_data.dropna(subset=['rc_yaw', 'rc_pitch', 'rc_roll'])
    axs[1].plot(clean_data['timestamp'], clean_data['rc_roll'], label='Roll', color='blue')
    axs[1].plot(clean_data['timestamp'], clean_data['rc_pitch'], label='Pitch', color='green')
    axs[1].plot(clean_data['timestamp'], clean_data['rc_yaw'], label='Yaw', color='red')

    axs[1].set_title('RC Yaw, Pitch and Roll')
    axs[1].set_xlabel('Timestamp [s]')
    axs[1].set_ylabel('Value [a.u.]')
    axs[1].legend()

    # drone data
    clean_data = drone_data.dropna(subset=['yaw', 'pitch', 'roll'])
    axs[2].plot(clean_data['timestamp'], clean_data['roll'], label='Roll', color='blue')
    axs[2].plot(clean_data['timestamp'], clean_data['pitch'], label='Pitch', color='green')
    axs[2].plot(clean_data['timestamp'], clean_data['yaw'], label='Yaw', color='red')

    axs[2].set_title('Drone Yaw, Pitch and Roll')
    axs[2].set_xlabel('Timestamp [s]')
    axs[2].set_ylabel('Angle [°]')
    axs[2].legend()

    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    main()