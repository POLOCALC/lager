from DataLoader import DataLoader
import argparse
from matplotlib import pyplot as plt
import matplotlib as mpl
import parameters as params
import os

parser = argparse.ArgumentParser(description='Plot telemetry data')
parser.add_argument('--data-folder', 
                    default=os.path.join(params.DATA_PATH, params.CURRENT_DATA_FOLDER), 
                    help='Folder containing the decoded data')
parser.add_argument('--ascii', default=False, action='store_true', help='Display plots in ASCII format')


def main():
    args = parser.parse_args()
    data_folder = args.data_folder

    loader = DataLoader(data_folder)
    data = loader.get_data()

    if args.ascii:
        print("Displaying plots in ASCII format...")
        mpl.use("module://mpl_ascii")     

    # get the drone and gimbal data from the loaded data as pandas dataframes
    drone_data = data['drone']
    gimbal_data = data['gimbal']

    fig, axs = plt.subplots(3, 3, figsize=(20, 10), sharex=True)

    #### DRONE DATA ####
    # accelerometer
    clean_data = drone_data.dropna(subset=['ax', 'ay', 'az'])
    time = clean_data['timestamp']-clean_data['timestamp'].iloc[0]  # convert to relative time
    axs[0][0].plot(time, clean_data['ax'], label='Acc X', color='blue')
    axs[0][0].plot(time, clean_data['ay'], label='Acc Y', color='green')
    axs[0][0].plot(time, clean_data['az'], label='Acc Z', color='red')
    axs[0][0].set_title('Drone Accelerometer Data')
    axs[0][0].set_ylabel('Acc. [m/s2]')
    axs[0][0].legend()

    # gyro data
    clean_data = drone_data.dropna(subset=['gx', 'gy', 'gz'])
    time = clean_data['timestamp']-clean_data['timestamp'].iloc[0]
    axs[0][1].plot(time, clean_data['gx'], label='Gyro X', color='blue')
    axs[0][1].plot(time, clean_data['gy'], label='Gyro Y', color='green')
    axs[0][1].plot(time, clean_data['gz'], label='Gyro Z', color='red')
    axs[0][1].set_title('Drone Gyroscope Data')
    axs[0][1].set_ylabel('Ang. vel. [rad/s]')
    axs[0][1].legend()

    # magnetometer data
    clean_data = drone_data.dropna(subset=['mx', 'my', 'mz'])
    time = clean_data['timestamp']-clean_data['timestamp'].iloc[0]
    axs[0][2].plot(time, clean_data['mx'], label='Mag X', color='blue')
    axs[0][2].plot(time, clean_data['my'], label='Mag Y', color='green')
    axs[0][2].plot(time, clean_data['mz'], label='Mag Z', color='red')
    axs[0][2].set_title('Drone Magnetometer Data')
    axs[0][2].set_ylabel('Counts')
    axs[0][2].legend()

    # attitude data
    clean_data = drone_data.dropna(subset=['yaw', 'pitch', 'roll'])
    time = clean_data['timestamp']-clean_data['timestamp'].iloc[0]
    axs[1][0].plot(time, clean_data['roll'], label='Roll', color='blue')
    axs[1][0].plot(time, clean_data['pitch'], label='Pitch', color='green')
    axs[1][0].plot(time, clean_data['yaw'], label='Yaw', color='red')
    axs[1][0].set_title('Drone Yaw, Pitch and Roll')
    axs[1][0].set_ylabel('Angle [°]')
    axs[1][0].legend()

    # velocity data
    clean_data = drone_data.dropna(subset=['vx', 'vy', 'vz'])
    time = clean_data['timestamp']-clean_data['timestamp'].iloc[0]
    axs[1][1].plot(time, clean_data['vx'], label='Vel X', color='blue')
    axs[1][1].plot(time, clean_data['vy'], label='Vel Y', color='green')
    axs[1][1].plot(time, clean_data['vz'], label='Vel Z', color='red')
    axs[1][1].set_title('Drone Velocity Data')
    axs[1][1].set_ylabel('Vel. [m/s]')
    axs[1][1].legend()

    # battery data
    clean_data = drone_data.dropna(subset=['bat_percentage'])
    time = clean_data['timestamp']-clean_data['timestamp'].iloc[0]
    axs[1][2].plot(time, clean_data['bat_percentage'], label='Battery %', color='blue')
    axs[1][2].set_title('Drone Battery Percentage')
    axs[1][2].set_ylabel('Battery [%]')
    axs[1][2].legend()

    #### GIMBAL DATA ####
    # attitude
    clean_data = gimbal_data.dropna(subset=['yaw', 'pitch', 'roll'])
    time = clean_data['timestamp']-clean_data['timestamp'].iloc[0]
    axs[2][0].plot(time, clean_data['roll'], label='Roll', color='blue')
    axs[2][0].plot(time, clean_data['pitch'], label='Pitch', color='green')
    axs[2][0].plot(time, clean_data['yaw'], label='Yaw', color='red')
    axs[2][0].set_title('Gimbal Yaw, Pitch and Roll')
    axs[2][0].set_xlabel('Time [s]')
    axs[2][0].set_ylabel('Angle [°]')
    axs[2][0].legend()

    # battery
    clean_data = gimbal_data.dropna(subset=['voltage_battery'])
    time = clean_data['timestamp']-clean_data['timestamp'].iloc[0]
    axs[2][1].plot(time, clean_data['voltage_battery'], label='Supply Voltage', color='blue')
    axs[2][1].set_title('Gimbal Supply Voltage')
    axs[2][1].set_xlabel('Time [s]')
    axs[2][1].set_ylabel('Voltage [mV]')
    axs[2][1].legend()

    #### RC DATA ####
    clean_data = drone_data.dropna(subset=['rc_yaw', 'rc_pitch', 'rc_roll', 'rc_thr'])
    time = clean_data['timestamp']-clean_data['timestamp'].iloc[0]
    axs[2][2].plot(time, clean_data['rc_roll'], label='Roll', color='blue')
    axs[2][2].plot(time, clean_data['rc_pitch'], label='Pitch', color='green')
    axs[2][2].plot(time, clean_data['rc_yaw'], label='Yaw', color='red')
    axs[2][2].plot(time, clean_data['rc_thr'], label='Throttle', color='orange')
    axs[2][2].set_title('RC Yaw, Pitch, Roll and Throttle')
    axs[2][2].set_xlabel('Time [s]')
    axs[2][2].set_ylabel('Value [a.u.]')
    axs[2][2].legend()

    fig.savefig(os.path.join(data_folder, 'telemetry_plots.png'))
    print(f"Telemetry plots saved to {os.path.join(data_folder, 'telemetry_plots.png')}")

    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    main()