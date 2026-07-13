from lager import PointingController as PC
import yaml
import time 

gimbal_roll_limits  = (-30, 30) # degrees
gimbal_pitch_limits = (-30, 30) # degrees
gimbal_yaw_limits   = (-50, 50) # degrees
wait_time = 1 # seconds

def main():
    # Load the configuration file
    with open('../config/ext_controller.yaml', 'r') as f:
        config = yaml.safe_load(f)
    config = config['pointing_controller']  # Extract the pointing_controller section

    pc = PC(config, "../data", verbose=True)

    # connect to the gimbal
    pc.connect()

    # start telemetry logging
    pc.start_telemetry()

    # stop telemetry logging if ctrl+c is pressed
    try:
        while True:
            # set speed for gimbal movement
            pc.gimbal.set_speed(roll_speed=5, pitch_speed=5, yaw_speed=5)
            
            roll, pitch, yaw = 0, 0, 0
            pc.gimbal.goto(roll=roll, pitch=pitch, yaw=yaw, wait=True)
            time.sleep(wait_time)

            roll, pitch, yaw = gimbal_roll_limits[0], 0, 0
            pc.gimbal.goto(roll=roll, pitch=pitch, yaw=yaw, wait=True)
            time.sleep(wait_time)

            roll, pitch, yaw = gimbal_roll_limits[1], 0, 0
            pc.gimbal.goto(roll=roll, pitch=pitch, yaw=yaw, wait=True)
            time.sleep(wait_time)

            # home gimbal
            roll, pitch, yaw = 0, 0, 0
            pc.gimbal.goto(roll=roll, pitch=pitch, yaw=yaw, wait=True)
            time.sleep(wait_time)
        
    except KeyboardInterrupt:
        pc.stop_telemetry()
        
    # disconnect from the drone and gimbal
    pc.disconnect()

if __name__ == "__main__":
    main()