import sys
import time
sys.path.append('..')

from Controller import Controller

gimbal_roll_limits  = (-30, 30) # degrees
gimbal_pitch_limits = (-30, 30) # degrees
gimbal_yaw_limits   = (-50, 50) # degrees

goto_command_rate = 10 # Hz
RC_limits = (-10000, 10000) # -10000 to 10000

def map_RC_to_angle(x, x_min, x_max, y_min, y_max):
    """
    Map x from range [x_min, x_max] to range [y_min, y_max].
    """
    # Clamp x to the input range
    x = max(min(x, x_max), x_min)
    # Linear mapping
    y = y_min + (x - x_min) * (y_max - y_min) / (x_max - x_min)
    return y
    

def main():
    config_file = '../config/move_gimbal_config.yaml'
    controller = Controller(config_file)
    
    # connect to drone and gimbal
    controller.connect()


    # start telemetry logging
    controller.start_telemetry()

    # stop telemetry logging if ctrl+c is pressed
    try:
        while True:
            drone_telemetry = controller.drone.telemetry_state.get()
            rc_roll = drone_telemetry.get("rc_roll", 0)
            rc_pitch = drone_telemetry.get("rc_pitch", 0)
            rc_yaw = drone_telemetry.get("rc_yaw", 0)

            # map RC inputs to gimbal angles
            roll = map_RC_to_angle(rc_roll, RC_limits[0], RC_limits[1], gimbal_roll_limits[0], gimbal_roll_limits[1])
            pitch = map_RC_to_angle(rc_pitch, RC_limits[0], RC_limits[1], gimbal_pitch_limits[0], gimbal_pitch_limits[1])
            yaw = map_RC_to_angle(rc_yaw, RC_limits[0], RC_limits[1], gimbal_yaw_limits[0], gimbal_yaw_limits[1])

            # send gimbal control command
            controller.gimbal.goto(roll=roll, pitch=pitch, yaw=yaw, wait=False)

            time.sleep(1/goto_command_rate)

    except KeyboardInterrupt:
        controller.stop_telemetry()
        
    # disconnect from the drone and gimbal
    controller.disconnect()


if __name__ == "__main__":
    main()