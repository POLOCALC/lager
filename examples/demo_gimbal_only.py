import time
from lager import PointingController

gimbal_roll_limits  = (-30, 30) # degrees
gimbal_pitch_limits = (-30, 30) # degrees
gimbal_yaw_limits   = (-50, 50) # degrees

wait_time = 1 # seconds

def main():
    config_file = '../config/demo_gimbal_only.yaml'
    controller = PointingController(config_file)
    
    # connect to the gimbal
    controller.connect()


    # start telemetry logging
    controller.start_telemetry()

    # stop telemetry logging if ctrl+c is pressed
    try:
        while True:
            # set speed for gimbal movement
            controller.gimbal.set_speed(roll_speed=5, pitch_speed=5, yaw_speed=5)
            
            roll, pitch, yaw = 0, 0, 0
            controller.gimbal.goto(roll=roll, pitch=pitch, yaw=yaw, wait=True)
            time.sleep(wait_time)

            roll, pitch, yaw = gimbal_roll_limits[0], 0, 0
            controller.gimbal.goto(roll=roll, pitch=pitch, yaw=yaw, wait=True)
            time.sleep(wait_time)

            roll, pitch, yaw = gimbal_roll_limits[1], 0, 0
            controller.gimbal.goto(roll=roll, pitch=pitch, yaw=yaw, wait=True)
            time.sleep(wait_time)

            # home gimbal
            roll, pitch, yaw = 0, 0, 0
            controller.gimbal.goto(roll=roll, pitch=pitch, yaw=yaw, wait=True)
            time.sleep(wait_time)
        
    except KeyboardInterrupt:
        controller.stop_telemetry()
        
    # disconnect from the drone and gimbal
    controller.disconnect()


if __name__ == "__main__":
    main()