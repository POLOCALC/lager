import sys
import time
sys.path.append('..')

from Controller import Controller

def main():
    config_file = '../config/follow_POI_config.yaml'
    controller = Controller(config_file)
    
    # connect to the drone and gimbal (if configured)
    controller.connect()

    # start telemetry logging (if configured)
    controller.start_telemetry()

    # wait a bit to ensure telemetry is being received
    time.sleep(5)

    # start following the POI
    controller.poi.start_tracking(controller.drone, controller.gimbal, forward_heading=True)

    # stop telemetry logging if ctrl+c is pressed
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        controller.poi.stop_tracking()
        controller.stop_telemetry()
        
    # disconnect from the drone and gimbal
    controller.disconnect()

if __name__ == "__main__":
    main()