from Controller import Controller
import logging
import time
import argparse

parser = argparse.ArgumentParser(description='Follow a point of interest (POI) with the drone and gimbal')
parser.add_argument('--config', default='config/follow_POI_config.yaml', help='Path to the configuration file')
config_file = parser.parse_args().config

def main():
    controller = None
    try:
        controller = Controller(config_file)

        # connect to the drone and gimbal (if configured)
        controller.connect()

        # start telemetry logging (if configured)
        controller.start_telemetry()

        # wait a bit to ensure telemetry is being received
        time.sleep(5)

        # start following the POI
        if controller.poi is not None:
            controller.poi.start_tracking(controller.drone, controller.gimbal, forward_heading=True)

        # stop telemetry logging if ctrl+c is pressed
        try:
            while True:
                time.sleep(1)
        except KeyboardInterrupt:
            if controller.poi is not None:
                controller.poi.stop_tracking()
            controller.stop_telemetry()

    except Exception:
        logging.exception("Unhandled exception, shutting down")

    finally:
        # always disconnect to release serial ports and flush telemetry buffers
        if controller is not None:
            controller.disconnect()

if __name__ == "__main__":
    main()