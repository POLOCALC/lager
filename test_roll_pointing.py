"""
Quick test: point the gimbal at a fixed yaw/pitch, then sweep the image roll
0 -> +angle -> -angle -> 0 while keeping the same line of sight.

Usage:
    python3 test_roll_pointing.py --yaw 0 --pitch 30 --angle 20
"""
import argparse
import logging
import time

import numpy as np

from lager.Gimbals.Gremsy_T7.Gimbal import Gimbal
from lager.POI import POI

logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(name)s: %(message)s")
logger = logging.getLogger(__name__)


def main():
    parser = argparse.ArgumentParser(description="Roll sweep while keeping a fixed pointing")
    parser.add_argument("--port", default="/dev/serial1", help="gimbal serial port")
    parser.add_argument("--yaw", type=float, default=0.0, help="pointing azimuth [deg]")
    parser.add_argument("--pitch", type=float, default=30.0, help="pointing elevation [deg]")
    parser.add_argument("--angle", type=float, default=20.0, help="max image roll [deg]")
    parser.add_argument("--step", type=float, default=.2, help="roll increment per update [deg]")
    parser.add_argument("--dt", type=float, default=0.1, help="update period [s]")
    args = parser.parse_args()

    # roll profile: 0 -> +angle -> -angle -> 0
    up = np.arange(0.0, args.angle, args.step)
    down = np.arange(args.angle, -args.angle, -args.step)
    back = np.arange(-args.angle, 0.0 + args.step, args.step)
    roll_profile = np.concatenate([up, down, back])

    gimbal = Gimbal(port=args.port)
    gimbal.connect()
    try:
        # move to the initial pointing with no roll and wait for it to settle
        yaw, pitch, roll = POI.los_to_gimbal_angles(args.yaw, args.pitch, 0.0)
        logger.info(f"Initial pointing: yaw={yaw:.2f} pitch={pitch:.2f} roll={roll:.2f} deg")
        gimbal.goto(yaw=yaw, pitch=pitch, roll=roll, wait=True)

        logger.info(f"Starting roll sweep: +-{args.angle} deg, step {args.step} deg, every {args.dt} s")
        for image_roll in roll_profile:
            t0 = time.monotonic()
            yaw, pitch, roll = POI.los_to_gimbal_angles(args.yaw, args.pitch, image_roll)
            gimbal.goto(yaw=yaw, pitch=pitch, roll=roll, wait=False)
            logger.info(f"image_roll={image_roll:6.2f} -> yaw={yaw:7.2f} pitch={pitch:7.2f} roll={roll:7.2f} deg")
            time.sleep(max(0.0, args.dt - (time.monotonic() - t0)))

        # settle back on the initial pointing
        yaw, pitch, roll = POI.los_to_gimbal_angles(args.yaw, args.pitch, 0.0)
        gimbal.goto(yaw=yaw, pitch=pitch, roll=roll, wait=True)
        logger.info("Roll sweep completed")
    except KeyboardInterrupt:
        logger.info("Interrupted by user")
    finally:
        gimbal.disconnect()


if __name__ == "__main__":
    main()
