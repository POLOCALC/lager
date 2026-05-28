from pymavlink import mavutil
import time
import logging
import numpy as np

logger = logging.getLogger(__name__)

# acknowledgment results
ack_results = {
            mavutil.mavlink.MAV_RESULT_ACCEPTED:             "Accepted",
            mavutil.mavlink.MAV_RESULT_TEMPORARILY_REJECTED: "Temporarily Rejected",
            mavutil.mavlink.MAV_RESULT_DENIED:               "Denied",
            mavutil.mavlink.MAV_RESULT_UNSUPPORTED:          "Unsupported",
            mavutil.mavlink.MAV_RESULT_FAILED:               "Failed",
            mavutil.mavlink.MAV_RESULT_IN_PROGRESS:          "In Progress"
            }


def parse_msg(msg):
    """
    Convert a MAVLink message object into a dictionary where the keys are the field names of the message
    and the values are the corresponding field values. If the input message is None, return None.
    
    Parameters:
    -----------
        msg : MAVLink message object
    
    Returns:
    --------
        dict or None: A dictionary containing the field names and values of the message, or None if the 
        input message is None.
    """
    if msg is None:
        return None
    return {key:getattr(msg, key) for key in msg.fieldnames}

def message_dispatcher(master, handlers, stop_event, poll_frequency=20):
    """
    Continuously drain the MAVLink buffer and dispatch incoming messages to registered
    handler callbacks. Designed to run in a dedicated thread.

    For each received message whose type has a registered handler, the handler is called
    with the parsed message dictionary as argument. Unknown message types are silently
    ignored. If no message is available, the thread sleeps for one poll interval to avoid
    busy-waiting.

    Parameters:
    -----------
        master : MAVLink connection object
            The active MAVLink connection to read messages from.
        handlers : dict[str, callable]
            A mapping from MAVLink message type strings (e.g. "RAW_IMU") to callables.
            Each callable receives a single argument: the parsed message dict.
            Example:
                {
                    "RAW_IMU": lambda d: print(d["xgyro"]),
                    "GIMBAL_DEVICE_ATTITUDE_STATUS": my_callback,
                }
        stop_event : threading.Event
            When set, the loop exits cleanly.
        poll_frequency : int, optional
            How many times per second to poll for new messages when the buffer is empty.
            Default is 20 Hz (50 ms sleep between empty polls). Set higher for lower
            latency, lower to reduce CPU usage. Has no effect when messages arrive back-
            to-back, since the loop re-reads immediately while data is available.

    Returns:
    --------
        str: The type of the last message processed before stop_event was set, or None if no messages were processed.
    """
    sleep_interval = 1.0 / poll_frequency
    wildcard = handlers.get("*")
    specific_types = [k for k in handlers if k != "*"]
    # if a wildcard handler is registered, disable the type filter so every
    # message reaches us; otherwise restrict to only the registered types
    msg_types = None if wildcard is not None else (specific_types or None)

    while not stop_event.is_set():
        msg = master.recv_match(type=msg_types, blocking=False)
        if msg is None:
            time.sleep(sleep_interval)
            continue
        else:
            msg_type = msg.get_type()
            handler = handlers.get(msg_type, wildcard)
            if handler is not None:
                try:
                    logging.debug(f"Msg received: '{msg_type}'")
                    handler(parse_msg(msg))
                except Exception:
                    logger.error(f"Error occurred while handling message of type {msg_type}")
                    continue
            else:
                logger.debug(f"No handler registered for message type '{msg_type}', ignoring.")


def interp(start, target, axis_time, elapsed):
    """
    Linearly interpolate between a starting value and a target value based on the elapsed time and the total
    time of the axis movement. If the elapsed time is equal to or greater than the total time of the axis
    movement, return the target value. If the total time of the axis movement is zero, return the target value to avoid division by zero.

    Parameters:
    -----------
        start : float
            The starting value of the axis.
        target : float
            The target value of the axis.
        axis_time : float
            The total time of the axis movement in seconds.
        elapsed : float
            The elapsed time since the start of the axis movement in seconds.

    Returns:
    --------
        float: The interpolated value of the axis.
    """
    if axis_time == 0.0:
        return target
    t = min(elapsed / axis_time, 1.0)
    return start + t * (target - start)

def euler_to_quaternion(yaw, pitch, roll):
    """
    Convert Euler angles (yaw, pitch, roll) in degrees to a quaternion (w, x, y, z).

    Parameters:
    -----------
    yaw : float
        The yaw angle in degrees.
    pitch : float
        The pitch angle in degrees.
    roll : float
        The roll angle in degrees.

    Returns:
    --------    
    list: A list containing the quaternion components [w, x, y, z].
    """

    # Convert degrees to radians
    yaw = np.radians(yaw)
    pitch = np.radians(pitch)
    roll = np.radians(roll)
    cy = np.cos(yaw * 0.5)
    sy = np.sin(yaw * 0.5)
    cp = np.cos(pitch * 0.5)
    sp = np.sin(pitch * 0.5)
    cr = np.cos(roll * 0.5)
    sr = np.sin(roll * 0.5)
    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    return [w, x, y, z]