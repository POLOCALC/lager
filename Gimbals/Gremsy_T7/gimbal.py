from pymavlink import mavutil, quaternion
import threading
import time
import numpy as np
import struct
import logging
import os
import pickle
import sys
from rich.table import Table
from rich.panel import Panel

import Gimbals.Gremsy_T7.parameters as params
import Gimbals.Gremsy_T7.keywords as kw
import Gimbals.Gremsy_T7.utils as utils
import Gimbals.Gremsy_T7.telemetry_state as telemetry_state


# use the Controller.py logging configuration
logger = logging.getLogger(__name__)

class Gimbal:
    def __init__(self, port: str = params.DEFAULT_SERIAL_PORT,
                 baudrate: int = params.DEFAULT_BAUDRATE,
                 timeout: float = params.TIMEOUT,
                 heartbeat_frequency: float = params.HEARTBEAT_FREQUENCY,
                 simulator: bool = False,
                 output_dir: str = ""):
        """
        Initialise a Gimbal instance and prepare all internal state.

        The serial connection is **not** opened here; call connect() to
        establish the link.

        Parameters:
        -----------
            port : str
                Serial port to use (e.g. '/dev/ttyUSB0').
            baudrate : int
                Baud rate for the serial connection.
            timeout : float
                Seconds to wait when polling for MAVLink messages.
            heartbeat_frequency : float
                Rate [Hz] at which heartbeat messages are sent to the gimbal.
        """
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.master = None
        self.system_id = None
        self.component_id = None
        self.mode = 'follow' # default mode
        self.heartbeat_frequency = heartbeat_frequency
        self.simulator = simulator

        self.output_dir = output_dir

        self.heartbeat_thread = None
        self.update_thread = None
        self.stop_event = threading.Event()

        self.reference_frame_flags = (mavutil.mavlink.GIMBAL_DEVICE_FLAGS_ROLL_LOCK |
                                      mavutil.mavlink.GIMBAL_DEVICE_FLAGS_PITCH_LOCK |
                                      mavutil.mavlink.GIMBAL_DEVICE_FLAGS_YAW_IN_EARTH_FRAME)

        # dictionaries to store data
        self.raw_imu           = {}
        self.attitude          = {}
        self.orientation       = {}
        self.mount_status      = {}
        self.sys_status        = {}
        self.acknowledgment    = {}
        self.heartbeat_dict    = {}
        self.param_values      = {}
        self.message_intervals = {}

        # buffer for recording data to file
        self._telemetry_log_buffer = []

        # filename for telemetry log
        if self.simulator:
            logger.info("Lab test simulator mode enabled - no serial connection will be made.")
            self.telemetry_log_filename = "gimbal_telemetry_simulator.bin"
        else:
            self.telemetry_log_filename = "gimbal_telemetry.bin"
        self.telemetry_log_filename = os.path.join(self.output_dir, self.telemetry_log_filename)
        logger.info(f"Telemetry log will be saved to: {self.telemetry_log_filename}")

        # desired movement speed (deg/s); None means use the gimbal's default speed
        self.yaw_speed   = None
        self.pitch_speed = None
        self.roll_speed  = None

        self.fw_version = None
        self.serial_number = None

        # telemetry state
        self.telemetry_state = telemetry_state.TelemetryState()
        self._log_telemetry_flag = False

    def start_telemetry(self):
        """
        Start logging telemetry data by setting the flag to True. Telemetry data will be logged
        in the update thread as it is received.
        """
        logger.info("Starting gimbal telemetry logging...")
        self._log_telemetry_flag = True

    def stop_telemetry(self):
        """
        Stop logging telemetry data by setting the flag to False. Any remaining data in the log
        buffer will be written to file.
        """
        if self._log_telemetry_flag:
            logger.info("Stopping gimbal telemetry logging...")
            self._log_telemetry_flag = False
            if self._telemetry_log_buffer:
                logger.info(f"Writing remaining {len(self._telemetry_log_buffer)} telemetry messages to file...")
                self._write_telemetry_log()

    def send_heartbeat(self):
        """
        Send heartbeat messages to the gimbal at regular intervals to maintain the connection.
        This is necessary because the gimbal may disconnect if it does not receive a heartbeat
        message within a certain time frame.
        """
        try:
            while not self.stop_event.is_set():
                self.master.mav.heartbeat_send(
                    mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
                    mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                    0, 0, 0
                )
                time.sleep(1.0/self.heartbeat_frequency)
        except Exception as e:
            logger.error(f"Error in heartbeat thread: {e}")
            self.stop_event.set()

    def connect(self):
        """
        Connect to the gimbal by opening the serial port and starting the heartbeat and status
        update threads. Also retrieves the system and component IDs from the gimbal to allow
        targeted communication.
        """
        if self.simulator:
            logger.info("Lab test simulator mode enabled - skipping serial connection.")
            logger.debug("Starting update thread...")
            self.update_thread = threading.Thread(target=self.update_status_simulator, daemon=True)
            self.update_thread.start()
            return
        
        self.stop_event.clear()
        try:
            self.master = mavutil.mavlink_connection(self.port, baud=self.baudrate)
        except Exception as e:
            logger.error(f"Failed to connect to gimbal on {self.port} at {self.baudrate} baudrate: {e}")
            sys.exit(1)
        logger.info(f"Connected to gimbal on {self.port} at {self.baudrate} baudrate")

        logger.info("Starting heartbeat thread: heartbeat will be sent every {} seconds".format(1.0/self.heartbeat_frequency))
        self.heartbeat_thread = threading.Thread(target=self.send_heartbeat, daemon=True)
        self.heartbeat_thread.start()

        msg = self.get_ids()
        if msg is not None:
            logger.info("Received heartbeat from gimbal with system_id={} and component_id={}".format(self.system_id, self.component_id))
        else:
            logger.error("Timed out while waiting for heartbeat from gimbal. Check connection and try again")
            raise TimeoutError("No heartbeat received from gimbal. Check connection and try again")
        
        logger.info("Starting update thread: status will be updated every {} seconds".format(1.0/params.MESSAGE_POLL_FREQUENCY))
        self.update_thread = threading.Thread(target=self.update_status, daemon=True)
        self.update_thread.start()

        # get firmware version and serial number for logging/debugging purposes
        version_info = self.get_version()
        self.fw_version = version_info.get('version')
        self.serial_number = version_info.get('serial_number')

        # apply default parameters
        self.set_message_rates(
            orientation      = params.DEFAULT_MAV_RATE_ORIEN,
            raw_imu          = params.DEFAULT_MAV_RATE_IMU,
            encoder_count    = params.DEFAULT_MAV_RATE_ENCCNT,
            sys_status       = params.DEFAULT_MAV_RATE_ST
        )
        self.set_stiffness(
            stiffness_pitch    = params.DEFAULT_STIFF_TILT,
            stiffness_roll     = params.DEFAULT_STIFF_ROLL,
            stiffness_yaw      = params.DEFAULT_STIFF_PAN,
            holdstrength_pitch = params.DEFAULT_PWR_TILT,
            holdstrength_roll  = params.DEFAULT_PWR_ROLL,
            holdstrength_yaw   = params.DEFAULT_PWR_PAN,
            output_filter      = params.DEFAULT_FILTER_OUT,
            gyro_filter        = params.DEFAULT_GYRO_LPF
        )
        self.set_follow(
            speed_pitch        = params.DEFAULT_FLW_SP_TILT,
            speed_yaw          = params.DEFAULT_FLW_SP_PAN,
            smooth_pitch       = params.DEFAULT_FLW_LPF_TILT,
            smooth_yaw         = params.DEFAULT_FLW_LPF_PAN,
            window_pitch       = params.DEFAULT_FLW_WD_TILT,
            window_yaw         = params.DEFAULT_FLW_WD_PAN
        )
        self.set_smoothing(
            smooth_pitch       = params.DEFAULT_RC_LPF_TILT,
            smooth_roll        = params.DEFAULT_RC_LPF_ROLL,
            smooth_yaw         = params.DEFAULT_RC_LPF_PAN
        )
        self.set_speed_control(
            speed_pitch        = params.DEFAULT_RC_SPD_TILT,
            speed_roll         = params.DEFAULT_RC_SPD_ROLL,
            speed_yaw          = params.DEFAULT_RC_SPD_PAN
        )
        self.set_rotation_limits(
            min_tilt           = params.DEFAULT_RC_LIM_MIN_TILT,
            max_tilt           = params.DEFAULT_RC_LIM_MAX_TILT,
            min_roll           = params.DEFAULT_RC_LIM_MIN_ROLL,
            max_roll           = params.DEFAULT_RC_LIM_MAX_ROLL,
            min_pan            = params.DEFAULT_RC_LIM_MIN_PAN,
            max_pan            = params.DEFAULT_RC_LIM_MAX_PAN
        )
        self.set_damping(
            tilt               = params.DEFAULT_TILT_DAMPING,
            roll               = params.DEFAULT_ROLL_DAMPING,
            pan                = params.DEFAULT_PAN_DAMPING
        )
        self.set_gyro_trust(
            value              = params.DEFAULT_GYRO_TRUST
        )
        self.set_home_pan(
            angle              = params.DEFAULT_GMB_HOME_PAN
        )
        self.set_mapping_angle(
            angle              = params.DEFAULT_MAPPING_ANGLE
        )
        self.set_rc_deadzone(
            tilt               = params.DEFAULT_RC_DZONE_TILT,
            roll               = params.DEFAULT_RC_DZONE_ROLL,
            pan                = params.DEFAULT_RC_DZONE_PAN
        )
        self.set_rc_mode(
            tilt               = params.DEFAULT_RC_MODE_TILT,
            roll               = params.DEFAULT_RC_MODE_ROLL,
            pan                = params.DEFAULT_RC_MODE_PAN
        )
        self.set_rc_trim(
            tilt               = params.DEFAULT_RC_TRIM_TILT,
            roll               = params.DEFAULT_RC_TRIM_ROLL
        )
        self.set_rc_channels(
            stilt              = params.DEFAULT_RC_CHAN_STILT,
            span               = params.DEFAULT_RC_CHAN_SPAN,
            tilt               = params.DEFAULT_RC_CHAN_TILT,
            roll               = params.DEFAULT_RC_CHAN_ROLL,
            pan                = params.DEFAULT_RC_CHAN_PAN,
            mode               = params.DEFAULT_RC_CHAN_MODE
        )
        self.set_rc_config(
            rc_type            = params.DEFAULT_RC_TYPE,
            reverse_axis       = params.DEFAULT_RC_REVERSE_AXIS
        )
        self.set_mav_config(
            emit_heartbeat     = params.DEFAULT_MAV_EMIT_HB,
            ts_encoder_count   = params.DEFAULT_MAV_TS_ENCNT
        )

        # initialize goto state to ensure we have a valid current position for any subsequent goto() calls that rely on it
        self.initialize_goto()

    def disconnect(self):
        """
        Disconnect from the gimbal by stopping the heartbeat and status update threads and
        closing the serial port.
        """
        self.stop_event.set()

        if self.master is not None:
            self.master.close()
            logger.info("Disconnected from gimbal")
            self.master = None

        if self.update_thread is not None and self.update_thread.is_alive():
            self.update_thread.join(timeout=self.timeout + 1)
            logger.debug("Update thread stopped")
        self.update_thread = None

        if self.heartbeat_thread is not None and self.heartbeat_thread.is_alive():
            self.heartbeat_thread.join(timeout=self.timeout + 1)
            logger.debug("Heartbeat thread stopped")
        self.heartbeat_thread = None

        # stop telemetry loggin if still active and write any remaining data to file
        self.stop_telemetry()

    def initialize_goto(self):
        """
        Initialize the goto state by sending a neutral (0, 0, 0) position
        command, ensuring a valid current position is established for any
        subsequent goto() calls that rely on it.
        """
        logger.info("Initializing goto state")
        self.goto(yaw=0, pitch=0, roll=0)

    def goto(self, yaw=None, pitch=None, roll=None, wait=False):
        """
        Move the gimbal to the specified target angles. Any angle left as None keeps the current
        value. The gimbal moves at the speed set by set_speed(), or at its own default speed if
        no speed was set. If wait=True, this method blocks until the gimbal has stopped moving or
        a timeout occurs.

        Parameters:
        -----------
            yaw : float or None, optional
                Target yaw angle in degrees. None keeps the current yaw.
            pitch : float or None, optional
                Target pitch angle in degrees. None keeps the current pitch.
            roll : float or None, optional
                Target roll angle in degrees. None keeps the current roll.
            wait : bool, optional
                If True, block until the gimbal has reached the target position or a timeout occurs.
        """
        # resolve target angles (degrees), falling back to current position
        target_yaw   = float(yaw)   if yaw   is not None else (self.orientation.get('yaw')   if self.orientation.get('yaw')   is not None else 0.0)
        target_pitch = float(pitch) if pitch is not None else (self.orientation.get('pitch') if self.orientation.get('pitch') is not None else 0.0)
        target_roll  = float(roll)  if roll  is not None else (self.orientation.get('roll')  if self.orientation.get('roll')  is not None else 0.0)

        # check limits
        if not (params.DEFAULT_RC_LIM_MIN_PAN <= target_yaw <= params.DEFAULT_RC_LIM_MAX_PAN):
            logger.warning(f"Requested yaw {target_yaw} deg is out of limits ({params.DEFAULT_RC_LIM_MIN_PAN}, {params.DEFAULT_RC_LIM_MAX_PAN}); clamping to limits")
            target_yaw = np.clip(target_yaw, params.DEFAULT_RC_LIM_MIN_PAN, params.DEFAULT_RC_LIM_MAX_PAN)
        if not (params.DEFAULT_RC_LIM_MIN_TILT <= target_pitch <= params.DEFAULT_RC_LIM_MAX_TILT):
            logger.warning(f"Requested pitch {target_pitch} deg is out of limits ({params.DEFAULT_RC_LIM_MIN_TILT}, {params.DEFAULT_RC_LIM_MAX_TILT}); clamping to limits")
            target_pitch = np.clip(target_pitch, params.DEFAULT_RC_LIM_MIN_TILT, params.DEFAULT_RC_LIM_MAX_TILT)
        if not (params.DEFAULT_RC_LIM_MIN_ROLL <= target_roll <= params.DEFAULT_RC_LIM_MAX_ROLL):
            logger.warning(f"Requested roll {target_roll} deg is out of limits ({params.DEFAULT_RC_LIM_MIN_ROLL}, {params.DEFAULT_RC_LIM_MAX_ROLL}); clamping to limits")
            target_roll = np.clip(target_roll, params.DEFAULT_RC_LIM_MIN_ROLL, params.DEFAULT_RC_LIM_MAX_ROLL)

        # check gimbal mode: if off, lock, mapping, or reset, ignore goto
        if self.mode == 'off' or self.mode == 'lock' or self.mode == 'mapping' or self.mode == 'reset':
            logger.warning(f"Gimbal is in '{self.mode}' mode; ignoring goto command")
            return

        if self.yaw_speed is None and self.pitch_speed is None and self.roll_speed is None:
            # no speed limit set: send a single command and let the gimbal move at its own speed
            self._send_goto_deg(target_yaw, target_pitch, target_roll)
            logger.debug(f"Direct goto: (y, p, r) = {target_yaw:7.2f}, {target_pitch:7.2f}, {target_roll:7.2f} deg")
        else:
            # software-interpolated move: send intermediate waypoints to limit speed
            start_yaw   = self.orientation.get('yaw')   if self.orientation.get('yaw')   is not None else target_yaw
            start_pitch = self.orientation.get('pitch') if self.orientation.get('pitch') is not None else target_pitch
            start_roll  = self.orientation.get('roll')  if self.orientation.get('roll')  is not None else target_roll

            d_yaw   = target_yaw   - start_yaw
            d_pitch = target_pitch - start_pitch
            d_roll  = target_roll  - start_roll

            # time required for each axis at the requested speed; NaN speed = instant
            t_yaw   = abs(d_yaw)   / self.yaw_speed   if self.yaw_speed   else 0.0
            t_pitch = abs(d_pitch) / self.pitch_speed if self.pitch_speed else 0.0
            t_roll  = abs(d_roll)  / self.roll_speed  if self.roll_speed  else 0.0
            total_time = max(t_yaw, t_pitch, t_roll)

            if total_time < 0.05:
                # distance too small or speed too high, just send directly
                self._send_goto_deg(target_yaw, target_pitch, target_roll)
                logger.debug(f"Direct goto: (y, p, r) = {target_yaw:7.2f}, {target_pitch:7.2f}, {target_roll:7.2f}")
            else:
                step_interval = 0.05  # seconds between waypoints (~20 Hz)
                n_steps = max(1, int(total_time / step_interval))
                for i in range(1, n_steps + 1):
                    elapsed = i * step_interval

                    # each axis advances at its own speed and clamps once it reaches the target
                    self._send_goto_deg(
                        utils.interp(start_yaw,   target_yaw,   t_yaw,   elapsed),
                        utils.interp(start_pitch, target_pitch, t_pitch, elapsed),
                        utils.interp(start_roll,  target_roll,  t_roll,  elapsed),
                    )
                    if i < n_steps:
                        time.sleep(step_interval)
                logger.debug(f"Interp goto: (y, p, r) = {target_yaw:7.2f}, {target_pitch:7.2f}, {target_roll:7.2f} deg, time = {total_time:.2f} s, steps = {n_steps}")
        if wait:
            self.wait_until_stopped(velocity_threshold=0.01)

    def is_moving(self, velocity_threshold=0.01):
        """
        Check if the gimbal is currently moving by comparing the absolute values of the angular
        velocities reported by GIMBAL_DEVICE_ATTITUDE_STATUS against a threshold.

        Parameters:
        -----------
            velocity_threshold : float, optional
                Minimum absolute angular velocity (rad/s) to consider the gimbal as moving.
                Default is 0.01 rad/s, which accounts for small fluctuations at rest.

        Returns:
        --------
            bool: True if any axis angular velocity exceeds the threshold, False otherwise.
        """
        vx = self.attitude.get('angular_velocity_x')
        vy = self.attitude.get('angular_velocity_y')
        vz = self.attitude.get('angular_velocity_z')
        if vx is None or vy is None or vz is None:
            return False
        return any(abs(v) > velocity_threshold for v in [vx, vy, vz])

    def wait_until_stopped(self, velocity_threshold=0.01, timeout=15):
        """
        Block until the gimbal has stopped moving (i.e. all angular velocities fall below the
        specified threshold) or the timeout expires.

        Parameters:
        -----------
            velocity_threshold : float, optional
                Minimum absolute angular velocity (rad/s) to consider the gimbal as moving.
                Default is 0.01 rad/s.
            timeout : float, optional
                Maximum time in seconds to wait before giving up. Default is 15 seconds.

        Returns:
        --------
            bool: True if the gimbal stopped within the timeout period, False otherwise.
        """
        time.sleep(0.5)  # initial delay to allow movement to start 
        start = time.time()
        while time.time() - start < timeout:
            if not self.is_moving(velocity_threshold=velocity_threshold):
                return True
            time.sleep(1.0 / params.MESSAGE_POLL_FREQUENCY)

        logger.warning(f"Gimbal did not stop within {timeout} seconds")
        return False
    
    def log_telemetry(self, message_dict: dict):
        """
        Add a telemetry message dictionary to the log buffer and write to file if the buffer exceeds the configured size.

        Parameters:
        -----------
            message_dict : dict
                A dictionary containing telemetry data to be logged.
        """
        self._telemetry_log_buffer.append(message_dict)
        if len(self._telemetry_log_buffer) >= params.TELEMETRY_LOG_BUFFER:
            self._write_telemetry_log()

    def _write_telemetry_log(self):
        """
        Write the contents of the telemetry log buffer to a binary file using the pickle module.
        """
        if not self._telemetry_log_buffer:
            logger.debug(f"Empty telemetry log buffer, nothing to write to {self.telemetry_log_filename}")
            return
        
        try:
            with open(self.telemetry_log_filename, 'ab') as f:
                pickle.dump(self._telemetry_log_buffer, f)
                self._telemetry_log_buffer.clear()
                logger.debug(f"Telemetry log written to {self.telemetry_log_filename}")
        except Exception as e:
            logger.error(f"Failed to write telemetry log: {e}")
    
    def update_status_simulator(self):
        """
        Simulated status update loop for testing without a real gimbal connection. This method
        generates synthetic telemetry data at regular intervals and updates the gimbal's instance
        variables accordingly, mimicking the behavior of receiving real MAVLink messages.
        """
        try: 
            while not self.stop_event.is_set():
                logger.debug("Simulating telemetry update...")
                # Generate synthetic telemetry data (replace with more realistic simulation as needed)
                timestamp = time.time()
                attitude_status = {
                    'angular_velocity_x': np.random.uniform(-0.05, 0.05),
                    'angular_velocity_y': np.random.uniform(-0.05, 0.05),
                    'angular_velocity_z': np.random.uniform(-0.05, 0.05),
                    'time_boot_ms': int(timestamp * 1000) % 2**32,
                    'q': utils.euler_to_quaternion(
                        np.random.uniform(-180, 180),  # yaw
                        np.random.uniform(-90, 90),    # pitch
                        np.random.uniform(-45, 45)     # roll
                    ),
                    'delta_yaw_velocity': np.random.uniform(-0.05, 0.05),
                }
                orientation_status = {
                    'yaw': np.random.uniform(-180, 180),
                    'pitch': np.random.uniform(-90, 90),
                    'roll': np.random.uniform(-45, 45),
                    'yaw_absolute': np.random.uniform(-180, 180),
                }
                raw_imu = {
                    'time_usec': int(timestamp * 1e6) % 2**64,
                    'xacc': np.random.uniform(-2, 2),
                    'yacc': np.random.uniform(-2, 2),
                    'zacc': np.random.uniform(-2, 2),
                    'xgyro': np.random.uniform(-0.1, 0.1),
                    'ygyro': np.random.uniform(-0.1, 0.1),
                    'zgyro': np.random.uniform(-0.1, 0.1),
                    'temperature': np.random.uniform(20, 40),
                    'xmag': np.random.uniform(-50, 50),
                    'ymag': np.random.uniform(-50, 50),
                    'zmag': np.random.uniform(-50, 50),
                }
                mount_status = {
                    'pointing_a': np.random.uniform(-180, 180),
                    'pointing_b': np.random.uniform(-90, 90),
                    'pointing_c': np.random.uniform(-45, 45),
                    'mount_mode': np.random.choice([0, 1, 2, 3])
                }
                sys_status = {
                    'load': np.random.randint(0, 100),
                    'voltage_battery': np.random.randint(11000, 13000),
                    'current_battery': np.random.randint(0, 2000),
                    'battery_remaining': np.random.randint(0, 100),
                    'drop_rate_comm': np.random.uniform(0, 1),
                    'errors_comm': np.random.randint(0, 10),
                    'errors_count1': np.random.randint(0, 10),
                    'errors_count2': np.random.randint(0, 10),
                    'errors_count3': np.random.randint(0, 10),
                    'errors_count4': np.random.randint(0, 10),
                }
                self._handle_attitude_status(attitude_status)
                self._handle_mount_orientation(orientation_status)
                self._handle_raw_imu(raw_imu)
                self._handle_mount_status(mount_status)
                self._handle_sys_status(sys_status)

                time.sleep(0.1)  # simulate message arrival rate of 10Hz
        except Exception as e:
            logger.error(f"Error in simulator update thread: {e}")
            self.stop_event.set()

    def update_status(self):
        """
        Continuously listen for incoming MAVLink messages and update the gimbal's instance
        variables whenever a relevant message is received. Uses message_dispatcher to handle
        each message type via a dedicated callback, replacing the previous sequential polling
        approach.
        """
        handlers = {
            kw.GIMBAL_DEVICE_ATTITUDE_STATUS_KEYWORD: self._handle_attitude_status,
            kw.MOUNT_ORIENTATION_KEYWORD:             self._handle_mount_orientation,
            kw.RAW_IMU_KEYWORD:                       self._handle_raw_imu,
            kw.MOUNT_STATUS_KEYWORD:                  self._handle_mount_status,
            kw.SYS_STATUS_KEYWORD:                    self._handle_sys_status,
            kw.HEARTBEAT_KEYWORD:                     self._handle_heartbeat,
            kw.COMMAND_ACK_KEYWORD:                   self._handle_command_ack,
            kw.PARAM_VALUE_KEYWORD:                   self._handle_param_value,
            "*":                                      self._handle_unhandled_message,
        }
        try:
            utils.message_dispatcher(self.master, handlers, self.stop_event,
                                     poll_frequency=params.MESSAGE_POLL_FREQUENCY)
        except Exception as e:
            logger.error(f"Update thread crashed: {e}")
            self.stop_event.set()

    def _handle_attitude_status(self, d):
        """Store the latest GIMBAL_DEVICE_ATTITUDE_STATUS message payload."""
        logger.debug(f"Received GIMBAL_DEVICE_ATTITUDE_STATUS")
        if self._log_telemetry_flag:
            timestamp = time.time()
            self.log_telemetry({
                "timestamp": timestamp,
                "keykord": kw.GIMBAL_DEVICE_ATTITUDE_STATUS_KEYWORD,
                "data": d
            })
        self.attitude = d
        self.telemetry_state.update(d)

    def _handle_mount_orientation(self, d):
        """Store the latest MOUNT_ORIENTATION message payload."""
        logger.debug(f"Received MOUNT_ORIENTATION")
        if self._log_telemetry_flag:
            timestamp = time.time()
            self.log_telemetry({
                "timestamp": timestamp,
                "keykord": kw.MOUNT_ORIENTATION_KEYWORD,
                "data": d
            })
        self.orientation = d
        self.telemetry_state.update(d)
    def _handle_raw_imu(self, d):
        """Store the latest RAW_IMU message payload."""
        logger.debug(f"Received RAW_IMU")
        if self._log_telemetry_flag:
            timestamp = time.time()
            self.log_telemetry({
                "timestamp": timestamp,
                "keykord": kw.RAW_IMU_KEYWORD,
                "data": d
            })
        self.raw_imu = d
        self.telemetry_state.update(d)

    def _handle_mount_status(self, d):
        """Store the latest MOUNT_STATUS message payload."""
        logger.debug(f"Received MOUNT_STATUS")
        if self._log_telemetry_flag:
            timestamp = time.time()
            self.log_telemetry({
                "timestamp": timestamp,
                "keykord": kw.MOUNT_STATUS_KEYWORD,
                "data": d
            })
        self.mount_status = d
        self.telemetry_state.update(d)

    def _handle_sys_status(self, d):
        """Store the latest SYS_STATUS message payload."""
        logger.debug(f"Received SYS_STATUS")
        if self._log_telemetry_flag:
            timestamp = time.time()
            self.log_telemetry({
                "timestamp": timestamp,
                "keykord": kw.SYS_STATUS_KEYWORD,
                "data": d
            })
        self.sys_status = d
        self.telemetry_state.update(d)

    def _handle_heartbeat(self, d):
        """Store the latest HEARTBEAT message payload."""
        logger.debug(f"Received HEARTBEAT")
        if self._log_telemetry_flag:
            timestamp = time.time()
            self.log_telemetry({
                "timestamp": timestamp,
                "keykord": kw.HEARTBEAT_KEYWORD,
                "data": d
            })
        self.heartbeat_dict = d
        self.telemetry_state.update(d)

    def _handle_command_ack(self, d):
        """Store the latest COMMAND_ACK payload and warn on non-accepted results."""
        self.acknowledgment = d
        if utils.ack_results.get(d['result']) != "Accepted":
            logger.warning(f"Received command acknowledgment with result: {utils.ack_results.get(d['result'])}")

    def _handle_param_value(self, d):
        """
        Store received PARAM_VALUE messages in a dictionary keyed by param_id string for 
        retrieval by _fetch_param().
        """
        if self._log_telemetry_flag:
            timestamp = time.time()
            self.log_telemetry({
                "timestamp": timestamp,
                "keykord": kw.PARAM_VALUE_KEYWORD,
                "data": d
            })
        param_id = d.get('param_id', '').rstrip('\x00')
        self.param_values[param_id] = d
        self.telemetry_state.update(d)

    def _handle_unhandled_message(self, d):
        """
        Default handler for any MAVLink message types that don't have a specific handler.
        """
        logger.debug(f"Unhandled message: {d}")

    def _param_id_bytes(self, name):
        """
        Return a 16-byte zero-padded bytes object for a MAVLink param_id.
        """
        return name.encode('ascii').ljust(16, b'\x00')

    def _request_param(self, param_id):
        """
        Send a PARAM_REQUEST_READ for the given param_id string, asking the
        gimbal to reply with a PARAM_VALUE message.
        """
        self.master.mav.param_request_read_send(
            target_system=self.system_id,
            target_component=self.component_id,
            param_id=self._param_id_bytes(param_id),
            param_index=-1  # -1 = look up by name, not index
        )

    def _set_param(self, param_id, value):
        """
        Send a PARAM_SET for the given param_id string and int16 value.
        """
        self.master.mav.param_set_send(
            target_system=self.system_id,
            target_component=self.component_id,
            param_id=self._param_id_bytes(param_id),
            param_value=float(value),
            param_type=mavutil.mavlink.MAV_PARAM_TYPE_INT16
        )
        logger.debug(f"PARAM_SET sent: {param_id} = {value}")

    def _fetch_param(self, param_id, timeout=2.0, retry_interval=0.2):
        """
        Request a parameter from the gimbal and wait up to *timeout* seconds
        for the PARAM_VALUE reply that the update thread stores in self.param_values.

        The request is **re-sent every retry_interval seconds** so that a single
        lost packet (e.g. when the serial bus is saturated by high-frequency
        telemetry) does not cause the whole fetch to fail.

        Parameters:
        -----------
            param_id : str
                The MAVLink parameter name (e.g. "STIFF_TILT").
            timeout : float
                Total seconds to wait for the reply.
            retry_interval : float
                Seconds between successive PARAM_REQUEST_READ retransmissions.

        Returns:
        --------
            int or None: The integer value of the parameter, or None on timeout.
        """
        # Clear any stale cached value so we can detect the fresh reply.
        self.param_values.pop(param_id, None)
        deadline     = time.time() + timeout
        next_request = 0.0  # send immediately on first iteration
        while time.time() < deadline:
            if time.time() >= next_request:
                self._request_param(param_id)
                next_request = time.time() + retry_interval
            d = self.param_values.get(param_id)
            if d is not None:
                return int(d['param_value'])
            time.sleep(0.02)
        logger.warning(f"Timeout waiting for PARAM_VALUE reply for '{param_id}' "
                       f"(timeout={timeout}s, retry_interval={retry_interval}s)")
        return None
    
    def _send_goto_deg(self, yaw_deg, pitch_deg, roll_deg):
        """
        Send a single GIMBAL_DEVICE_SET_ATTITUDE MAVLink command with the given target angles.
        This is a low-level helper used internally by goto().

        Parameters:
        -----------
            yaw_deg : float
                Target yaw angle in degrees.
            pitch_deg : float
                Target pitch angle in degrees.
            roll_deg : float
                Target roll angle in degrees.
        """
        q = quaternion.QuaternionBase([np.radians(roll_deg),
                                       np.radians(pitch_deg),
                                       np.radians(yaw_deg)])
        self.master.mav.gimbal_device_set_attitude_send(
            target_system=self.system_id,
            target_component=self.component_id,
            flags=self.reference_frame_flags,
            q=q,
            angular_velocity_x=float('nan'),
            angular_velocity_y=float('nan'),
            angular_velocity_z=float('nan'),
        )

    
    # ------------------------------------------------------------------
    #                           Get methods
    # ------------------------------------------------------------------

    def get_ids(self):
        """
        Wait for a heartbeat message from the gimbal and extract the system and component IDs.
        This is necessary to know the target_system and target_component for subsequent messages.

        Returns:
        --------
            MAVLink message or None: The received heartbeat message, or None if no heartbeat
            was received within the timeout period.
        """
        msg = self.master.wait_heartbeat(timeout=self.timeout)
        if msg is not None:
            self.system_id = msg.get_srcSystem()
            self.component_id = msg.get_srcComponent()
            return msg
        
        logger.warning("No heartbeat received from gimbal within timeout period")
        return None

    def get_stiffness(self, timeout=4.0, retry_interval=0.2):
        """
        Read the stiffness and hold-strength settings from the gimbal.

        Each parameter request is re-sent every *retry_interval* seconds until
        a reply is received or *timeout* elapses. Increase *timeout* or decrease
        *retry_interval* when the serial bus is saturated by high-frequency
        telemetry (e.g. 100 Hz orientation messages).

        Parameters:
        -----------
            timeout : float
                Seconds to wait for each parameter reply (default 4 s).
            retry_interval : float
                Seconds between successive retransmissions of a pending request
                (default 0.2 s).

        Returns:
        --------
            dict: Keys are 'stiffness_pitch', 'stiffness_roll', 'stiffness_yaw',
                  'holdstrength_pitch', 'holdstrength_roll', 'holdstrength_yaw',
                  'output_filter', 'gyro_filter'. Values are ints or None on timeout.
        """
        params = {
            'stiffness_pitch':    kw.PARAM_STIFF_TILT,
            'stiffness_roll':     kw.PARAM_STIFF_ROLL,
            'stiffness_yaw':      kw.PARAM_STIFF_PAN,
            'holdstrength_pitch': kw.PARAM_PWR_TILT,
            'holdstrength_roll':  kw.PARAM_PWR_ROLL,
            'holdstrength_yaw':   kw.PARAM_PWR_PAN,
            'output_filter':      kw.PARAM_FILTER_OUT,
            'gyro_filter':        kw.PARAM_GYRO_LPF,
        }
        result = {}
        for key, param_id in params.items():
            val = self._fetch_param(param_id, timeout=timeout, retry_interval=retry_interval)
            result[key] = val
            if val is not None:
                logger.info(f"Get: {param_id:16s} = {val}")
        return result

    def get_follow(self, timeout=4.0, retry_interval=0.2):
        """
        Read the follow-mode parameters from the gimbal (speed, smooth, window
        for tilt/pitch and pan/yaw axes). These parameters are only active when
        the gimbal is in follow mode.

        Each parameter request is retransmitted every *retry_interval* seconds
        until a reply is received or *timeout* elapses.

        Parameters:
        -----------
            timeout : float
                Seconds to wait for each parameter reply (default 4 s).
            retry_interval : float
                Seconds between successive retransmissions (default 0.2 s).

        Returns:
        --------
            dict with keys:
                'speed_pitch'  -- how fast the gimbal tracks body pitch changes (deg/s)
                'speed_yaw'    -- how fast the gimbal tracks body yaw changes (deg/s)
                'smooth_pitch' -- low-pass filter strength on the follow error, pitch [0-100]
                'smooth_yaw'   -- low-pass filter strength on the follow error, yaw [0-100]
                'window_pitch' -- dead-band half-width on pitch (degrees)
                'window_yaw'   -- dead-band half-width on yaw (degrees)
            Values are ints or None on timeout.
        """
        params = {
            'speed_pitch':  kw.PARAM_FLW_SP_TILT,
            'speed_yaw':    kw.PARAM_FLW_SP_PAN,
            'smooth_pitch': kw.PARAM_FLW_LPF_TILT,
            'smooth_yaw':   kw.PARAM_FLW_LPF_PAN,
            'window_pitch': kw.PARAM_FLW_WD_TILT,
            'window_yaw':   kw.PARAM_FLW_WD_PAN,
        }
        result = {}
        for key, param_id in params.items():
            val = self._fetch_param(param_id, timeout=timeout, retry_interval=retry_interval)
            result[key] = val
            if val is not None:
                logger.info(f"Get: {param_id:16s} = {val}")
        return result

    def get_smoothing(self, timeout=4.0, retry_interval=0.2):
        """
        Read the command-smoothing parameters from the gimbal (MAVLink tab in
        gTune). These are low-pass filters applied to every incoming angle
        command (goto), active in all non-off modes.

        Parameters:
        -----------
            timeout : float
                Seconds to wait for each parameter reply (default 4 s).
            retry_interval : float
                Seconds between successive retransmissions (default 0.2 s).

        Returns:
        --------
            dict with keys 'smooth_pitch', 'smooth_roll', 'smooth_yaw'.
            Values are ints [0-100] or None on timeout.
        """
        params = {
            'smooth_pitch': kw.PARAM_RC_LPF_TILT,
            'smooth_roll':  kw.PARAM_RC_LPF_ROLL,
            'smooth_yaw':   kw.PARAM_RC_LPF_PAN,
        }
        result = {}
        for key, param_id in params.items():
            val = self._fetch_param(param_id, timeout=timeout, retry_interval=retry_interval)
            result[key] = val
            if val is not None:
                logger.info(f"Get: {param_id:16s} = {val}")
        return result

    def get_speed_control(self, timeout=4.0, retry_interval=0.2):
        """
        Read the command-speed parameters from the gimbal (MAVLink tab in
        gTune). These cap the maximum angular rate used in rate-control mode.

        Parameters:
        -----------
            timeout : float
                Seconds to wait for each parameter reply (default 4 s).
            retry_interval : float
                Seconds between successive retransmissions (default 0.2 s).

        Returns:
        --------
            dict with keys 'speed_pitch', 'speed_roll', 'speed_yaw'.
            Values are ints [deg/s] or None on timeout.
        """
        params = {
            'speed_pitch': kw.PARAM_RC_SPD_TILT,
            'speed_roll':  kw.PARAM_RC_SPD_ROLL,
            'speed_yaw':   kw.PARAM_RC_SPD_PAN,
        }
        result = {}
        for key, param_id in params.items():
            val = self._fetch_param(param_id, timeout=timeout, retry_interval=retry_interval)
            result[key] = val
            if val is not None:
                logger.info(f"Get: {param_id:16s} = {val}")
        return result

    def get_rotation_limits(self, timeout=4.0, retry_interval=0.2):
        """
        Read the software rotation end-stops from the gimbal.

        Parameters:
        -----------
            timeout : float
                Seconds to wait for each parameter reply (default 4 s).
            retry_interval : float
                Seconds between successive retransmissions (default 0.2 s).

        Returns:
        --------
            dict with keys 'min_tilt', 'max_tilt', 'min_roll', 'max_roll',
            'min_pan', 'max_pan'. Values are ints [degrees] or None on timeout.
        """
        params = {
            'min_tilt': kw.PARAM_RC_LIM_MIN_TILT,
            'max_tilt': kw.PARAM_RC_LIM_MAX_TILT,
            'min_roll': kw.PARAM_RC_LIM_MIN_ROLL,
            'max_roll': kw.PARAM_RC_LIM_MAX_ROLL,
            'min_pan':  kw.PARAM_RC_LIM_MIN_PAN,
            'max_pan':  kw.PARAM_RC_LIM_MAX_PAN,
        }
        result = {}
        for key, param_id in params.items():
            val = self._fetch_param(param_id, timeout=timeout, retry_interval=retry_interval)
            result[key] = val
            if val is not None:
                logger.info(f"Get: {param_id:20s} = {val}")
        return result

    def get_damping(self, timeout=4.0, retry_interval=0.2):
        """
        Read the motor output damping parameters from the gimbal.

        Parameters:
        -----------
            timeout : float
                Seconds to wait for each parameter reply (default 4 s).
            retry_interval : float
                Seconds between successive retransmissions (default 0.2 s).

        Returns:
        --------
            dict with keys 'tilt', 'roll', 'pan'. Values are ints [0-100]
            or None on timeout.
        """
        params = {
            'tilt': kw.PARAM_TILT_DAMPING,
            'roll': kw.PARAM_ROLL_DAMPING,
            'pan':  kw.PARAM_PAN_DAMPING,
        }
        result = {}
        for key, param_id in params.items():
            val = self._fetch_param(param_id, timeout=timeout, retry_interval=retry_interval)
            result[key] = val
            if val is not None:
                logger.info(f"Get: {param_id:20s} = {val}")
        return result

    def get_gyro_trust(self, timeout=4.0, retry_interval=0.2):
        """
        Read the gyro sensor trust value from the gimbal.

        Parameters:
        -----------
            timeout : float
                Seconds to wait for the parameter reply (default 4 s).
            retry_interval : float
                Seconds between successive retransmissions (default 0.2 s).

        Returns:
        --------
            int [0-255] or None on timeout.
        """
        val = self._fetch_param(kw.PARAM_GYRO_TRUST, timeout=timeout, retry_interval=retry_interval)
        if val is not None:
            logger.info(f"Get: GYRO_TRUST           = {val}")
        return val

    def get_home_pan(self, timeout=4.0, retry_interval=0.2):
        """
        Read the home pan/yaw angle from the gimbal.

        Parameters:
        -----------
            timeout : float
                Seconds to wait for the parameter reply (default 4 s).
            retry_interval : float
                Seconds between successive retransmissions (default 0.2 s).

        Returns:
        --------
            int [degrees] or None on timeout.
        """
        val = self._fetch_param(kw.PARAM_GMB_HOME_PAN, timeout=timeout, retry_interval=retry_interval)
        if val is not None:
            logger.info(f"Get: GMB_HOME_PAN          = {val}")
        return val

    def get_mapping_angle(self, timeout=4.0, retry_interval=0.2):
        """
        Read the mapping mode tilt angle from the gimbal.

        Parameters:
        -----------
            timeout : float
                Seconds to wait for the parameter reply (default 4 s).
            retry_interval : float
                Seconds between successive retransmissions (default 0.2 s).

        Returns:
        --------
            int [degrees] or None on timeout.
        """
        val = self._fetch_param(kw.PARAM_MAPPING_ANGLE, timeout=timeout, retry_interval=retry_interval)
        if val is not None:
            logger.info(f"Get: MAPPING_ANGLE         = {val}")
        return val

    def get_rc_deadzone(self, timeout=4.0, retry_interval=0.2):
        """
        Read the RC dead-zone values from the gimbal.

        Parameters:
        -----------
            timeout : float
                Seconds to wait for each parameter reply (default 4 s).
            retry_interval : float
                Seconds between successive retransmissions (default 0.2 s).

        Returns:
        --------
            dict with keys 'tilt', 'roll', 'pan'. Values are ints [0-100]
            or None on timeout.
        """
        params = {
            'tilt': kw.PARAM_RC_DZONE_TILT,
            'roll': kw.PARAM_RC_DZONE_ROLL,
            'pan':  kw.PARAM_RC_DZONE_PAN,
        }
        result = {}
        for key, param_id in params.items():
            val = self._fetch_param(param_id, timeout=timeout, retry_interval=retry_interval)
            result[key] = val
            if val is not None:
                logger.info(f"Get: {param_id:20s} = {val}")
        return result

    def get_rc_mode(self, timeout=4.0, retry_interval=0.2):
        """
        Read the RC control mode for each axis (0 = speed, 1 = angle).

        Parameters:
        -----------
            timeout : float
                Seconds to wait for each parameter reply (default 4 s).
            retry_interval : float
                Seconds between successive retransmissions (default 0.2 s).

        Returns:
        --------
            dict with keys 'tilt', 'roll', 'pan'. Values are 0 or 1, or None
            on timeout.
        """
        params = {
            'tilt': kw.PARAM_RC_MODE_TILT,
            'roll': kw.PARAM_RC_MODE_ROLL,
            'pan':  kw.PARAM_RC_MODE_PAN,
        }
        result = {}
        for key, param_id in params.items():
            val = self._fetch_param(param_id, timeout=timeout, retry_interval=retry_interval)
            result[key] = val
            if val is not None:
                logger.info(f"Get: {param_id:20s} = {val}")
        return result

    def get_rc_trim(self, timeout=4.0, retry_interval=0.2):
        """
        Read the RC trim (home offset) for tilt and roll axes.

        Parameters:
        -----------
            timeout : float
                Seconds to wait for each parameter reply (default 4 s).
            retry_interval : float
                Seconds between successive retransmissions (default 0.2 s).

        Returns:
        --------
            dict with keys 'tilt', 'roll'. Values are ints [degrees] or None
            on timeout.
        """
        params = {
            'tilt': kw.PARAM_RC_TRIM_TILT,
            'roll': kw.PARAM_RC_TRIM_ROLL,
        }
        result = {}
        for key, param_id in params.items():
            val = self._fetch_param(param_id, timeout=timeout, retry_interval=retry_interval)
            result[key] = val
            if val is not None:
                logger.info(f"Get: {param_id:20s} = {val}")
        return result

    def get_rc_channels(self, timeout=4.0, retry_interval=0.2):
        """
        Read the RC channel assignments from the gimbal.

        Parameters:
        -----------
            timeout : float
                Seconds to wait for each parameter reply (default 4 s).
            retry_interval : float
                Seconds between successive retransmissions (default 0.2 s).

        Returns:
        --------
            dict with keys 'stilt', 'span', 'tilt', 'roll', 'pan', 'mode'.
            Values are channel numbers (int) or None on timeout.
        """
        params = {
            'stilt': kw.PARAM_RC_CHAN_STILT,
            'span':  kw.PARAM_RC_CHAN_SPAN,
            'tilt':  kw.PARAM_RC_CHAN_TILT,
            'roll':  kw.PARAM_RC_CHAN_ROLL,
            'pan':   kw.PARAM_RC_CHAN_PAN,
            'mode':  kw.PARAM_RC_CHAN_MODE,
        }
        result = {}
        for key, param_id in params.items():
            val = self._fetch_param(param_id, timeout=timeout, retry_interval=retry_interval)
            result[key] = val
            if val is not None:
                logger.info(f"Get: {param_id:20s} = {val}")
        return result

    def get_rc_config(self, timeout=4.0, retry_interval=0.2):
        """
        Read miscellaneous RC configuration from the gimbal.

        Parameters:
        -----------
            timeout : float
                Seconds to wait for each parameter reply (default 4 s).
            retry_interval : float
                Seconds between successive retransmissions (default 0.2 s).

        Returns:
        --------
            dict with keys 'rc_type' and 'reverse_axis'. Values are ints or
            None on timeout.
        """
        params = {
            'rc_type':      kw.PARAM_RC_TYPE,
            'reverse_axis': kw.PARAM_RC_REVERSE_AXIS,
        }
        result = {}
        for key, param_id in params.items():
            val = self._fetch_param(param_id, timeout=timeout, retry_interval=retry_interval)
            result[key] = val
            if val is not None:
                logger.info(f"Get: {param_id:20s} = {val}")
        return result

    def get_mav_config(self, timeout=4.0, retry_interval=0.2):
        """
        Read MAVLink interface configuration parameters from the gimbal.

        Parameters:
        -----------
            timeout : float
                Seconds to wait for each parameter reply (default 4 s).
            retry_interval : float
                Seconds between successive retransmissions (default 0.2 s).

        Returns:
        --------
            dict with keys 'emit_heartbeat', 'ts_encoder_count',
            'baudrate_com2', 'baudrate_com4', 'component_id'.
            Baudrates are stored as ×100 values (e.g. 1152 = 115200 baud).
            Values are ints or None on timeout.
        """
        params = {
            'emit_heartbeat':   kw.PARAM_MAV_EMIT_HB,
            'ts_encoder_count': kw.PARAM_MAV_TS_ENCNT,
            'baudrate_com2':    kw.PARAM_BAUDRATE_COM2,
            'baudrate_com4':    kw.PARAM_BAUDRATE_COM4,
            'component_id':     kw.PARAM_GIMBAL_COMPID,
        }
        result = {}
        for key, param_id in params.items():
            val = self._fetch_param(param_id, timeout=timeout, retry_interval=retry_interval)
            result[key] = val
            if val is not None:
                logger.info(f"Get: {param_id:20s} = {val}")
        return result

    def get_message_rates(self, timeout=4.0, retry_interval=0.2):
        """
        Read the MAVLink message output rates from the gimbal.

        Parameters:
        -----------
            timeout : float
                Seconds to wait for each parameter reply (default 4 s).
            retry_interval : float
                Seconds between successive retransmissions (default 0.2 s).

        Returns:
        --------
            dict with keys matching the firmware param names:
            'MAV_RATE_ORIEN', 'MAV_RATE_IMU', 'MAV_RATE_ENCCNT',
            'MAV_RATE_ST'.
            Values are ints [Hz] or None on timeout.
        """
        params = {
            'MAV_RATE_ORIEN':  kw.PARAM_MAV_RATE_ORIEN,
            'MAV_RATE_IMU':    kw.PARAM_MAV_RATE_IMU,
            'MAV_RATE_ENCCNT': kw.PARAM_MAV_RATE_ENCCNT,
            'MAV_RATE_ST':     kw.PARAM_MAV_RATE_ST
        }
        result = {}
        for key, param_id in params.items():
            val = self._fetch_param(param_id, timeout=timeout, retry_interval=retry_interval)
            result[key] = val
            if val is not None:
                logger.info(f"Get: {param_id:20s} = {val}")
        return result

    def get_version(self, timeout=4.0, retry_interval=0.2):
        """
        Read the firmware version and serial number from the gimbal.

        Parameters:
        -----------
            timeout : float
                Seconds to wait for each parameter reply (default 4 s).
            retry_interval : float
                Seconds between successive retransmissions (default 0.2 s).

        Returns:
        --------
            dict with keys 'version' (str "X.Y.Z"), 'version_x', 'version_y',
            'version_z' (ints), and 'serial_number' (int or None on timeout).
        """
        params = {
            'version_x':     kw.PARAM_VERSION_X,
            'version_y':     kw.PARAM_VERSION_Y,
            'version_z':     kw.PARAM_VERSION_Z,
            'serial_number': kw.PARAM_SRL_NUMBER,
        }
        result = {}
        for key, param_id in params.items():
            val = self._fetch_param(param_id, timeout=timeout, retry_interval=retry_interval)
            result[key] = val
        x = result.get('version_x')
        y = result.get('version_y')
        z = result.get('version_z')
        result['version'] = f"{x}.{y}.{z}" if None not in (x, y, z) else None
        if result['version'] is not None:
            logger.info(f"Firmware version: {result['version']} serial number: {result['serial_number']}")
        return result

    # ------------------------------------------------------------------
    #                           Set methods
    # ------------------------------------------------------------------

    def set_stiffness(self, stiffness_pitch=None, stiffness_roll=None, stiffness_yaw=None,
                      holdstrength_pitch=None, holdstrength_roll=None, holdstrength_yaw=None,
                      output_filter=None, gyro_filter=None):
        """
        Set motor control parameters (stiffness and/or hold strength) on the gimbal.
        Only the parameters that are not None are sent.

        Parameters:
        -----------
            stiffness_pitch : int or None
                Pitch stiffness [0-100]. Higher values make the gimbal resist disturbances
                more aggressively. Increase until oscillation, then back off.
            stiffness_roll : int or None
                Roll stiffness [0-100].
            stiffness_yaw : int or None
                Yaw (pan) stiffness [0-100].
            holdstrength_pitch : int or None
                Pitch hold strength / motor power [0-100]. Default is 40.
            holdstrength_roll : int or None
                Roll hold strength [0-100].
            holdstrength_yaw : int or None
                Yaw hold strength [0-100].
            output_filter : int or None
                Output filter coefficient for denoising [0-100]. Default is 3.
            gyro_filter : int or None
                Gyro LPF filter coefficient [0-100]. Default is 2.
        """
        to_send = [
            (kw.PARAM_STIFF_TILT,  stiffness_pitch),
            (kw.PARAM_STIFF_ROLL,  stiffness_roll),
            (kw.PARAM_STIFF_PAN,   stiffness_yaw),
            (kw.PARAM_PWR_TILT,    holdstrength_pitch),
            (kw.PARAM_PWR_ROLL,    holdstrength_roll),
            (kw.PARAM_PWR_PAN,     holdstrength_yaw),
            (kw.PARAM_FILTER_OUT,  output_filter),
            (kw.PARAM_GYRO_LPF,    gyro_filter),
        ]
        for param_id, value in to_send:
            if value is not None:
                self._set_param(param_id, int(value))
                logger.info(f"Set: {param_id:16s} = {value}")

    def set_follow(self, speed_pitch=None, speed_yaw=None,
                   smooth_pitch=None, smooth_yaw=None,
                   window_pitch=None, window_yaw=None):
        """
        Set follow-mode parameters on the gimbal. Only parameters that are not
        None are sent. Changes take effect immediately if the gimbal is already
        in follow mode; otherwise they are stored and applied when follow mode
        is next activated.

        Parameters:
        -----------
            speed_pitch : int or None
                Maximum angular velocity at which the pitch axis tracks the
                vehicle body in follow mode (deg/s). Lower = lazier tracking.
            speed_yaw : int or None
                Maximum angular velocity at which the yaw axis tracks the
                vehicle body in follow mode (deg/s).
            smooth_pitch : int or None
                Low-pass filter on the pitch follow error [0-100]. Higher =
                more smoothing, slower but more cinematic response.
            smooth_yaw : int or None
                Low-pass filter on the yaw follow error [0-100].
            window_pitch : int or None
                Dead-band half-width on pitch (degrees). The gimbal does not
                begin tracking until body pitch moves outside this window,
                preventing micro-jitter from triggering follow motion.
            window_yaw : int or None
                Dead-band half-width on yaw (degrees).
        """
        to_send = [
            (kw.PARAM_FLW_SP_TILT,  speed_pitch),
            (kw.PARAM_FLW_SP_PAN,   speed_yaw),
            (kw.PARAM_FLW_LPF_TILT, smooth_pitch),
            (kw.PARAM_FLW_LPF_PAN,  smooth_yaw),
            (kw.PARAM_FLW_WD_TILT,  window_pitch),
            (kw.PARAM_FLW_WD_PAN,   window_yaw),
        ]
        for param_id, value in to_send:
            if value is not None:
                self._set_param(param_id, int(value))
                logger.info(f"Set: {param_id:16s} = {value}")

    def set_smoothing(self, smooth_pitch=None, smooth_roll=None, smooth_yaw=None):
        """
        Set the command-smoothing parameters on the gimbal. These low-pass
        filters are applied to every incoming angle command (goto), in all
        non-off modes. Only parameters that are not None are sent.

        Parameters:
        -----------
            smooth_pitch : int or None
                Low-pass filter strength on pitch commands [0-100]. Higher =
                more smoothing; the gimbal ramps gradually to each target angle
                rather than jumping immediately. 0 disables filtering entirely.
            smooth_roll : int or None
                Same for roll commands [0-100].
            smooth_yaw : int or None
                Same for yaw commands [0-100].
        """
        to_send = [
            (kw.PARAM_RC_LPF_TILT, smooth_pitch),
            (kw.PARAM_RC_LPF_ROLL, smooth_roll),
            (kw.PARAM_RC_LPF_PAN,  smooth_yaw),
        ]
        for param_id, value in to_send:
            if value is not None:
                self._set_param(param_id, int(value))
                logger.info(f"Set: {param_id:16s} = {value}")

    def set_speed_control(self, speed_pitch=None, speed_roll=None, speed_yaw=None):
        """
        Set the command-speed parameters on the gimbal. These cap the maximum
        angular rate used in rate-control mode. Only parameters that are not
        None are sent.

        Parameters:
        -----------
            speed_pitch : int or None
                Maximum pitch rate [deg/s]. Caps how fast the gimbal moves in
                rate-control mode on the pitch axis.
            speed_roll : int or None
                Maximum roll rate [deg/s].
            speed_yaw : int or None
                Maximum yaw rate [deg/s].
        """
        to_send = [
            (kw.PARAM_RC_SPD_TILT, speed_pitch),
            (kw.PARAM_RC_SPD_ROLL, speed_roll),
            (kw.PARAM_RC_SPD_PAN,  speed_yaw),
        ]
        for param_id, value in to_send:
            if value is not None:
                self._set_param(param_id, int(value))
                logger.info(f"Set: {param_id:16s} = {value}")

    def set_rotation_limits(self, min_tilt=None, max_tilt=None,
                            min_roll=None, max_roll=None,
                            min_pan=None,  max_pan=None):
        """
        Set the software rotation end-stops on the gimbal. Only parameters
        that are not None are sent.

        Parameters:
        -----------
            min_tilt : int or None   Pitch lower end-stop [degrees].
            max_tilt : int or None   Pitch upper end-stop [degrees].
            min_roll : int or None   Roll lower end-stop  [degrees].
            max_roll : int or None   Roll upper end-stop  [degrees].
            min_pan  : int or None   Yaw lower end-stop   [degrees].
            max_pan  : int or None   Yaw upper end-stop   [degrees].
        """
        # exit state flag
        exit_flag = False

        # check logical limits before sending to gimbal
        if min_tilt is not None:
            if min_tilt < params.DEFAULT_RC_LIM_MIN_TILT:
                logger.warning(f"min_tilt {min_tilt} is below the default limit of {params.DEFAULT_RC_LIM_MIN_TILT}, clipping to {params.DEFAULT_RC_LIM_MIN_TILT}")
                min_tilt = params.DEFAULT_RC_LIM_MIN_TILT
            if min_tilt > params.DEFAULT_RC_LIM_MAX_TILT:
                logger.warning(f"min_tilt {min_tilt} is above the default limit of {params.DEFAULT_RC_LIM_MAX_TILT}")
                exit_flag = True
        if max_tilt is not None:
            if max_tilt > params.DEFAULT_RC_LIM_MAX_TILT:
                logger.warning(f"max_tilt {max_tilt} is above the default limit of {params.DEFAULT_RC_LIM_MAX_TILT}, clipping to {params.DEFAULT_RC_LIM_MAX_TILT}")
                max_tilt = params.DEFAULT_RC_LIM_MAX_TILT
            if max_tilt < params.DEFAULT_RC_LIM_MIN_TILT:
                logger.warning(f"max_tilt {max_tilt} is below the default limit of {params.DEFAULT_RC_LIM_MIN_TILT}")
                exit_flag = True
        if min_roll is not None:
            if min_roll < params.DEFAULT_RC_LIM_MIN_ROLL:
                logger.warning(f"min_roll {min_roll} is below the default limit of {params.DEFAULT_RC_LIM_MIN_ROLL}, clipping to {params.DEFAULT_RC_LIM_MIN_ROLL}")
                min_roll = params.DEFAULT_RC_LIM_MIN_ROLL
            if min_roll > params.DEFAULT_RC_LIM_MAX_ROLL:
                logger.warning(f"min_roll {min_roll} is above the default limit of {params.DEFAULT_RC_LIM_MAX_ROLL}")
                exit_flag = True
        if max_roll is not None:
            if max_roll > params.DEFAULT_RC_LIM_MAX_ROLL:
                logger.warning(f"max_roll {max_roll} is above the default limit of {params.DEFAULT_RC_LIM_MAX_ROLL}, clipping to {params.DEFAULT_RC_LIM_MAX_ROLL}")
                max_roll = params.DEFAULT_RC_LIM_MAX_ROLL
            if max_roll < params.DEFAULT_RC_LIM_MIN_ROLL:
                logger.warning(f"max_roll {max_roll} is below the default limit of {params.DEFAULT_RC_LIM_MIN_ROLL}")
                exit_flag = True
        if min_pan is not None:
            if min_pan < params.DEFAULT_RC_LIM_MIN_PAN:
                logger.warning(f"min_pan {min_pan} is below the default limit of {params.DEFAULT_RC_LIM_MIN_PAN}, clipping to {params.DEFAULT_RC_LIM_MIN_PAN}")
                min_pan = params.DEFAULT_RC_LIM_MIN_PAN
            if min_pan > params.DEFAULT_RC_LIM_MAX_PAN:
                logger.warning(f"min_pan {min_pan} is above the default limit of {params.DEFAULT_RC_LIM_MAX_PAN}")
                exit_flag = True
        if max_pan is not None:
            if max_pan > params.DEFAULT_RC_LIM_MAX_PAN:
                logger.warning(f"max_pan {max_pan} is above the default limit of {params.DEFAULT_RC_LIM_MAX_PAN}, clipping to {params.DEFAULT_RC_LIM_MAX_PAN}")
                max_pan = params.DEFAULT_RC_LIM_MAX_PAN
            if max_pan < params.DEFAULT_RC_LIM_MIN_PAN:
                logger.warning(f"max_pan {max_pan} is below the default limit of {params.DEFAULT_RC_LIM_MIN_PAN}")
                exit_flag = True

        if min_tilt is not None and max_tilt is not None and min_tilt >= max_tilt:
            logger.warning(f"min_tilt {min_tilt} cannot be greater or equal to max_tilt {max_tilt}")
            exit_flag = True
        if min_roll is not None and max_roll is not None and min_roll >= max_roll:
            logger.warning(f"min_roll {min_roll} cannot be greater or equal to max_roll {max_roll}")
            exit_flag = True
        if min_pan is not None and max_pan is not None and min_pan >= max_pan:
            logger.warning(f"min_pan {min_pan} cannot be greater or equal to max_pan {max_pan}")
            exit_flag = True

        if exit_flag:
            logger.warning("Exiting set_rotation_limits()")
            return

        to_send = [
            (kw.PARAM_RC_LIM_MIN_TILT, min_tilt),
            (kw.PARAM_RC_LIM_MAX_TILT, max_tilt),
            (kw.PARAM_RC_LIM_MIN_ROLL, min_roll),
            (kw.PARAM_RC_LIM_MAX_ROLL, max_roll),
            (kw.PARAM_RC_LIM_MIN_PAN,  min_pan),
            (kw.PARAM_RC_LIM_MAX_PAN,  max_pan),
        ]
        for param_id, value in to_send:
            if value is not None:
                self._set_param(param_id, int(value))
                logger.info(f"Set: {param_id:20s} = {value}")

    def set_damping(self, tilt=None, roll=None, pan=None):
        """
        Set the motor output damping on the gimbal. Only parameters that are
        not None are sent. Higher values reduce oscillation but slow response.

        Parameters:
        -----------
            tilt : int or None   Pitch axis damping [0-100].
            roll : int or None   Roll axis damping  [0-100].
            pan  : int or None   Yaw axis damping   [0-100].
        """
        to_send = [
            (kw.PARAM_TILT_DAMPING, tilt),
            (kw.PARAM_ROLL_DAMPING, roll),
            (kw.PARAM_PAN_DAMPING,  pan),
        ]
        for param_id, value in to_send:
            if value is not None:
                self._set_param(param_id, int(value))
                logger.info(f"Set: {param_id:20s} = {value}")

    def set_gyro_trust(self, value):
        """
        Set the gyro sensor trust value. Higher values give more weight to the
        gyro in the sensor fusion.

        Parameters:
        -----------
            value : int
                Gyro trust level [0-255].
        """
        self._set_param(kw.PARAM_GYRO_TRUST, int(value))
        logger.info(f"Set: GYRO_TRUST           = {value}")

    def set_home_pan(self, angle):
        """
        Set the home pan/yaw angle.

        Parameters:
        -----------
            angle : int
                Home yaw angle [degrees].
        """
        self._set_param(kw.PARAM_GMB_HOME_PAN, int(angle))
        logger.info(f"Set: GMB_HOME_PAN          = {angle}")

    def set_mapping_angle(self, angle):
        """
        Set the mapping mode tilt angle.

        Parameters:
        -----------
            angle : int
                Tilt angle used in mapping mode [degrees].
        """
        self._set_param(kw.PARAM_MAPPING_ANGLE, int(angle))
        logger.info(f"Set: MAPPING_ANGLE         = {angle}")

    def set_rc_deadzone(self, tilt=None, roll=None, pan=None):
        """
        Set the RC dead-zone per axis. Only parameters that are not None are sent.

        Parameters:
        -----------
            tilt : int or None   Pitch dead-zone [0-100].
            roll : int or None   Roll dead-zone  [0-100].
            pan  : int or None   Yaw dead-zone   [0-100].
        """
        to_send = [
            (kw.PARAM_RC_DZONE_TILT, tilt),
            (kw.PARAM_RC_DZONE_ROLL, roll),
            (kw.PARAM_RC_DZONE_PAN,  pan),
        ]
        for param_id, value in to_send:
            if value is not None:
                self._set_param(param_id, int(value))
                logger.info(f"Set: {param_id:20s} = {value}")

    def set_rc_mode(self, tilt=None, roll=None, pan=None):
        """
        Set the RC control mode per axis. Only parameters that are not None are sent.

        Parameters:
        -----------
            tilt : int or None   0 = speed mode, 1 = angle mode.
            roll : int or None
            pan  : int or None
        """
        to_send = [
            (kw.PARAM_RC_MODE_TILT, tilt),
            (kw.PARAM_RC_MODE_ROLL, roll),
            (kw.PARAM_RC_MODE_PAN,  pan),
        ]
        for param_id, value in to_send:
            if value is not None:
                self._set_param(param_id, int(value))
                logger.info(f"Set: {param_id:20s} = {value}")

    def set_rc_trim(self, tilt=None, roll=None):
        """
        Set the RC trim (home offset) for tilt and/or roll axes.
        Only parameters that are not None are sent.

        Parameters:
        -----------
            tilt : int or None
                Pitch axis home offset [degrees].
            roll : int or None
                Roll axis home offset [degrees].
        """
        to_send = [
            (kw.PARAM_RC_TRIM_TILT, tilt),
            (kw.PARAM_RC_TRIM_ROLL, roll),
        ]
        for param_id, value in to_send:
            if value is not None:
                self._set_param(param_id, int(value))
                logger.info(f"Set: {param_id:20s} = {value}")

    def set_rc_channels(self, stilt=None, span=None, tilt=None,
                        roll=None, pan=None, mode=None):
        """
        Set RC channel assignments. Only parameters that are not None are sent.

        Parameters:
        -----------
            stilt : int or None   Channel for tilt speed axis.
            span  : int or None   Channel for pan speed axis.
            tilt  : int or None   Channel for tilt angle axis.
            roll  : int or None   Channel for roll angle axis.
            pan   : int or None   Channel for pan angle axis.
            mode  : int or None   Channel for mode switch.
        """
        to_send = [
            (kw.PARAM_RC_CHAN_STILT, stilt),
            (kw.PARAM_RC_CHAN_SPAN,  span),
            (kw.PARAM_RC_CHAN_TILT,  tilt),
            (kw.PARAM_RC_CHAN_ROLL,  roll),
            (kw.PARAM_RC_CHAN_PAN,   pan),
            (kw.PARAM_RC_CHAN_MODE,  mode),
        ]
        for param_id, value in to_send:
            if value is not None:
                self._set_param(param_id, int(value))
                logger.info(f"Set: {param_id:20s} = {value}")

    def set_rc_config(self, rc_type=None, reverse_axis=None):
        """
        Set miscellaneous RC configuration. Only parameters that are not None
        are sent.

        Parameters:
        -----------
            rc_type      : int or None   RC input type.
            reverse_axis : int or None   Bitmask of reversed axes.
        """
        to_send = [
            (kw.PARAM_RC_TYPE,         rc_type),
            (kw.PARAM_RC_REVERSE_AXIS, reverse_axis),
        ]
        for param_id, value in to_send:
            if value is not None:
                self._set_param(param_id, int(value))
                logger.info(f"Set: {param_id:20s} = {value}")

    def set_mav_config(self, emit_heartbeat=None, ts_encoder_count=None):
        """
        Set writable MAVLink interface configuration. Only parameters that are
        not None are sent.

        Parameters:
        -----------
            emit_heartbeat   : int or None   1 = emit heartbeat, 0 = suppress.
            ts_encoder_count : int or None   1 = timestamp encoder count messages.
        """
        to_send = [
            (kw.PARAM_MAV_EMIT_HB,  emit_heartbeat),
            (kw.PARAM_MAV_TS_ENCNT, ts_encoder_count),
        ]
        for param_id, value in to_send:
            if value is not None:
                self._set_param(param_id, int(value))
                logger.info(f"Set: {param_id:20s} = {value}")

    def set_message_rates(self, orientation=None, raw_imu=None,
                          encoder_count=None, sys_status=None):
        """
        Set the MAVLink output message rates for each data stream.

        All rates are written via PARAM_SET and persist across reboots.
        GIMBAL_DEVICE_ATTITUDE_STATUS rate is not configurable by the firmware.

        Parameters:
        -----------
            orientation : float or None
                Rate for MOUNT_ORIENTATION (MAV_RATE_ORIEN) [Hz].
            raw_imu : float or None
                Rate for RAW_IMU (MAV_RATE_IMU) [Hz].
            encoder_count : float or None
                Rate for MOUNT_STATUS / encoder count (MAV_RATE_ENCCNT) [Hz].
            sys_status : float or None
                Rate for SYS_STATUS (MAV_RATE_ST) [Hz].
        """
        to_send = [
            (kw.PARAM_MAV_RATE_ORIEN,  orientation),
            (kw.PARAM_MAV_RATE_IMU,    raw_imu),
            (kw.PARAM_MAV_RATE_ENCCNT, encoder_count),
            (kw.PARAM_MAV_RATE_ST,     sys_status),
        ]
        for param_id, value in to_send:
            if value is not None:
                self._set_param(param_id, int(value))
                logger.info(f"Set: {param_id:20s} = {value}")

    def set_mode(self, mode):
        """
        Set gimbal mode by sending a PARAM_EXT_SET message with the appropriate parameters.

        Parameters:
        -----------
            mode : str
                The mode to set the gimbal to. Must be one of "off", "lock", "follow",
                "mapping", or "reset".
        """
        mode_map = {"off": 0, "lock": 1, "follow": 2, "mapping": 3, "reset": 4}
        if mode not in mode_map:
            logger.warning("Invalid mode '%s' requested, keeping current mode. Valid modes are: %s", mode, list(mode_map.keys()))
            return

        # param_id must be exactly 16 bytes (null-padded), per MAVLink PARAM_EXT_SET spec
        param_id = b"GB_MODE\x00\x00\x00\x00\x00\x00\x00\x00\x00"  # 16 bytes

        # param_value for UINT32 must be the 4-byte little-endian integer, padded to 128 bytes
        mode_value = mode_map[mode]
        param_value = struct.pack('<I', mode_value).ljust(128, b'\x00')

        self.master.mav.param_ext_set_send(
            target_system=self.system_id,
            target_component=self.component_id,
            param_id=param_id,
            param_value=param_value,
            param_type=mavutil.mavlink.MAV_PARAM_EXT_TYPE_UINT32
        )

        if self.mode == 'off' and mode != 'off':
            time.sleep(2) # give the gimbal some time to power on before sending further commands
            self.wait_until_stopped()
            # send again the command
            self.master.mav.param_ext_set_send(
                target_system=self.system_id,
                target_component=self.component_id,
                param_id=param_id,
                param_value=param_value,
                param_type=mavutil.mavlink.MAV_PARAM_EXT_TYPE_UINT32
            )

        if mode == 'reset':
            logger.info(f"Resetting gimbal...")
            self.wait_until_stopped()
            self.mode = 'follow'
            self.initialize_goto()
        elif mode == 'off':
            self.mode = 'off'
        elif mode == 'lock':
            self.mode = 'lock'
        elif mode == 'follow':
            self.mode = 'follow'
            self.initialize_goto()
            self.wait_until_stopped()
        elif mode == 'mapping':
            self.mode = 'mapping'
            self.wait_until_stopped()

        logger.info(f"Mode set to '{self.mode}'")

    def set_speed(self, yaw_speed=None, pitch_speed=None, roll_speed=None):
        """
        Set the desired movement speed for each axis in degrees per second, used by subsequent
        goto() calls. If a speed is None that axis is left unchanged. Calling set_speed() with
        no arguments resets all axes to the gimbal's default speed.

        Parameters:
        -----------
            yaw_speed : float or None, optional
                Desired yaw speed in deg/s. None leaves the current value unchanged.
            pitch_speed : float or None, optional
                Desired pitch speed in deg/s. None leaves the current value unchanged.
            roll_speed : float or None, optional
                Desired roll speed in deg/s. None leaves the current value unchanged.
        """
        if yaw_speed is None and pitch_speed is None and roll_speed is None:
            # reset all speeds to gimbal default
            self.yaw_speed = None
            self.pitch_speed = None
            self.roll_speed = None
            logger.info("Speed set to default")
        else:
            if yaw_speed is not None:
                self.yaw_speed = float(yaw_speed)
            if pitch_speed is not None:
                self.pitch_speed = float(pitch_speed)
            if roll_speed is not None:
                self.roll_speed = float(roll_speed)
            logger.info(f"Speed set to (y, p, r) = {self.yaw_speed:7.2f}, {self.pitch_speed:7.2f}, {self.roll_speed:7.2f} deg/s")


    def render_panel(self) -> Panel:
        # read current gimbal data for display
        gimbal_data = self.telemetry_state.get()

        if not gimbal_data:
            return Panel("[dim]waiting for telemetry frames...[/dim]",
                         title="Telemetry", border_style="blue", title_align="left")
        
        if self.simulator:
            title = f"[bold]Gremsy T7 - Lab test simulator[/bold]"
        else:
            title = f"[bold]Gremsy T7 - v{self.fw_version}[/bold]"

        def kv_table() -> Table:
            t = Table.grid(expand=True, padding=(0, -5))
            t.add_column(justify="left", style="bold yellow", no_wrap=False)
            t.add_column(justify="left", no_wrap=True)
            return t
        
        c1 = kv_table()
        c1.add_row("Time (ms)", f"{gimbal_data.get('time_boot_ms', 0)}")
        c1.add_row("Time (us)", f"{gimbal_data.get('time_usec', 0)}")
        c1.add_row("[bold blue] ATTITUDE (°)", "")
        c1.add_row("Roll",  f"{gimbal_data.get('roll', 0):+3.3f}")
        c1.add_row("Pitch", f"{gimbal_data.get('pitch', 0):+3.3f}")
        c1.add_row("Yaw",   f"{gimbal_data.get('yaw', 0):+3.3f}")
        c1.add_row("Yaw abs", f"{gimbal_data.get('yaw_absolute', 0):+3.3f}")
        c1.add_row("[bold blue] ANGULAR VEL (?)", "")
        c1.add_row("x", f"{gimbal_data.get('angular_velocity_x', 0):+3.3f}")
        c1.add_row("y", f"{gimbal_data.get('angular_velocity_y', 0):+3.3f}")
        c1.add_row("z", f"{gimbal_data.get('angular_velocity_z', 0):+3.3f}")
        c1.add_row("[bold blue] RAW IMU", "")
        c1.add_row("x acc", f"{gimbal_data.get('xacc', 0):+3.3f}")
        c1.add_row("y acc", f"{gimbal_data.get('yacc', 0):+3.3f}")
        c1.add_row("z acc", f"{gimbal_data.get('zacc', 0):+3.3f}")
        c1.add_row("x gyro", f"{gimbal_data.get('xgyro', 0):+3.3f}")
        c1.add_row("y gyro", f"{gimbal_data.get('ygyro', 0):+3.3f}")
        c1.add_row("z gyro", f"{gimbal_data.get('zgyro', 0):+3.3f}")
        c1.add_row("x mag", f"{gimbal_data.get('xmag', 0):+3.3f}")
        c1.add_row("y mag", f"{gimbal_data.get('ymag', 0):+3.3f}")
        c1.add_row("z mag", f"{gimbal_data.get('zmag', 0):+3.3f}")
        c1.add_row("Temp", f"{gimbal_data.get('temperature', 0):+3.3f}")
        c1.add_row("[bold blue] SYSTEM", "")
        c1.add_row("Load (%)", f"{gimbal_data.get('load', 0):+3.3f}")
        c1.add_row("Battery (V)", f"{gimbal_data.get('voltage_battery', 0):.1f}")
        c1.add_row("Current (A)", f"{gimbal_data.get('current_battery', 0):.1f}")
        c1.add_row("Battery (%)", f"{gimbal_data.get('battery_remaining', 0):.1f}")

        return Panel(c1, title=title, border_style="blue", title_align="left")

"""        
            q = attitude.get('q') or [None, None, None, None]
            data['q0'].append(q[0])
            data['q1'].append(q[1])
            data['q2'].append(q[2])
            data['q3'].append(q[3])
            data['delta_yaw'].append(attitude.get('delta_yaw'))
            data['delta_yaw_velocity'].append(attitude.get('delta_yaw_velocity'))

            data['pointing_a'].append(mount_status.get('pointing_a'))
            data['pointing_b'].append(mount_status.get('pointing_b'))
            data['pointing_c'].append(mount_status.get('pointing_c'))
            data['mount_mode'].append(mount_status.get('mount_mode'))

            data['drop_rate_comm'].append(sys_status.get('drop_rate_comm'))
            data['errors_comm'].append(sys_status.get('errors_comm'))
            data['errors_count1'].append(sys_status.get('errors_count1'))
            data['errors_count2'].append(sys_status.get('errors_count2'))
            data['errors_count3'].append(sys_status.get('errors_count3'))
            data['errors_count4'].append(sys_status.get('errors_count4'))
"""