import logging
import os
import queue
import random
import shutil
import signal
import socket
import subprocess
import threading
import time

import lager.UAVs.DJI_M400.parameters as params
import lager.UAVs.DJI_M400.stream as stream
import lager.UAVs.DJI_M400.utils as utils
import lager.UAVs.DJI_M400.telemetry_state as telemetry_state

logger = logging.getLogger(__name__)


class Drone:
    """
    DJI Matrice 400 telemetry through the E-Port. Two ways to get the psdk_telemetry stream (Payload-SDK):
      - socket_path set: read it from psdk-bridge.service (psdk_telemetry --service), which owns the PSDK
        connection from boot and also runs the RC widgets,
      - otherwise: run psdk_telemetry as a child process and read its stdout.
    """
    def __init__(self, binary: str = params.DEFAULT_BINARY,
                 device: str = params.DEFAULT_DEVICE,
                 fast_hz: int = params.FAST_HZ,
                 slow_hz: int = params.SLOW_HZ,
                 imu_hz: int = params.IMU_HZ,
                 status_hz: int = params.STATUS_HZ,
                 stats_s: int = params.STATS_S,
                 connect_timeout: float = params.CONNECT_TIMEOUT,
                 simulator: bool = False,
                 output_dir: str = "",
                 socket_path: str = None):
        self.binary = binary if binary not in [None, ""] else params.DEFAULT_BINARY
        self.device = device if device not in [None, ""] else params.DEFAULT_DEVICE
        self.fast_hz = fast_hz if fast_hz is not None else params.FAST_HZ
        self.slow_hz = slow_hz if slow_hz is not None else params.SLOW_HZ
        self.imu_hz = imu_hz if imu_hz is not None else params.IMU_HZ
        self.status_hz = status_hz if status_hz is not None else params.STATUS_HZ
        self.stats_s = stats_s if stats_s is not None else params.STATS_S
        self.connect_timeout = connect_timeout if connect_timeout is not None else params.CONNECT_TIMEOUT
        self.simulator = simulator
        self.output_dir = output_dir
        self.socket_path = socket_path or None
        self.sock = None
        self._reader = None

        self.proc = None
        self.link_state = "idle"            # last STATUS state of psdk_telemetry, or "exited"
        self.version_name = None            # text of the CONNECTED status ("connected: aircraft type 103 (Matrice 400) ...")
        self.fw_version = 0                 # kept for compatibility with the M600 Drone
        self.topics = {}                    # topic id -> {"name", "hz", "size", "subscribed"}, from TOPIC_TABLE
        self.dropped = 0                    # records dropped inside psdk_telemetry, from STATS
        self.bad_crc = 0

        self.stop_event = threading.Event()
        self.subscribed_event = threading.Event()
        self.log_queue = queue.Queue(maxsize=params.LOG_QUEUE_MAXSIZE)
        self.telemetry_state = telemetry_state.TelemetryState()
        self._threads = []

        if self.simulator:
            logger.info("Lab test simulator mode enabled - psdk_telemetry will not be started.")
            self.version_name = "Lab test simulator"

    # public interface
    def connect(self):
        """Quick checks only; raise if psdk_telemetry cannot be started."""
        if self.simulator:
            return
        if self.socket_path:
            if not os.path.exists(self.socket_path):
                raise RuntimeError(f"psdk-bridge service not connected to the aircraft yet ({self.socket_path} missing; "
                                   "systemctl status psdk-bridge)")
            return
        path = shutil.which(self.binary) if os.sep not in self.binary else self.binary
        if not path or not os.access(path, os.X_OK):
            raise RuntimeError(f"{self.binary} not found or not executable (build it with install_porter.sh)")
        self.binary = path
        if not os.path.exists(self.device):
            raise RuntimeError(f"{self.device} missing: USB gadget not active (systemctl status dji-vcom-gadget)")

    def start_telemetry(self):
        """Start psdk_telemetry and the threads; return when the topics are subscribed, raise otherwise."""
        self.stop_event.clear()
        self.subscribed_event.clear()
        self._threads = []

        if self.simulator:
            logger.info("Starting telemetry simulation...")
            self._start_thread(self._read_loop_sim)
            self._start_thread(self._write_loop)
            return

        if self.socket_path:
            self.sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
            try:
                self.sock.connect(self.socket_path)
            except OSError as e:
                self.sock.close()
                self.sock = None
                raise RuntimeError(f"cannot connect to psdk-bridge ({self.socket_path}): {e}")
            logger.info(f"Reading UAV telemetry from psdk-bridge ({self.socket_path})")
            self.link_state = "starting"
            self._reader = self._start_thread(self._read_loop)
            self._start_thread(self._write_loop)
            # the service sends its CONNECTED/TOPIC_TABLE/SUBSCRIBED records first
            if not self.subscribed_event.wait(params.SERVICE_GREETING_TIMEOUT):
                self.stop_telemetry()
                raise RuntimeError("psdk-bridge didn't send its topic table: service not subscribed yet")
            logger.info(f"UAV telemetry running: {self.version_name}")
            return

        cmd = [self.binary, "--imu", str(self.imu_hz), "--fast", str(self.fast_hz), "--status", str(self.status_hz),
               "--slow", str(self.slow_hz), "--stats", str(self.stats_s)]
        logger.info(f"Starting {' '.join(cmd)}")
        self.proc = subprocess.Popen(cmd, stdin=subprocess.DEVNULL, stdout=subprocess.PIPE,
                                     stderr=subprocess.PIPE, bufsize=0)
        self.link_state = "starting"
        self._reader = self._start_thread(self._read_loop)
        self._start_thread(self._stderr_loop)
        self._start_thread(self._write_loop)

        # wait for SUBSCRIBED, but give up at once if the program exits
        deadline = time.monotonic() + self.connect_timeout
        while not self.subscribed_event.wait(0.2):
            code = self.proc.poll()
            if code is not None or time.monotonic() > deadline:
                reason = (f"exited with code {code} ({params.EXIT_CODES.get(code, 'unknown')})"
                          if code is not None else f"not subscribed after {self.connect_timeout:.0f} s")
                self.stop_telemetry()
                raise RuntimeError(f"psdk_telemetry {reason}")
        logger.info(f"UAV telemetry running: {self.version_name}")

    def stop_telemetry(self):
        """Stop psdk_telemetry cleanly (SIGINT), then the threads."""
        logger.info("Stopping telemetry reception...")
        self.stop_event.set()
        self._stop_process()
        self._close_socket()
        for t in self._threads:
            t.join(timeout=3)
        logger.info("Telemetry reception stopped.")

    def disconnect(self):
        """No serial port to close: make sure the process is gone, or the service socket closed."""
        self._stop_process()
        self._close_socket()

    def is_alive(self) -> bool:
        """False once psdk_telemetry has exited (drone off, cable unplugged, crash)."""
        if self.simulator:
            return not self.stop_event.is_set()
        if self.socket_path:
            return self._reader is not None and self._reader.is_alive()
        return self.proc is not None and self.proc.poll() is None

    # threads
    def _start_thread(self, target):
        t = threading.Thread(target=target, daemon=True, name=f"M400{target.__name__}")
        t.start()
        self._threads.append(t)
        return t

    def _read_loop(self):
        """psdk_telemetry stdout -> raw copy to the log queue + decoded values to the telemetry state."""
        parser = stream.StreamParser()
        sock = self.sock                                # _close_socket() may clear self.sock meanwhile
        if sock is not None:
            read = lambda: sock.recv(params.READ_SIZE)
        else:
            fd = self.proc.stdout.fileno()
            read = lambda: os.read(fd, params.READ_SIZE)   # returns what is available, unlike proc.stdout.read(n)
        try:
            while True:
                chunk = read()
                if not chunk:
                    break                               # EOF: the process has exited, or the service closed
                self._queue_raw(chunk)
                for record in parser.feed(chunk):
                    self._handle_record(*record)
                self.bad_crc = parser.bad_crc
        except Exception as e:
            if not self.stop_event.is_set():
                logger.error(f"Error in M400 read loop: {e}")
        if sock is not None or self.proc is None:
            self.link_state = "exited"
            if not self.stop_event.is_set():
                logger.error("psdk-bridge closed the telemetry stream (service restarting or link to the aircraft lost)")
            return
        code = self.proc.wait()
        self.link_state = "exited"
        if not self.stop_event.is_set():
            logger.error(f"psdk_telemetry exited with code {code} ({params.EXIT_CODES.get(code, 'unknown')})")
        else:
            logger.info(f"psdk_telemetry exited with code {code}")

    def _handle_record(self, rtype, topic, host_ns, ac_ms, ac_us, payload):
        if rtype == params.RECORD_DATA:
            info = self.topics.get(topic)
            if info is None:
                return
            name = info["name"]
            values = utils.decode(name, payload)
            if not values:
                return
            values["t_host"] = host_ns / 1e9
            values["t_aircraft_ms"] = ac_ms
            self.telemetry_state.update(values)
            if name in ("battery1", "battery2"):
                self.telemetry_state.update(utils.battery_summary(self.telemetry_state.get()))

        elif rtype == params.RECORD_STATUS:
            state, text = stream.parse_status(payload)
            self.link_state = params.STATE_NAMES.get(state, str(state))
            if state == params.STATE_ERROR:
                logger.error(f"psdk_telemetry: {text}")
            elif state == params.STATE_WARNING:
                logger.warning(f"psdk_telemetry: {text}")
            else:
                logger.info(f"psdk_telemetry: {text}")
            if state == params.STATE_CONNECTED:
                self.version_name = text
            elif state == params.STATE_SUBSCRIBED:
                self.subscribed_event.set()

        elif rtype == params.RECORD_TOPIC_TABLE:
            self.topics = stream.parse_topic_table(payload)
            missing = [t["name"] for t in self.topics.values() if not t["subscribed"]]
            if missing:
                logger.warning(f"UAV topics not subscribed: {', '.join(missing)}")

        elif rtype == params.RECORD_STATS:
            dropped, received = stream.parse_stats(payload)
            if dropped > self.dropped:
                logger.warning(f"psdk_telemetry dropped {dropped - self.dropped} records (reader too slow?)")
            self.dropped = dropped
            logger.debug("UAV topics received: " + " ".join(
                f"{self.topics.get(t, {}).get('name', t)}={n}" for t, n in received.items()))

    def _stderr_loop(self):
        """psdk_telemetry stderr (SDK log) -> psdk_telemetry.log; errors also to the lager log.
        Must always run: a full stderr pipe would block psdk_telemetry."""
        path = os.path.join(self.output_dir, params.PSDK_LOG_FILENAME)
        with open(path, "a", buffering=1) as f:
            for raw in iter(self.proc.stderr.readline, b""):
                line = raw.decode("utf-8", "replace").rstrip()
                f.write(line + "\n")
                if "[Error]" in line:
                    logger.warning(f"PSDK: {line}")

    def _queue_raw(self, chunk):
        try:
            self.log_queue.put_nowait(chunk)
        except queue.Full:
            logger.warning(f"UAV log queue full ({params.LOG_QUEUE_MAXSIZE} chunks); dropping the oldest")
            try:
                self.log_queue.get_nowait()
            except queue.Empty:
                pass
            self.log_queue.put_nowait(chunk)

    def _write_loop(self):
        """Raw stream -> uav_telemetry.bin. The stream is already framed (timestamps, CRC): no extra header."""
        path = os.path.join(self.output_dir, params.TELEMETRY_FILENAME)
        logger.info(f"Logging raw UAV telemetry to {path}")
        buf, count = [], 0
        with open(path, "ab") as f:
            while not self.stop_event.is_set() or not self.log_queue.empty():
                try:
                    buf.append(self.log_queue.get(timeout=0.1))
                except queue.Empty:
                    continue
                count += 1
                if len(buf) >= params.BATCH_SIZE:
                    f.write(b"".join(buf))
                    buf.clear()
                if count % params.FLUSH_EVERY == 0:
                    f.flush()
            if buf:
                f.write(b"".join(buf))

    def _close_socket(self):
        s, self.sock = self.sock, None
        if s is None:
            return
        try:
            s.shutdown(socket.SHUT_RDWR)                # wakes up the reader
        except OSError:
            pass
        s.close()

    def _stop_process(self):
        p = self.proc
        if p is None or p.poll() is not None:
            return
        try:
            p.send_signal(signal.SIGINT)                # clean stop: unsubscribe, final STATS, exit 0
            p.wait(timeout=params.STOP_TIMEOUT)
        except subprocess.TimeoutExpired:
            logger.warning("psdk_telemetry did not stop on SIGINT; terminating it")
            p.terminate()
            try:
                p.wait(timeout=2)
            except subprocess.TimeoutExpired:
                p.kill()
                p.wait()

    def _read_loop_sim(self):
        """Fake values with the same keys as the real decoder."""
        while not self.stop_event.is_set():
            values = {
                "roll": random.uniform(-10, 10), "pitch": random.uniform(-10, 10), "yaw": random.uniform(-180, 180),
                "q": [1, 0, 0, 0], "vx": random.uniform(-10, 10), "vy": random.uniform(-10, 10),
                "vz": random.uniform(-2, 2), "vel_info": 1,
                "gps_lat_deg": random.uniform(41, 42), "gps_lon_deg": random.uniform(12, 13),
                "gps_hfsl_m": random.uniform(0, 100), "hdop": 0.8, "pdop": 1.2, "gps_fix": 3, "gps_nsv": 20,
                "rtk_lat_deg": random.uniform(41, 42), "rtk_lon_deg": random.uniform(12, 13),
                "rtk_hfsl_m": random.uniform(0, 100), "rtk_yaw_deg": random.uniform(-180, 180),
                "rtk_pos_health": 50, "rtk_yaw_health": 50, "hgt_m": random.uniform(0, 100),
                "flight_status": 2, "display_mode": 6, "flight_error": 0,
                "bat1_percentage": 80, "bat1_voltage_mv": 51000, "bat1_current_ma": -15000,
                "bat1_remaining_mah": 7000, "bat1_temp_c": 30.0,
                "bat2_percentage": 79, "bat2_voltage_mv": 50900, "bat2_current_ma": -15000,
                "bat2_remaining_mah": 6900, "bat2_temp_c": 31.0,
                "esc_max_temp_c": random.uniform(30, 60),
                "t_host": time.time(),
            }
            values.update(utils.battery_summary(values))
            self.telemetry_state.update(values)
            time.sleep(1 / self.fast_hz)
