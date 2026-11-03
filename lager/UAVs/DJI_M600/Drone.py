import logging
import time
import struct
import threading
import queue
import random
import os

import lager.UAVs.DJI_M600.parameters as params
import lager.UAVs.DJI_M600.connection as connection
import lager.UAVs.DJI_M600.utils as utils
import lager.UAVs.DJI_M600.telemetry_state as telemetry_state


# use the Controller.py logging configuration
logger = logging.getLogger(__name__)

class Drone:
    def __init__(self, device: str= params.DEFAULT_SERIAL_PORT, 
                 baudrate: int = params.DEFAULT_BAUDRATE,
                 timeout: float = params.SERIAL_TIMEOUT,
                 telemetry_frequency: int = params.TELEMETRY_FREQ,
                 simulator: bool = False,
                 output_dir: str = ""):

        self.device = device if device not in [None, ""] else params.DEFAULT_SERIAL_PORT
        self.baudrate = baudrate if baudrate is not None else params.DEFAULT_BAUDRATE
        self.timeout = timeout if timeout is not None else params.SERIAL_TIMEOUT
        self.telemetry_frequency = telemetry_frequency if telemetry_frequency is not None else params.TELEMETRY_FREQ
        
        # output directory for telemetry log
        self.output_dir = output_dir
        
        # serial connection 
        self.connection = connection.DroneConnection(self.device, self.baudrate, self.timeout)

        # firmware version info
        self.version_name = None
        self.fw_version = 0
        
        # flag to simulate mock telemetry data instead of reading from serial port
        self.simulator = simulator
        if self.simulator:
            logger.info("Lab test simulator mode enabled - no serial connection will be made.")
            self.version_name = "Lab test simulator"

        self.stop_event = threading.Event()
        self.log_queue  = queue.Queue(maxsize=params.LOG_QUEUE_MAXSIZE)

        # telemetry state
        self.telemetry_state = telemetry_state.TelemetryState()

    def _read_loop(self):
        """
        Background thread to read incoming telemetry frames from the serial port,
        decode them, and add them to a queue for writing to disk.
        """
        # Bytes left over from a failed header parse. On a CRC16 mismatch we
        # advance by exactly 1 byte instead of discarding the whole 12-byte
        # read, so that a noise byte in the stream doesn't cause a long
        # resync gap.
        residual = b''
        try:
            while not self.stop_event.is_set():
                # check if serial port is open before attempting to read
                if not self.connection.serial or not self.connection.serial.is_open:
                    logger.error("Serial port not open. Exiting read loop.")
                    break

                # fill header buffer, reusing leftover bytes from a previous mismatch
                need = params.HEADER_LEN - len(residual)
                if need > 0:
                    chunk = self.connection.serial.read(need)
                    if len(chunk) < need:
                        residual += chunk
                        logger.debug(f"Incomplete header received (got {len(residual)} bytes), waiting for more data...")
                        continue
                    header = residual + chunk
                else:
                    header = residual[:params.HEADER_LEN]
                residual = b''

                # validate CRC16 of the header (first 10 bytes, CRC stored at bytes 10-11)
                stored_crc16 = struct.unpack_from('<H', header, 10)[0]
                computed_crc16 = utils.crc16(header[:10])
                if stored_crc16 != computed_crc16:
                    # advance by 1 byte — keep header[1:] as the start of the next candidate
                    logger.debug(f"Header CRC16 mismatch: stored=0x{stored_crc16:04X}, computed=0x{computed_crc16:04X}. Advancing 1 byte.")
                    residual = header[1:]
                    continue

                # parse total frame length
                w0 = struct.unpack_from('<I', header, 0)[0]
                total_len = (w0 >> 8) & 0x3FF

                if total_len < params.HEADER_LEN + params.CRC32_LEN:
                    logger.debug(f"Invalid frame length received: {total_len}")
                    residual = header[1:]
                    continue

                # read the rest of the frame based on the total length
                rest = self.connection.serial.read(total_len - params.HEADER_LEN)
                if len(rest) < (total_len - params.HEADER_LEN):
                    logger.debug(f"Incomplete frame received (got {len(rest)} bytes), waiting for more data...")
                    continue

                # complete frame
                timestamp = time.time()
                frame = header + rest

                # add validated frames and their timestamp to the queue for writing;
                # drop the oldest frame with a warning if the writer thread is behind
                try:
                    self.log_queue.put_nowait((timestamp, frame))
                except queue.Full:
                    logger.warning(f"Log queue full ({params.LOG_QUEUE_MAXSIZE} frames); dropping oldest frame")
                    try:
                        self.log_queue.get_nowait()
                    except queue.Empty:
                        pass
                    self.log_queue.put_nowait((timestamp, frame))

                # decode broadcast frames and update shared display state
                f = utils.parse_frame(frame)
                if f:
                    if (not f['is_ack'] and f['crc32_ok']
                            and f['cmd_set'] == params.CMD_SET_BROADCAST
                            and f['cmd_id'] == params.CMD_ID_BROADCAST):
                        bd = utils.decode_broadcast(f['payload'])

                        # update telemetry state
                        self.telemetry_state.update(bd)

        except Exception as e:
            if not self.stop_event.is_set():
                logger.error(f"Error in read loop: {e}")
            self.stop_event.set()
            self.connection.disconnect()

    def _read_loop_sim(self):
        
        t0 = time.time()
        tick = 0
        while not self.stop_event.is_set():
            # Simulate a telemetry frame (replace with your actual frame structure)
            fake_telem = {
                't_ticks': tick,
                'flag': 0x0FFF,
                'roll': random.uniform(-10, 10),
                'pitch': random.uniform(-10, 10),
                'yaw': random.uniform(-180, 180),
                'q': [1, 0, 0, 0],
                'qmag': 1.0,
                'ax': random.uniform(-1, 1),
                'ay': random.uniform(-1, 1),
                'az': random.uniform(-1, 1),
                'vx': random.uniform(-10, 10),
                'vy': random.uniform(-10, 10),
                'vz': random.uniform(-10, 10),
                'vel_info': 0,
                'gx': random.uniform(-1, 1),
                'gy': random.uniform(-1, 1),
                'gz': random.uniform(-1, 1),
                'lat_deg': random.uniform(41, 42),
                'lon_deg': random.uniform(12, 13),
                'alt_m': random.uniform(0, 100),
                'hgt_m': random.uniform(0, 100),
                'gps_health': 5,
                'gps_lat_deg': random.uniform(41, 42),
                'gps_lon_deg': random.uniform(12, 13),
                'gps_hfsl_m': random.uniform(0, 100),
                'gps_vn_cms': random.uniform(-100, 100),
                'gps_ve_cms': random.uniform(-100, 100),
                'gps_vd_cms': random.uniform(-100, 100),
                'gps_nsv': 12,
                'hdop': 0.8,
                'pdop': 1.2,
                'rtk_lat_deg': random.uniform(41, 42),
                'rtk_lon_deg': random.uniform(12, 13),
                'rtk_hfsl_m': random.uniform(0, 100),
                'rtk_yaw_deg': random.uniform(-180, 180),
                'rtk_pos_health': 1,
                'rtk_yaw_health': 1,
                'mx': random.randint(-1000, 1000),
                'my': random.randint(-1000, 1000),
                'mz': random.randint(-1000, 1000),
                'rc_roll': 0,
                'rc_pitch': 0,
                'rc_yaw': 0,
                'rc_thr': 0,
                'rc_mode': 0,
                'rc_gear': 0,
                'gim_roll': 0,
                'gim_pitch': 0,
                'gim_yaw': 0,
                'gim_flags': 0,
            }
            fake_status = {
                'flight_status': 2,
                'display_mode': 6,
                'landing_gear': 1,
                'flight_error': 0,
            }
            fake_battery = {
                'bat_capacity_mah': 15000,
                'bat_percentage': random.uniform(0, 100),
                'bat_voltage_mv': random.uniform(40000, 51000),
                'bat_current_ma': random.uniform(-5000, 0),
                'sdk_control_mode': 0,
                'sdk_device_bits': 0x00,
            }

            # update telemetry state
            self.telemetry_state.update(fake_telem)
            self.telemetry_state.update(fake_status)
            self.telemetry_state.update(fake_battery)

            # Simulate frame rate (e.g., 50 Hz)
            time.sleep(1 / self.telemetry_frequency)
            tick += 1

    def _write_loop(self):
        """
        Background thread to write incoming telemetry frames to disk in batches.
        """
        # create data file
        data_filename = params.TELEMETRY_FILENAME
        data_filename = os.path.join(self.output_dir, data_filename)
        logger.info(f"Logging raw telemetry data to {data_filename}")

        write_buf = []
        frame_count = 0

        with open(data_filename, "wb") as f:
            # drains the log_queue and writes to disk in batches until stop_event is set and queue is empty
            while not self.stop_event.is_set() or not self.log_queue.empty():
                try:
                    timestamp, frame = self.log_queue.get(timeout=0.1)

                    write_buf.append(struct.pack('<dI', timestamp, len(frame)))  # 8 bytes for timestamp + 4 bytes for frame length
                    write_buf.append(frame)
                    
                    frame_count += 1

                    if len(write_buf) >= params.BATCH_SIZE:
                        f.write(b''.join(write_buf))
                        write_buf.clear()

                    if frame_count % params.FLUSH_EVERY == 0:
                        f.flush()
                        logger.debug(f"Flushed {frame_count} frames to disk")
            
                except Exception as e:
                    if isinstance(e, queue.Empty):
                        continue
                    else:
                        logger.error(f"Error in write loop: {e}")
                        self.stop_event.set()
                        self.disconnect()
                        break

            # flush any remaining frames
            if write_buf:
                f.write(b''.join(write_buf))
                f.flush()
                logger.debug(f"Flushed final {len(write_buf)} frames to disk")

    def start_telemetry(self):
        """
        Start the telemetry reception by launching background threads for reading and
        writing telemetry frames.
        """
        if self.simulator:
            logger.info("Starting telemetry simulation...")
            target_read_loop = self._read_loop_sim
        else:
            logger.info("Starting telemetry reception...")
            target_read_loop = self._read_loop

        # start a background thread to read incoming telemetry frames
        self.telemetry_reader_thread = threading.Thread(target=target_read_loop, daemon=True)
        self.telemetry_reader_thread.start()
        logger.debug("Telemetry reader thread started.")

        # start a background thread to write incoming telemetry frames to disk
        self.telemetry_writer_thread = threading.Thread(target=self._write_loop, daemon=True)
        self.telemetry_writer_thread.start()
        logger.debug("Telemetry writer thread started.")


    def stop_telemetry(self):
        """
        Stop the telemetry reception by signaling background threads to exit and
        waiting for them to finish.
        """
        logger.info("Stopping telemetry reception...")
        self.stop_event.set()
        self.telemetry_reader_thread.join(timeout=3)
        self.telemetry_writer_thread.join(timeout=3)
        logger.info("Telemetry reception stopped.")

    def connect(self):
        """
        Connect to the serial port and prepare for telemetry reception.
        """
        # is simulator is enabled, skip actual serial connection and return immediately
        if self.simulator:
            logger.info("Lab test simulator mode enabled - skipping serial connection.")
            return
        
        # connect to the serial port
        self.connection.connect()
        time.sleep(0.1)

        # read version info to confirm connection
        self.get_version()

    def disconnect(self):
        """
        Disconnect from the serial port and clean up resources.
        """
        # is simulator is enabled, skip actual serial connection and return immediately
        if self.simulator:
            logger.info("Lab test simulator mode enabled - no serial connection to close.")
            return
        
        # disconnect from the serial port
        self.connection.disconnect()

    def get_version(self, timeout: float = params.ACK_TIMEOUT, retries: int = 5):
        """
        Perform a handshake by sending a GET_VERSION command and waiting for an ACK.
        If successful, this confirms that the link is alive and active telemetry can proceed.
        If no ACK is received after the specified number of retries, it assumes passive telemetry.
        """
        if self.simulator:
            logger.info("Lab test simulator mode enabled - skipping handshake.")
            self.version_name = "Lab Test Simulator"
            self.fw_version = 0
            return
        
        # try handshake: send a GET_VERSION command and wait for ACK
        # if we get an ACK, the link is alive and we can proceed to parse 
        # incoming telemetry frames
        for attempt in range(1, retries+1):
            # drain stale bytes
            stale = self.connection.serial.read(params.SERIAL_READ_SIZE)
            if stale:
                logger.info(f"Drained {len(stale)} stale bytes")

            ver_frame = self.build_frame(params.CMD_SET_GENERAL, params.CMD_ID_VERSION, b'\x00', seq=0, session=2)
            logger.info(f"Handshake attempt {attempt}/{retries}")
            logger.debug(f"TX: {ver_frame.hex(' ')}")
            self.connection.serial.write(ver_frame)
            self.connection.serial.flush()

            # wait for ACK
            ack, _ = self.wait_for_ack(expected_seq=0, timeout=timeout)
            if ack is None:
                if attempt < retries:
                    logger.warning(f"No ACK received for GET_VERSION command, retrying...") 
                else:
                    logger.warning("No ACK received for GET_VERSION command. Assuming passive telemetry.")
                    return
            
            else:
                payload = ack.get('payload', None)
                if payload is None:
                    if attempt < retries:
                        logger.warning(f"No payload in ACK for GET_VERSION command, retrying...")
                    else:
                        logger.error("No payload in ACK for GET_VERSION command. Assuming passive telemetry.")
                        return

                if len(payload) < 2:
                    if attempt < retries:
                        logger.warning(f"ACK payload too short for GET_VERSION command, retrying...")
                    else:
                        logger.error("ACK payload too short for GET_VERSION command. Assuming passive telemetry.")
                        return
                
                if ack.get('crc32_ok'):
                    logger.info("Handshake successful. Active telemetry mode enabled.")
                    break
                else:
                    if attempt < retries:
                        logger.warning(f"ACK received but CRC32 check failed for GET_VERSION command, retrying...")
                    else:
                        logger.warning("ACK received but CRC32 check failed.")
                        return

            time.sleep(0.5)
        
        logger.debug(f"RX: {payload.hex(' ')}")
        
        version_ack = struct.unpack_from('<H', payload, 0)[0]
        logger.info(f"Version_ack = 0x{version_ack:04X}")

        # skip version_ack (2 bytes) + null-terminated crc_id string
        idx = 2
        while idx < len(payload) and payload[idx] != 0:
            idx += 1
        idx += 1  # skip the NUL

        # version_name starts at idx - read full null-terminated string (no 32-byte cap).
        # The OSDK parseDroneVersionInfo reads directly from the raw buffer so it sees
        # the full string; only its struct copy is truncated at 32 bytes.
        name_end = payload.find(b'\x00', idx)
        if name_end == -1:
            name_end = len(payload)
        self.version_name = payload[idx:name_end].decode('ascii', errors='replace')
        logger.info(f"Version_name: {self.version_name!r}")

        # Parse fw_version from "... HW-fw1.fw2.fw3.fw4"
        try:
            # find the last '-' which separates HW from FW
            dash_pos = self.version_name.rfind('-')
            fw_str_raw = self.version_name[dash_pos+1:]
            parts = [int(x) for x in fw_str_raw.split('.')]
            while len(parts) < 4:
                parts.append(0)
            self.fw_version = (parts[0]<<24)|(parts[1]<<16)|(parts[2]<<8)|parts[3]
            fw_str = '.'.join(str(p) for p in parts[:4])
            logger.info(f"Firmware version: {fw_str}  (0x{self.fw_version:08X})")
        except Exception as e:
            logger.warning(f"Could not parse fw_version from {self.version_name!r}: {e}")
            logger.debug(f"Raw payload: {payload.hex(' ')}")
            # Use 0 as fallback
            self.fw_version = 0


    def build_frame(self, cmd_set: int, cmd_id: int, payload: bytes,
                    seq: int, session: int = 2) -> bytes:
        """
        Build a complete DJI OSDK frame (header + body + CRC32).

        Header (12 bytes):
        W0[7:0]   = SOF (0xAA)
        W0[17:8]  = total frame length
        W0[23:18] = version = 0
        W0[28:24] = session_id
        W0[29]    = is_ack = 0
        W1        = 0  (padding=0, enc=0, reserved1=0)
        W2[15:0]  = seq
        W2[31:16] = CRC16 over header bytes [0:10]
        Body: cmd_set(1) + cmd_id(1) + payload
        Tail: CRC32 over entire frame except last 4 bytes

        Parameters:
        -----------
        cmd_set: int
            The command set identifier (1 byte).
        cmd_id: int
            The command identifier (1 byte).
        payload: bytes
            The payload data to be included in the frame.
        seq: int
            The sequence number for the frame (16 bits).
        session: int, optional
            The session identifier (default is 2).

        Returns:
        --------
        bytes
            The complete DJI OSDK frame as a bytes object.
        """
        body      = struct.pack('BB', cmd_set, cmd_id) + payload
        total_len = params.HEADER_LEN + len(body) + params.CRC32_LEN
        w0        = params.SOF | (total_len << 8) | (session << 24)
        pre_crc   = struct.pack('<IIH', w0, 0, seq & 0xFFFF)   # 10 bytes
        header    = pre_crc + struct.pack('<H', utils.crc16(pre_crc))
        frame     = header + body
        return frame + struct.pack('<I', utils.crc32(frame))


    def wait_for_ack(self, expected_seq: int, timeout: float = params.ACK_TIMEOUT) -> tuple:
        """
        Read until ACK with seq==expected_seq arrives or timeout.

        Returns (ack_dict | None, raw_bytearray).
        Prints every raw chunk and every parsed frame for full visibility.

        Parameters:
        -----------
            ser: serial.Serial
                The serial port object to read from.
            expected_seq: int
                The expected sequence number for the ACK frame.
            timeout: float, optional
                The maximum time to wait for the ACK frame in seconds (default is params.ACK_TIMEOUT
                which is typically 5 seconds).
        
        Returns:
        --------
            tuple
                A tuple containing the ACK frame as a dictionary (or None if timeout) and the raw
                bytearray of all data read from the serial port during the wait.
        """
        buf    = bytearray()
        seen   : set[int] = set()
        total_b = 0
        t_end  = time.monotonic() + timeout
        while time.monotonic() < t_end:
            chunk = self.connection.serial.read(256)
            if chunk:
                total_b += len(chunk)
                logger.debug(f"[raw +{len(chunk):3d}B] {chunk.hex(' ')}")
                buf.extend(chunk)
            for off, raw in self.scan_frames(bytes(buf)):
                if off in seen:
                    continue
                seen.add(off)
                f = utils.parse_frame(raw)
                if f['is_ack']:
                    rc = (struct.unpack_from('<H', f['payload'], 0)[0]
                        if len(f['payload']) >= 2 else None)
                    rc_s = f"  retcode=0x{rc:04X}" if rc is not None else ""
                    logger.debug(f"[ACK]  seq={f['seq']:3d}  crc32={'OK' if f['crc32_ok'] else 'FAIL'}{rc_s}")
                    if f['seq'] == expected_seq:
                        return f, buf
                else:
                    logger.debug(f"[push] cmd=0x{f['cmd_set']:02X}/0x{f['cmd_id']:02X}"
                        f"  seq={f['seq']:3d}  len={f['length']}")
        logger.debug(f"[timeout]  {total_b} raw bytes  /  {len(seen)} frame(s) parsed")
        return None, buf


    def scan_frames(self, buf: bytes):
        """
        Yield (offset, raw_bytes) for every valid frame found in buf.
        This is a generator that scans through the buffer and identifies valid frames based on the
        DJI OSDK frame structure. It checks for the Start of Frame (SOF) byte, validates the length, 
        and verifies the CRC16 checksum for the header.
        
        Parameters:
        -----------
        buf: bytes
            The input byte buffer to scan for frames.
        
        Yields:
        -------
        Generator[Tuple[int, bytes]]
            A generator that yields tuples of (offset, raw_bytes) for each valid frame found in the input buffer.
        """
        i = 0
        while i < len(buf):
            idx = buf.find(params.SOF, i)
            if idx == -1:
                break
            if idx + params.HEADER_LEN > len(buf):
                break
            w0 = struct.unpack_from('<I', buf, idx)[0]
            length = (w0 >> 8) & 0x3FF
            if length < params.PKG_MIN or length > params.PKG_MAX:
                i = idx + 1; continue
            if idx + length > len(buf):
                break       # incomplete — wait for more data
            frame = buf[idx : idx + length]
            stored = struct.unpack_from('<H', frame, 10)[0]
            if utils.crc16(frame[:10]) != stored:
                i = idx + 1; continue
            yield idx, frame
            i = idx + length