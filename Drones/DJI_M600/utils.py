import math
import logging
import struct

import Drones.DJI_M600.utils as utils
import Drones.DJI_M600.parameters as params

log = logging.getLogger(__name__)

def crc16(data: bytes) -> int:
    """
    Compute CRC-16-CCITT (XModem) over the given data.
    
    Parameters:
    -----------
    data: bytes
        The input data for which the CRC16 checksum is to be computed.

    Returns:
    --------
    int
        The computed CRC16 checksum as an integer.
    """
    crc = params.CRC_INIT
    for b in data:
        crc = (crc >> 8) ^ params.CRC16_TAB[(crc ^ b) & 0xFF]
    return crc & 0xFFFF

def crc32(data: bytes) -> int:
    """
    Compute CRC-32 (IEEE 802.3) over the given data.

    Parameters:
    -----------
    data: bytes
        The input data for which the CRC32 checksum is to be computed.

    Returns:
    --------
    int
        The computed CRC32 checksum as an integer.
    """
    crc = params.CRC_INIT
    for b in data:
        crc = (crc >> 8) ^ params.CRC32_TAB[(crc ^ b) & 0xFF]
    return crc & 0xFFFFFFFF


def parse_frame(raw: bytes) -> dict:
        """
        Decode a validated frame into a dict.
        
        Parameters:
        -----------
        raw: bytes
            The raw bytes of a validated DJI OSDK frame to be parsed into a dictionary format.
        
        Returns:
        --------
        dict
            A dictionary containing the parsed information from the frame, including:
            - 'length': The total length of the frame.
            - 'is_ack': A boolean indicating whether the frame is an ACK.
            - 'seq': The sequence number extracted from the frame.
            - 'cmd_set': The command set identifier (None for ACK frames).
            - 'cmd_id': The command identifier (None for ACK frames).
            - 'payload': The payload data extracted from the frame.
            - 'crc32_ok': A boolean indicating whether the CRC32 checksum is valid.
            - 'raw': The original raw bytes of the frame.
        """
        w0     = struct.unpack_from('<I', raw, 0)[0]
        length = (w0 >> 8) & 0x3FF
        is_ack = bool((w0 >> 29) & 1)
        seq    = struct.unpack_from('<H', raw, 8)[0]
        crc32_ok = (utils.crc32(raw[:length - params.CRC32_LEN]) ==
                    struct.unpack_from('<I', raw, length - params.CRC32_LEN)[0])
        if is_ack:
            cmd_set, cmd_id = None, None
            payload = raw[params.HEADER_LEN : length - params.CRC32_LEN]
        else:
            cmd_set = raw[params.HEADER_LEN]
            cmd_id  = raw[params.HEADER_LEN + 1]
            payload = raw[params.HEADER_LEN + 2 : length - params.CRC32_LEN]
        return {
            'length': length, 'is_ack': is_ack, 'seq': seq,
            'cmd_set': cmd_set, 'cmd_id': cmd_id,
            'payload': payload, 'crc32_ok': crc32_ok, 'raw': raw,
        }


def decode_broadcast(data: bytes) -> dict | None:
    """
    Decode a cmd=0x02/0x00 broadcast payload (bytes after cmd_set/cmd_id).

    Returns a dict with all telemetry fields, or None if the payload is too short.
    Two frame types are handled:
    - flag=0x07FF / 0x0FFF  : full telemetry (263 or 267 bytes)
    - flag=0x3000           : battery + SDKInfo only (17 bytes)
    """
    if len(data) < 4:
        return None

    flag = struct.unpack_from('<H', data, params.OFF_FLAG)[0]

    # battery + device only (0x3000)
    if flag == (params.FLAG_BATTERY | params.FLAG_DEVICE):
        if len(data) < params.OFF_SDK + 2:
            return None
        bat_cap, bat_v, bat_i, bat_pct = struct.unpack_from('<IiiB', data, params.OFF_BAT)
        sdk_ctrl, sdk_dev = struct.unpack_from('<BB', data, params.OFF_SDK)
        return {
            'flag':             flag,
            'bat_capacity_mah': bat_cap,
            'bat_voltage_mv':   bat_v,
            'bat_current_ma':   bat_i,
            'bat_percentage':   bat_pct,
            'sdk_control_mode': sdk_ctrl,
            'sdk_device_bits':  sdk_dev,
        }

    # standard telemetry (0x07FF or 0x0FFF)
    if len(data) < params.OFF_STATUS:
        return None

    t_ticks = struct.unpack_from('<I', data, params.OFF_TMS)[0]
    t_ns    = struct.unpack_from('<I', data, params.OFF_TNS)[0]

    q0, q1, q2, q3 = struct.unpack_from('<4f', data, params.OFF_Q)
    roll, pitch, yaw = utils.quat_to_euler(q0, q1, q2, q3)
    qmag = math.sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3)

    ax, ay, az   = struct.unpack_from('<3f', data, params.OFF_A)
    vx, vy, vz   = struct.unpack_from('<3f', data, params.OFF_V)
    vel_info      = data[params.OFF_VI]
    gx, gy, gz   = struct.unpack_from('<3f', data, params.OFF_W)

    lat_rad = struct.unpack_from('<d', data, params.OFF_LAT)[0]
    lon_rad = struct.unpack_from('<d', data, params.OFF_LON)[0]
    alt_m   = struct.unpack_from('<f', data, params.OFF_ALT)[0]
    hgt_m   = struct.unpack_from('<f', data, params.OFF_HGT)[0]
    gps_h   = data[params.OFF_GPSH]

    rp_down, rp_front, rp_right, rp_back, rp_left, rp_up = \
        struct.unpack_from('<6f', data, params.OFF_RP)

    gps_date, gps_time_hhmmss = struct.unpack_from('<II', data, params.OFF_GPS_DATE)
    gps_lon_raw, gps_lat_raw, gps_hfsl_mm = struct.unpack_from('<3i', data, params.OFF_GPS_LON)
    gps_vn, gps_ve, gps_vd = struct.unpack_from('<3f', data, params.OFF_GPS_VEL)
    hdop, pdop  = struct.unpack_from('<2f', data, params.OFF_GPS_DTL)
    gps_nsv     = struct.unpack_from('<H', data, params.OFF_GPS_DTL + 32)[0]

    rtk_lon, rtk_lat = struct.unpack_from('<2d', data, params.OFF_RTK + 8)
    rtk_hfsl         = struct.unpack_from('<f',  data, params.OFF_RTK + 24)[0]
    rtk_yaw          = struct.unpack_from('<h',  data, params.OFF_RTK + 40)[0]
    rtk_pos_health   = data[params.OFF_RTK + 42]
    rtk_yaw_health   = data[params.OFF_RTK + 43]

    mx, my, mz = struct.unpack_from('<3h', data, params.OFF_MX)

    rc_roll, rc_pitch, rc_yaw, rc_thr, rc_mode, rc_gear = \
        struct.unpack_from('<6h', data, params.OFF_RC)

    gim_roll, gim_pitch, gim_yaw = struct.unpack_from('<3f', data, params.OFF_GIMBAL)
    gim_flags = data[params.OFF_GIMBAL + 12]

    out = {
        'flag':     flag,
        't_ticks':  t_ticks,
        't_ns':     t_ns,
        'q':        (q0, q1, q2, q3),
        'qmag':     qmag,
        'roll':     roll,
        'pitch':    pitch,
        'yaw':      yaw,
        'ax': ax,   'ay': ay,   'az': az,
        'vx': vx,   'vy': vy,   'vz': vz,
        'vel_info': vel_info,
        'gx': gx,   'gy': gy,   'gz': gz,
        'lat_deg':    math.degrees(lat_rad),
        'lon_deg':    math.degrees(lon_rad),
        'alt_m':      alt_m,
        'hgt_m':      hgt_m,
        'gps_health': gps_h,
        'rp_down':  rp_down,  'rp_front': rp_front,  'rp_right': rp_right,
        'rp_back':  rp_back,  'rp_left':  rp_left,   'rp_up':    rp_up,
        'gps_date':    gps_date,
        'gps_time':    gps_time_hhmmss,
        'gps_lon_deg': gps_lon_raw / 1e7,
        'gps_lat_deg': gps_lat_raw / 1e7,
        'gps_hfsl_m':  gps_hfsl_mm / 1000.0,
        'gps_vn_cms':  gps_vn,
        'gps_ve_cms':  gps_ve,
        'gps_vd_cms':  gps_vd,
        'hdop': hdop,  'pdop': pdop,  'gps_nsv': gps_nsv,
        'rtk_lon_deg':    rtk_lon,
        'rtk_lat_deg':    rtk_lat,
        'rtk_hfsl_m':     rtk_hfsl,
        'rtk_yaw_deg':    rtk_yaw * 0.1,
        'rtk_pos_health': rtk_pos_health,
        'rtk_yaw_health': rtk_yaw_health,
        'mx': mx,  'my': my,  'mz': mz,
        'rc_roll':  rc_roll,  'rc_pitch': rc_pitch,
        'rc_yaw':   rc_yaw,   'rc_thr':   rc_thr,
        'rc_mode':  rc_mode,  'rc_gear':  rc_gear,
        'gim_roll': gim_roll,  'gim_pitch': gim_pitch,
        'gim_yaw':  gim_yaw,   'gim_flags': gim_flags,
    }

    if (flag & params.FLAG_STATUS) and len(data) >= params.OFF_STATUS + 4:
        fl_status, fl_mode, fl_gear, fl_error = \
            struct.unpack_from('<4B', data, params.OFF_STATUS)
        out.update({
            'flight_status': fl_status,
            'display_mode':  fl_mode,
            'landing_gear':  fl_gear,
            'flight_error':  fl_error,
        })

    return out

def quat_to_euler(q0: float, q1: float, q2: float, q3: float) -> tuple:
    """
    Convert quaternion (q0=w, q1=x, q2=y, q3=z) to (roll, pitch, yaw) in degrees.
    Parameters:
    -----------
    q0, q1, q2, q3: float
        The components of the quaternion.
    Returns:
    --------
    tuple
        A tuple containing the Euler angles (roll, pitch, yaw) in degrees.
    """
    sinr_cosp = 2 * (q0 * q1 + q2 * q3)
    cosr_cosp = 1 - 2 * (q1 * q1 + q2 * q2)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2 * (q0 * q2 - q3 * q1)
    pitch = math.copysign(math.pi / 2, sinp) if abs(sinp) >= 1 else math.asin(sinp)

    siny_cosp = 2 * (q0 * q3 + q1 * q2)
    cosy_cosp = 1 - 2 * (q2 * q2 + q3 * q3)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)
