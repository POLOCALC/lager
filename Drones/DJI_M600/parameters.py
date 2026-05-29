# parameters and constants for telemetry decoding and processing
DEFAULT_SERIAL_PORT = '/dev/serial0'  # default serial port for telemetry
DEFAULT_BAUDRATE = 115200             # default serial baudrate
SERIAL_TIMEOUT = 3.0                  # seconds
ACK_TIMEOUT = 3.0                     # seconds


# telemetry frequency
TELEMETRY_FREQ = 50  # Hz
TELEMETRY_FILENAME = "drone_telemetry.bin"  # filename for telemetry log (binary format)


# SDK control mode bit masks (from DJI OSDK dji_broadcast.hpp)
CRC_INIT = 0x3AA3
CMD_SET_GENERAL   = 0x00
CMD_ID_VERSION    = 0x00

SOF        = 0xAA
HEADER_LEN = 12
CRC32_LEN  = 4
PKG_MIN    = HEADER_LEN + CRC32_LEN
PKG_MAX    = 1024

SERIAL_READ_SIZE = 4096   # bytes per ser.read() call
BATCH_SIZE       = 50     # validated frames to accumulate before writing
FLUSH_EVERY      = 200    # frames between f.flush() calls  (~2 s at 100 Hz)

# timing constant
# The M600/A3 FC runs at 400 Hz. The 't_ms' field in the broadcast payload is
# NOT in milliseconds — it is a 400 Hz tick counter (1 tick = 2.5 ms).
FC_TICK_MS = 2.5   # ms per tick

# broadcast frame identification
CMD_SET_BROADCAST = 0x02
CMD_ID_BROADCAST  = 0x00

# flag bit masks (from DJI OSDK dji_broadcast.hpp, A3/N3/M600 firmware)
FLAG_TIME           = 0x0001
FLAG_QUATERNION     = 0x0002
FLAG_ACCELERATION   = 0x0004
FLAG_VELOCITY       = 0x0008
FLAG_ANGULAR_RATE   = 0x0010
FLAG_POSITION       = 0x0020
FLAG_GPSINFO        = 0x0040
FLAG_RTKINFO        = 0x0080
FLAG_MAG            = 0x0100
FLAG_RC             = 0x0200
FLAG_GIMBAL         = 0x0400
FLAG_STATUS         = 0x0800
FLAG_BATTERY        = 0x1000
FLAG_DEVICE         = 0x2000

# fixed payload offsets (data after cmd_set/cmd_id, starting at flag word) 
OFF_FLAG     = 0
OFF_TMS      = 2    # TimeStamp.time_ms  uint32 (400Hz ticks)
OFF_TNS      = 6    # TimeStamp.time_ns  uint32
OFF_Q        = 17   # Quaternion q0 q1 q2 q3  4×float32
OFF_A        = 33   # Accel ax ay az  3×float32  g
OFF_V        = 45   # Velocity vx vy vz  3×float32  m/s
OFF_VI       = 57   # VelocityInfo uint8
OFF_W        = 58   # AngularRate gx gy gz  3×float32  rad/s
OFF_LAT      = 70   # GlobalPosition latitude  float64  rad
OFF_LON      = 78   # GlobalPosition longitude float64  rad
OFF_ALT      = 86   # GlobalPosition altitude  float32  m
OFF_HGT      = 90   # GlobalPosition height    float32  m (AGL)
OFF_GPSH     = 94   # GlobalPosition gps_health uint8
OFF_RP       = 95   # RelativePosition (6×float32 + uint8 flags)
OFF_GPS_DATE = 120  # GPSInfo date  uint32 yyyymmdd
OFF_GPS_TIME = 124  # GPSInfo time  uint32 hhmmss
OFF_GPS_LON  = 128  # GPSInfo longitude int32 deg×1e7
OFF_GPS_LAT  = 132  # GPSInfo latitude  int32 deg×1e7
OFF_GPS_HFSL = 136  # GPSInfo HFSL      int32 mm
OFF_GPS_VEL  = 140  # GPSInfo velocityNED 3×float32 cm/s
OFF_GPS_DTL  = 152  # GPSInfo GPSDetail start
OFF_RTK      = 188  # RTK start (44B)
OFF_MX       = 232  # Mag x  int16
OFF_RC       = 238  # RC 6×int16
OFF_GIMBAL   = 250  # Gimbal 3×float32 + uint8
OFF_STATUS   = 263  # Status 4×uint8 (only when FLAG_STATUS set)
OFF_BAT      = 2    # Battery start (in 0x3000 frame, after flag word)
OFF_SDK      = 15   # SDKInfo start  (in 0x3000 frame)


# CRC16_IBM  poly 0x8005
CRC16_TAB = [
    0x0000,0xc0c1,0xc181,0x0140,0xc301,0x03c0,0x0280,0xc241,0xc601,0x06c0,0x0780,0xc741,0x0500,0xc5c1,0xc481,0x0440,
    0xcc01,0x0cc0,0x0d80,0xcd41,0x0f00,0xcfc1,0xce81,0x0e40,0x0a00,0xcac1,0xcb81,0x0b40,0xc901,0x09c0,0x0880,0xc841,
    0xd801,0x18c0,0x1980,0xd941,0x1b00,0xdbc1,0xda81,0x1a40,0x1e00,0xdec1,0xdf81,0x1f40,0xdd01,0x1dc0,0x1c80,0xdc41,
    0x1400,0xd4c1,0xd581,0x1540,0xd701,0x17c0,0x1680,0xd641,0xd201,0x12c0,0x1380,0xd341,0x1100,0xd1c1,0xd081,0x1040,
    0xf001,0x30c0,0x3180,0xf141,0x3300,0xf3c1,0xf281,0x3240,0x3600,0xf6c1,0xf781,0x3740,0xf501,0x35c0,0x3480,0xf441,
    0x3c00,0xfcc1,0xfd81,0x3d40,0xff01,0x3fc0,0x3e80,0xfe41,0xfa01,0x3ac0,0x3b80,0xfb41,0x3900,0xf9c1,0xf881,0x3840,
    0x2800,0xe8c1,0xe981,0x2940,0xeb01,0x2bc0,0x2a80,0xea41,0xee01,0x2ec0,0x2f80,0xef41,0x2d00,0xedc1,0xec81,0x2c40,
    0xe401,0x24c0,0x2580,0xe541,0x2700,0xe7c1,0xe681,0x2640,0x2200,0xe2c1,0xe381,0x2340,0xe101,0x21c0,0x2080,0xe041,
    0xa001,0x60c0,0x6180,0xa141,0x6300,0xa3c1,0xa281,0x6240,0x6600,0xa6c1,0xa781,0x6740,0xa501,0x65c0,0x6480,0xa441,
    0x6c00,0xacc1,0xad81,0x6d40,0xaf01,0x6fc0,0x6e80,0xae41,0xaa01,0x6ac0,0x6b80,0xab41,0x6900,0xa9c1,0xa881,0x6840,
    0x7800,0xb8c1,0xb981,0x7940,0xbb01,0x7bc0,0x7a80,0xba41,0xbe01,0x7ec0,0x7f80,0xbf41,0x7d00,0xbdc1,0xbc81,0x7c40,
    0xb401,0x74c0,0x7580,0xb541,0x7700,0xb7c1,0xb681,0x7640,0x7200,0xb2c1,0xb381,0x7340,0xb101,0x71c0,0x7080,0xb041,
    0x5000,0x90c1,0x9181,0x5140,0x9301,0x53c0,0x5280,0x9241,0x9601,0x56c0,0x5780,0x9741,0x5500,0x95c1,0x9481,0x5440,
    0x9c01,0x5cc0,0x5d80,0x9d41,0x5f00,0x9fc1,0x9e81,0x5e40,0x5a00,0x9ac1,0x9b81,0x5b40,0x9901,0x59c0,0x5880,0x9841,
    0x8801,0x48c0,0x4980,0x8941,0x4b00,0x8bc1,0x8a81,0x4a40,0x4e00,0x8ec1,0x8f81,0x4f40,0x8d01,0x4dc0,0x4c80,0x8c41,
    0x4400,0x84c1,0x8581,0x4540,0x8701,0x47c0,0x4680,0x8641,0x8201,0x42c0,0x4380,0x8341,0x4100,0x81c1,0x8081,0x4040,
]

# CRC32_Common  poly 0x04C11DB7 (reflected)
CRC32_TAB = [
    0x00000000,0x77073096,0xee0e612c,0x990951ba,0x076dc419,0x706af48f,0xe963a535,0x9e6495a3,
    0x0edb8832,0x79dcb8a4,0xe0d5e91e,0x97d2d988,0x09b64c2b,0x7eb17cbd,0xe7b82d07,0x90bf1d91,
    0x1db71064,0x6ab020f2,0xf3b97148,0x84be41de,0x1adad47d,0x6ddde4eb,0xf4d4b551,0x83d385c7,
    0x136c9856,0x646ba8c0,0xfd62f97a,0x8a65c9ec,0x14015c4f,0x63066cd9,0xfa0f3d63,0x8d080df5,
    0x3b6e20c8,0x4c69105e,0xd56041e4,0xa2677172,0x3c03e4d1,0x4b04d447,0xd20d85fd,0xa50ab56b,
    0x35b5a8fa,0x42b2986c,0xdbbbc9d6,0xacbcf940,0x32d86ce3,0x45df5c75,0xdcd60dcf,0xabd13d59,
    0x26d930ac,0x51de003a,0xc8d75180,0xbfd06116,0x21b4f4b5,0x56b3c423,0xcfba9599,0xb8bda50f,
    0x2802b89e,0x5f058808,0xc60cd9b2,0xb10be924,0x2f6f7c87,0x58684c11,0xc1611dab,0xb6662d3d,
    0x76dc4190,0x01db7106,0x98d220bc,0xefd5102a,0x71b18589,0x06b6b51f,0x9fbfe4a5,0xe8b8d433,
    0x7807c9a2,0x0f00f934,0x9609a88e,0xe10e9818,0x7f6a0dbb,0x086d3d2d,0x91646c97,0xe6635c01,
    0x6b6b51f4,0x1c6c6162,0x856530d8,0xf262004e,0x6c0695ed,0x1b01a57b,0x8208f4c1,0xf50fc457,
    0x65b0d9c6,0x12b7e950,0x8bbeb8ea,0xfcb9887c,0x62dd1ddf,0x15da2d49,0x8cd37cf3,0xfbd44c65,
    0x4db26158,0x3ab551ce,0xa3bc0074,0xd4bb30e2,0x4adfa541,0x3dd895d7,0xa4d1c46d,0xd3d6f4fb,
    0x4369e96a,0x346ed9fc,0xad678846,0xda60b8d0,0x44042d73,0x33031de5,0xaa0a4c5f,0xdd0d7cc9,
    0x5005713c,0x270241aa,0xbe0b1010,0xc90c2086,0x5768b525,0x206f85b3,0xb966d409,0xce61e49f,
    0x5edef90e,0x29d9c998,0xb0d09822,0xc7d7a8b4,0x59b33d17,0x2eb40d81,0xb7bd5c3b,0xc0ba6cad,
    0xedb88320,0x9abfb3b6,0x03b6e20c,0x74b1d29a,0xead54739,0x9dd277af,0x04db2615,0x73dc1683,
    0xe3630b12,0x94643b84,0x0d6d6a3e,0x7a6a5aa8,0xe40ecf0b,0x9309ff9d,0x0a00ae27,0x7d079eb1,
    0xf00f9344,0x8708a3d2,0x1e01f268,0x6906c2fe,0xf762575d,0x806567cb,0x196c3671,0x6e6b06e7,
    0xfed41b76,0x89d32be0,0x10da7a5a,0x67dd4acc,0xf9b9df6f,0x8ebeeff9,0x17b7be43,0x60b08ed5,
    0xd6d6a3e8,0xa1d1937e,0x38d8c2c4,0x4fdff252,0xd1bb67f1,0xa6bc5767,0x3fb506dd,0x48b2364b,
    0xd80d2bda,0xaf0a1b4c,0x36034af6,0x41047a60,0xdf60efc3,0xa867df55,0x316e8eef,0x4669be79,
    0xcb61b38c,0xbc66831a,0x256fd2a0,0x5268e236,0xcc0c7795,0xbb0b4703,0x220216b9,0x5505262f,
    0xc5ba3bbe,0xb2bd0b28,0x2bb45a92,0x5cb36a04,0xc2d7ffa7,0xb5d0cf31,0x2cd99e8b,0x5bdeae1d,
    0x9b64c2b0,0xec63f226,0x756aa39c,0x026d930a,0x9c0906a9,0xeb0e363f,0x72076785,0x05005713,
    0x95bf4a82,0xe2b87a14,0x7bb12bae,0x0cb61b38,0x92d28e9b,0xe5d5be0d,0x7cdcefb7,0x0bdbdf21,
    0x86d3d2d4,0xf1d4e242,0x68ddb3f8,0x1fda836e,0x81be16cd,0xf6b9265b,0x6fb077e1,0x18b74777,
    0x88085ae6,0xff0f6a70,0x66063bca,0x11010b5c,0x8f659eff,0xf862ae69,0x616bffd3,0x166ccf45,
    0xa00ae278,0xd70dd2ee,0x4e048354,0x3903b3c2,0xa7672661,0xd06016f7,0x4969474d,0x3e6e77db,
    0xaed16a4a,0xd9d65adc,0x40df0b66,0x37d83bf0,0xa9bcae53,0xdebb9ec5,0x47b2cf7f,0x30b5ffe9,
    0xbdbdf21c,0xcabac28a,0x53b39330,0x24b4a3a6,0xbad03605,0xcdd70693,0x54de5729,0x23d967bf,
    0xb3667a2e,0xc4614ab8,0x5d681b02,0x2a6f2b94,0xb40bbe37,0xc30c8ea1,0x5a05df1b,0x2d02ef8d,
]

# FlightStatus constants (from DJI OSDK dji_status.hpp)
FLIGHT_STATUS_STOPPED = 0
FLIGHT_STATUS_ON_GROUND = 1
FLIGHT_STATUS_IN_AIR = 2

FLIGHT_STATUS = {
    FLIGHT_STATUS_STOPPED:   "Stopped",
    FLIGHT_STATUS_ON_GROUND: "On ground",
    FLIGHT_STATUS_IN_AIR:    "In air",
}

# LandingGearMode constants (from DJI OSDK dji_status.hpp)
LANDING_GEAR_UNDEFINED = 0
LANDING_GEAR_DOWN = 1
LANDING_GEAR_UP_TO_DOWN = 2
LANDING_GEAR_UP = 3
LANDING_GEAR_DOWN_TO_UP = 4
LANDING_GEAR_HOLD = 5
LANDING_GEAR_PACKED = 6
LANDING_GEAR_PACKING_IN_PROGRESS = 7
LANDING_GEAR_UNPACKING_IN_PROGRESS = 8

LANDING_GEAR_MODE = {
    LANDING_GEAR_UNDEFINED:             "Undefined",
    LANDING_GEAR_DOWN:                  "Down",
    LANDING_GEAR_UP_TO_DOWN:            "Up to down",
    LANDING_GEAR_UP:                    "Up",
    LANDING_GEAR_DOWN_TO_UP:            "Down to up",
    LANDING_GEAR_HOLD:                  "Hold",
    LANDING_GEAR_PACKED:                "Packed",
    LANDING_GEAR_PACKING_IN_PROGRESS:   "Packing in progress",
    LANDING_GEAR_UNPACKING_IN_PROGRESS: "Unpacking in progress",
}

# StatusError (motor start inhibit) constants (from DJI OSDK dji_error.hpp)
STATUS_ERROR_NONE = 0
STATUS_ERROR_IMU_NEED_ADV_CALIBRATION = 1
STATUS_ERROR_IMU_SN_ERROR = 2
STATUS_ERROR_IMU_PREHEATING = 3
STATUS_ERROR_COMPASS_CALIBRATING = 4
STATUS_ERROR_IMU_NO_ATTITUDE = 5
STATUS_ERROR_NO_GPS_IN_NOVICE_MODE = 6
STATUS_ERROR_BATTERY_CELL_ERROR = 7
STATUS_ERROR_BATTERY_COMMUNICATION_ERROR = 8
STATUS_ERROR_BATTERY_VOLTAGE_TOO_LOW = 9
STATUS_ERROR_BATTERY_USER_LOW_LAND = 10
STATUS_ERROR_BATTERY_MAIN_VOL_LOW = 11
STATUS_ERROR_BATTERY_TEMP_VOL_LOW = 12
STATUS_ERROR_BATTERY_SMART_LOW_LAND = 13
STATUS_ERROR_BATTERY_NOT_READY = 14
STATUS_ERROR_RUNNING_SIMULATOR = 15
STATUS_ERROR_PACK_MODE = 16
STATUS_ERROR_IMU_ATTI_LIMIT = 17
STATUS_ERROR_NOT_ACTIVATED = 18
STATUS_ERROR_IN_FLYLIMIT_AREA = 19
STATUS_ERROR_IMU_BIAS_LIMIT = 20
STATUS_ERROR_ESC_ERROR = 21
STATUS_ERROR_IMU_INITING = 22
STATUS_ERROR_UPGRADING = 23
STATUS_ERROR_HAVE_RUN_SIM = 24
STATUS_ERROR_IMU_CALIBRATING = 25
STATUS_ERROR_TAKEOFF_TILT_TOO_LARGE = 26
STATUS_ERROR_INVALID_SN = 40
STATUS_ERROR_FLASH_OPERATING = 41
STATUS_ERROR_GPS_DISCONNECT = 42
STATUS_ERROR_INTERNAL_46 = 43
STATUS_ERROR_RECORDER_ERROR = 44
STATUS_ERROR_INVALID_PRODUCT = 45
STATUS_ERROR_IMU_DISCONNECTED = 56
STATUS_ERROR_RC_CALIBRATING = 57
STATUS_ERROR_RC_CALI_DATA_OUT_RANGE = 58
STATUS_ERROR_RC_QUIT_CALI = 59
STATUS_ERROR_RC_CENTER_OUT_RANGE = 60
STATUS_ERROR_RC_MAP_ERROR = 61
STATUS_ERROR_WRONG_AIRCRAFT_TYPE = 62
STATUS_ERROR_SOME_MODULE_NOT_CONFIGURED = 63
STATUS_ERROR_NS_ABNORMAL = 72
STATUS_ERROR_TOPOLOGY_ABNORMAL = 73
STATUS_ERROR_RC_NEED_CALI = 74
STATUS_ERROR_INVALID_FLOAT = 75
STATUS_ERROR_M600_BAT_TOO_FEW = 76
STATUS_ERROR_M600_BAT_AUTH_ERR = 77
STATUS_ERROR_M600_BAT_COMM_ERR = 78
STATUS_ERROR_M600_BAT_DIF_VOLT_LARGE_1 = 79
STATUS_ERROR_BATTERY_BOLTAHGE_DIFF_82 = 80
STATUS_ERROR_INVALID_VERSION = 81
STATUS_ERROR_GIMBAL_GYRO_ABNORMAL = 82
STATUS_ERROR_GIMBAL_ESC_PITCH_NO_DATA = 83
STATUS_ERROR_GIMBAL_ESC_ROLL_NO_DATA = 84
STATUS_ERROR_GIMBAL_ESC_YAW_NO_DATA = 85
STATUS_ERROR_TAKEOFF_EXCEPTION = 86
STATUS_ERROR_ESC_STALL_NEAR_GOUND = 87
STATUS_ERROR_ESC_UNBALANCE_ON_GRD = 88
STATUS_ERROR_ESC_PART_EMPTY_ON_GRD = 89
STATUS_ERROR_ENGINE_START_FAILED = 90
STATUS_ERROR_AUTO_TAKEOFF_LAUNCH_FAILED = 91
STATUS_ERROR_ROLL_OVER_ON_GRD = 92
STATUS_ERROR_BAT_VERSION_ERR = 93
STATUS_ERROR_RTK_INITING = 94
STATUS_ERROR_RTK_FAIL_TO_INIT = 95
STATUS_ERROR_START_MOTOR_FAIL_MOTOR_STARTED = 110
STATUS_ERROR_INTERNAL_111 = 111
STATUS_ERROR_ESC_CALIBRATING = 112
STATUS_ERROR_GPS_SIGNATURE_INVALID = 113
STATUS_ERROR_GIMBAL_CALIBRATING = 114
STATUS_ERROR_FORCE_DISABLE = 115
STATUS_ERROR_TAKEOFF_HEIGHT_EXCEPTION = 116
STATUS_ERROR_ESC_NEED_UPGRADE = 117
STATUS_ERROR_GYRO_DATA_NOT_MATCH = 118
STATUS_ERROR_APP_NOT_ALLOW = 119
STATUS_ERROR_COMPASS_IMU_MISALIGN = 120
STATUS_ERROR_FLASH_UNLOCK = 121
STATUS_ERROR_ESC_SCREAMING = 122
STATUS_ERROR_ESC_TEMP_HIGH = 123
STATUS_ERROR_BAT_ERR = 124
STATUS_ERROR_IMPACT_IS_DETECTED = 125
STATUS_ERROR_MODE_FAILURE = 126
STATUS_ERROR_CRAFT_FAIL_LATELY = 127
STATUS_ERROR_KILL_SWITCH_ON = 135
STATUS_ERROR_MOTOR_CODE_ERROR = 255

STATUS_ERROR = {
    STATUS_ERROR_NONE: "No error",
    STATUS_ERROR_IMU_NEED_ADV_CALIBRATION: "IMU needs adv calibration",
    STATUS_ERROR_IMU_SN_ERROR: "IMU serial number error",
    STATUS_ERROR_IMU_PREHEATING: "IMU preheating",
    STATUS_ERROR_COMPASS_CALIBRATING: "Compass calibrating",
    STATUS_ERROR_IMU_NO_ATTITUDE: "IMU no attitude output",
    STATUS_ERROR_NO_GPS_IN_NOVICE_MODE: "No GPS in Novice Mode",
    STATUS_ERROR_BATTERY_CELL_ERROR: "Battery cell error",
    STATUS_ERROR_BATTERY_COMMUNICATION_ERROR: "Battery comm error",
    STATUS_ERROR_BATTERY_VOLTAGE_TOO_LOW: "Battery voltage too low",
    STATUS_ERROR_BATTERY_USER_LOW_LAND: "User low battery land",
    STATUS_ERROR_BATTERY_MAIN_VOL_LOW: "Main battery voltage low",
    STATUS_ERROR_BATTERY_TEMP_VOL_LOW: "Battery temp/volt low",
    STATUS_ERROR_BATTERY_SMART_LOW_LAND: "Smart battery low land",
    STATUS_ERROR_BATTERY_NOT_READY: "Battery not ready",
    STATUS_ERROR_RUNNING_SIMULATOR: "Simulator running",
    STATUS_ERROR_PACK_MODE: "Packing mode (Inspire)",
    STATUS_ERROR_IMU_ATTI_LIMIT: "IMU attitude limit",
    STATUS_ERROR_NOT_ACTIVATED: "Device not activated",
    STATUS_ERROR_IN_FLYLIMIT_AREA: "In restricted area",
    STATUS_ERROR_IMU_BIAS_LIMIT: "IMU bias limit",
    STATUS_ERROR_ESC_ERROR: "ESC error",
    STATUS_ERROR_IMU_INITING: "IMU initializing",
    STATUS_ERROR_UPGRADING: "System upgrading",
    STATUS_ERROR_HAVE_RUN_SIM: "Simulator already run",
    STATUS_ERROR_IMU_CALIBRATING: "IMU calibrating",
    STATUS_ERROR_TAKEOFF_TILT_TOO_LARGE: "Takeoff tilt too large",
    STATUS_ERROR_INVALID_SN: "Invalid serial number",
    STATUS_ERROR_FLASH_OPERATING: "Flash operating",
    STATUS_ERROR_GPS_DISCONNECT: "GPS disconnected",
    STATUS_ERROR_INTERNAL_46: "Internal error",
    STATUS_ERROR_RECORDER_ERROR: "SD card error",
    STATUS_ERROR_INVALID_PRODUCT: "Invalid product",
    STATUS_ERROR_IMU_DISCONNECTED: "IMU disconnected",
    STATUS_ERROR_RC_CALIBRATING: "RC calibrating",
    STATUS_ERROR_RC_CALI_DATA_OUT_RANGE: "RC cal data out of range",
    STATUS_ERROR_RC_QUIT_CALI: "RC cal unfinished",
    STATUS_ERROR_RC_CENTER_OUT_RANGE: "RC center out of range",
    STATUS_ERROR_RC_MAP_ERROR: "RC mapping error",
    STATUS_ERROR_WRONG_AIRCRAFT_TYPE: "Wrong aircraft type",
    STATUS_ERROR_SOME_MODULE_NOT_CONFIGURED: "Modules not configured",
    STATUS_ERROR_NS_ABNORMAL: "Nav system abnormal",
    STATUS_ERROR_TOPOLOGY_ABNORMAL: "Topology abnormal",
    STATUS_ERROR_RC_NEED_CALI: "RC needs calibration",
    STATUS_ERROR_INVALID_FLOAT: "Illegal data detected",
    STATUS_ERROR_M600_BAT_TOO_FEW: "M600: Not enough batteries",
    STATUS_ERROR_M600_BAT_AUTH_ERR: "M600: Battery cert failed",
    STATUS_ERROR_M600_BAT_COMM_ERR: "M600: Battery comm error",
    STATUS_ERROR_M600_BAT_DIF_VOLT_LARGE_1: "M600: Bat volt diff large",
    STATUS_ERROR_BATTERY_BOLTAHGE_DIFF_82: "Battery volt diff error",
    STATUS_ERROR_INVALID_VERSION: "Version mismatch",
    STATUS_ERROR_GIMBAL_GYRO_ABNORMAL: "M600: Gimbal gyro error",
    STATUS_ERROR_GIMBAL_ESC_PITCH_NO_DATA: "M600: Gimbal pitch ESC error",
    STATUS_ERROR_GIMBAL_ESC_ROLL_NO_DATA: "M600: Gimbal roll ESC error",
    STATUS_ERROR_GIMBAL_ESC_YAW_NO_DATA: "M600: Gimbal yaw ESC error",
    STATUS_ERROR_TAKEOFF_EXCEPTION: "Takeoff exception",
    STATUS_ERROR_ESC_STALL_NEAR_GOUND: "ESC stall near ground",
    STATUS_ERROR_ESC_UNBALANCE_ON_GRD: "ESC unbalance on ground",
    STATUS_ERROR_ESC_PART_EMPTY_ON_GRD: "ESC part empty on ground",
    STATUS_ERROR_ENGINE_START_FAILED: "Engine start failed",
    STATUS_ERROR_AUTO_TAKEOFF_LAUNCH_FAILED: "Auto takeoff launch failed",
    STATUS_ERROR_ROLL_OVER_ON_GRD: "Rollover on ground",
    STATUS_ERROR_BAT_VERSION_ERR: "Battery version error",
    STATUS_ERROR_RTK_INITING: "RTK initializing",
    STATUS_ERROR_RTK_FAIL_TO_INIT: "RTK init failed",
    STATUS_ERROR_START_MOTOR_FAIL_MOTOR_STARTED: "Motor already started",
    STATUS_ERROR_INTERNAL_111: "Internal error",
    STATUS_ERROR_ESC_CALIBRATING: "ESC calibrating",
    STATUS_ERROR_GPS_SIGNATURE_INVALID: "GPS signature invalid",
    STATUS_ERROR_GIMBAL_CALIBRATING: "Gimbal calibrating",
    STATUS_ERROR_FORCE_DISABLE: "Force disable",
    STATUS_ERROR_TAKEOFF_HEIGHT_EXCEPTION: "Takeoff height abnormal",
    STATUS_ERROR_ESC_NEED_UPGRADE: "ESC needs upgrade",
    STATUS_ERROR_GYRO_DATA_NOT_MATCH: "IMU direction misaligned",
    STATUS_ERROR_APP_NOT_ALLOW: "APP not allowed",
    STATUS_ERROR_COMPASS_IMU_MISALIGN: "Compass/IMU misaligned",
    STATUS_ERROR_FLASH_UNLOCK: "Flash unlocked",
    STATUS_ERROR_ESC_SCREAMING: "ESC buzzing",
    STATUS_ERROR_ESC_TEMP_HIGH: "ESC temperature high",
    STATUS_ERROR_BAT_ERR: "Battery not in place",
    STATUS_ERROR_IMPACT_IS_DETECTED: "Impact detected",
    STATUS_ERROR_MODE_FAILURE: "Mode failure",
    STATUS_ERROR_CRAFT_FAIL_LATELY: "Recent craft failure",
    STATUS_ERROR_KILL_SWITCH_ON: "Kill switch on",
    STATUS_ERROR_MOTOR_CODE_ERROR: "Motor code error",
}

# DisplayMode constants (from DJI OSDK dji_status.hpp)
MODE_MANUAL_CTRL = 0           # Manual control
MODE_ATTITUDE = 1              # Attitude mode
MODE_P_GPS = 6                 # GPS position hold
MODE_HOTPOINT_MODE = 9         # Hotpoint mode
MODE_ASSISTED_TAKEOFF = 10     # Assisted takeoff
MODE_AUTO_TAKEOFF = 11         # Auto takeoff
MODE_AUTO_LANDING = 12         # Auto landing
MODE_NAVI_GO_HOME = 15         # Return-to-Home
MODE_NAVI_SDK_CTRL = 17        # SDK control mode
MODE_FORCE_AUTO_LANDING = 33   # Forced auto landing
MODE_SEARCH_MODE = 40          # Search mode (RC lost)
MODE_ENGINE_START = 41         # Motor starting

# DisplayMode lookup table (short descriptions)
DISPLAY_MODE = {
    MODE_MANUAL_CTRL:        "Manual control",
    MODE_ATTITUDE:           "Attitude mode",
    MODE_P_GPS:              "GPS position hold",
    MODE_HOTPOINT_MODE:      "Hotpoint mode",
    MODE_ASSISTED_TAKEOFF:   "Assisted takeoff",
    MODE_AUTO_TAKEOFF:       "Auto takeoff",
    MODE_AUTO_LANDING:       "Auto landing",
    MODE_NAVI_GO_HOME:       "Return-to-Home",
    MODE_NAVI_SDK_CTRL:      "SDK control mode",
    MODE_FORCE_AUTO_LANDING: "Forced auto landing",
    MODE_SEARCH_MODE:        "Search mode (RC lost)",
    MODE_ENGINE_START:       "Motor starting",
}


# RC mode parser
RC_MODE_P =  10000
RC_MODE_A =      0
RC_MODE_F = -10000

RC_MODE = {
    RC_MODE_P : "Position",
    RC_MODE_A : "Attitude",
    RC_MODE_F : "Force",
}