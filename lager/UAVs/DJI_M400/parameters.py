# psdk_telemetry process
DEFAULT_BINARY      = "psdk_telemetry"        # name on PATH (~/.local/bin) or absolute path
DEFAULT_DEVICE      = "/dev/ttyGS0"           # only for a pre-check; the binary uses hal_uart.h
IMU_HZ              = 0                       # imu group: IMU navigation data, 50 Hz only; 0 = off (link bandwidth)
FAST_HZ             = 10                      # fast group: attitude, IMU, altitudes, fused position, obstacles, RC
STATUS_HZ           = 5                       # status group: GPS and RTK (5 Hz on the M400), visual odometry, RC link, flight state, ESCs
SLOW_HZ             = 1                       # slow group: battery, GPS date/time, home point
                                              # allowed: 1, 5, 10, 50, 100, 200, 400 Hz; groups with the same rate
                                              # share the PSDK limit of 242 bytes per rate
STATS_S             = 5                       # seconds between STATS records (0 = off)
CONNECT_TIMEOUT     = 60.0                    # s to wait for SUBSCRIBED; DjiCore_Init takes up to ~40 s when it fails
STOP_TIMEOUT        = 5.0                     # s to wait after SIGINT before SIGTERM/SIGKILL
SERVICE_GREETING_TIMEOUT = 5.0                # s to get the topic table from psdk-bridge after connecting

# logging to disk
TELEMETRY_FILENAME  = "uav_telemetry.bin"     # same name as the M600: the raw stream, record by record
PSDK_LOG_FILENAME   = "psdk_telemetry.log"    # the program's stderr (SDK log)
READ_SIZE           = 4096                    # bytes per os.read() on the pipe
BATCH_SIZE          = 50                      # chunks to accumulate before writing
FLUSH_EVERY         = 200                     # chunks between f.flush() calls
LOG_QUEUE_MAXSIZE   = 2000                    # chunks buffered in memory; the oldest are dropped if the writer is behind

# stream format (application/telemetry_stream.h)
SYNC                = b"\xA5\x5A"
HEADER_FMT          = "<BBHHqII"              # after SYNC: version, type, topic, length, host ns, aircraft ms, aircraft us
HEADER_SIZE         = 24                      # SYNC + HEADER_FMT
CRC_SIZE            = 2
MAX_PAYLOAD         = 2048                    # TELEMETRY_STREAM_MAX_PAYLOAD (the topic table is 2 + 39 bytes per topic)
STREAM_VERSION      = 1
TOPIC_NAME_SIZE     = 32
TOPIC_ENTRY_SIZE    = 7 + TOPIC_NAME_SIZE     # u16 topic, u16 Hz, u16 size, u8 subscribed, char name[32]

RECORD_DATA, RECORD_STATUS, RECORD_TOPIC_TABLE, RECORD_STATS = 1, 2, 3, 4
STATE_STARTING, STATE_CONNECTING, STATE_CONNECTED, STATE_SUBSCRIBED, STATE_STOPPING = 1, 2, 3, 4, 5
STATE_INFO, STATE_WARNING, STATE_ERROR = 10, 11, 100
STATE_NAMES = {1: "starting", 2: "connecting", 3: "connected", 4: "subscribed", 5: "stopping",
               10: "info", 11: "warning", 100: "error"}

EXIT_CODES = {0: "stopped",
              1: "bad options",
              2: "setup/credentials error",
              3: "port missing/busy or DjiCore_Init failed",
              4: "no topic subscribed",
              5: "reader gone",
              6: "link to the aircraft lost (no telemetry for 10 s)",
              130: "interrupted while connecting"}

# units (check on the ground, see step 9)
DOP_SCALE           = 0.01                    # hdop/pdop unit (PSDK header: 100 = 1.00)
