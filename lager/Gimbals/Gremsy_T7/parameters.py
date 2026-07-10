# ------------------------------------------------------------------
#  Utility parameters
#  These parameters control the behavior of utility functions such as
#  the message polling loop and the interpolation function.
# ------------------------------------------------------------------
DEFAULT_SERIAL_PORT          = '/dev/serial1' # serial port
DEFAULT_BAUDRATE             = 115200         # serial baudrate
HEARTBEAT_FREQUENCY          = 1              # heartbeat frequency [Hz]
TIMEOUT                      = 5              # message polling timeout [seconds]

MESSAGE_POLL_FREQUENCY  = 100  # adjust according to the expected message rates to avoid busy-waiting
                               # generally 100Hz is a good choice for high-rate data logging, 
                               # but you can set it lower to reduce CPU usage if you only care about 
                               # low-rate messages

TELEMETRY_DATA_BUFFER    = 100                    # number of messages to store in the telemetry log  
                                                  # buffer before writing to file
TELEMETRY_FILENAME       = "gimbal_telemetry.bin" # filename for telemetry log (binary format)

DEFAULT_MAV_RATE_ORIEN   = 10  # MOUNT_ORIENTATION rate             [Hz]
DEFAULT_MAV_RATE_IMU     = 10  # RAW_IMU rate                       [Hz]
DEFAULT_MAV_RATE_ENCCNT  = 10  # MOUNT_STATUS (encoder count) rate  [Hz]
DEFAULT_MAV_RATE_ST      = 5   # SYS_STATUS rate                    [Hz]
DEFAULT_MAV_TS_ENCNT     = 0   # encoder count timestamp mode       [0/1]
DEFAULT_MAV_EMIT_HB      = 1   # emit heartbeat                     [0/1]

MAX_MAV_RATE_ORIEN       = 100  # Hz
MAX_MAV_RATE_IMU         =  50  # Hz
MAX_MAV_RATE_ENCCNT      =  30  # Hz
MAX_MAV_RATE_ST          =   5  # Hz



# ------------------------------------------------------------------
#  Movement parameters (not used in the code, found empirically)
# ------------------------------------------------------------------
DEFAULT_YAW_SPEED   = 180 # [deg/s]
DEFAULT_PITCH_SPEED = 120 # [deg/s]
DEFAULT_ROLL_SPEED  = 140 # [deg/s]


# ------------------------------------------------------------------
#  Stiffness, hold strength, and filter parameters
#  These parameters control the stiffness and hold strength of the gimbal
#  as well as the output and gyro filters. Adjust these parameters to
#  achieve the desired responsiveness and stability of the gimbal. Higher
#  stiffness and hold strength values will result in a more rigid and less
#  responsive gimbal, while lower values will result in a more flexible and
#  responsive gimbal. Adjust the filter parameters to reduce noise and heat.
# ------------------------------------------------------------------
DEFAULT_STIFF_TILT = 30  # pitch stiffness     [0-100]
DEFAULT_STIFF_ROLL = 35  # roll stiffness      [0-100]
DEFAULT_STIFF_PAN  = 45  # yaw stiffness       [0-100]
DEFAULT_PWR_TILT   = 40  # pitch hold strength [0-100]
DEFAULT_PWR_ROLL   = 40  # roll hold strength  [0-100]
DEFAULT_PWR_PAN    = 40  # yaw hold strength   [0-100]
DEFAULT_FILTER_OUT = 0   # output filter       [0-100]
DEFAULT_GYRO_LPF   = 8   # gyro LPF filter     [0-100]


# ------------------------------------------------------------------
#  Follow mode parameters
#  These parameters control the behavior of the gimbal in follow mode.
#  Adjust the speed parameters to control how fast the gimbal follows 
#  the target. Adjust the smooth parameters to control how much the 
#  gimbal smooths the follow motion (higher values result in smoother 
#  motion, but less responsive motion).
# ------------------------------------------------------------------
DEFAULT_FLW_SP_TILT  = 90   # follow speed  - pitch [deg/s]
DEFAULT_FLW_SP_PAN   = 90   # follow speed  - yaw   [deg/s]
DEFAULT_FLW_LPF_TILT = 40   # follow smooth - pitch [0-100]
DEFAULT_FLW_LPF_PAN  = 40   # follow smooth - yaw   [0-100]
DEFAULT_FLW_WD_TILT  = 0    # follow window - pitch [degrees]
DEFAULT_FLW_WD_PAN   = 0    # follow window - yaw   [degrees]


# ------------------------------------------------------------------
#  Rate control parameters
#  These parameters control the smoothing and speed of RC commands.
#  Adjust these to achieve the desired responsiveness and stability 
#  of the gimbal in response to RC
# ------------------------------------------------------------------
DEFAULT_RC_LPF_TILT = 25  # command smooth - pitch [0-100]
DEFAULT_RC_LPF_ROLL = 25  # command smooth - roll  [0-100]
DEFAULT_RC_LPF_PAN  = 25  # command smooth - yaw   [0-100]

DEFAULT_RC_SPD_TILT = 30  # command speed - pitch [deg/s]
DEFAULT_RC_SPD_ROLL = 30  # command speed - roll  [deg/s]
DEFAULT_RC_SPD_PAN  = 30  # command speed - yaw   [deg/s]


# ------------------------------------------------------------------
#  Motion limits
#  These parameters define the mechanical limits of the gimbal.
#  DO NOT CHANGE THE MECHANICAL LIMITS, as they are based on the physical
#  constraints of the gimbal. Adjust the RC limits to leave a margin for
#  motion corrections, but do not set them too close to the mechanical
#  limits to avoid saturating the gimbal and causing instability.
# ------------------------------------------------------------------
TILT_MECH_LIM_MIN = -150  # degrees, DO NOT CHANGE
TILT_MECH_LIM_MAX =  150  # degrees, DO NOT CHANGE
ROLL_MECH_LIM_MIN = -264  # degrees, DO NOT CHANGE
ROLL_MECH_LIM_MAX =   80  # degrees, DO NOT CHANGE
PAN_MECH_LIM_MIN  = -345  # degrees, DO NOT CHANGE
PAN_MECH_LIM_MAX  =  345  # degrees, DO NOT CHANGE

PAN_MARGIN = 20  # degrees, margin to leave from mechanical limits for motion corrections

# change following leaving margin for motion corrections
DEFAULT_RC_LIM_MIN_TILT =  -120  # pitch min limit [degrees]
DEFAULT_RC_LIM_MAX_TILT =   120  # pitch max limit [degrees]
DEFAULT_RC_LIM_MIN_ROLL =   -45  # roll  min limit [degrees]
DEFAULT_RC_LIM_MAX_ROLL =    45  # roll  max limit [degrees]
DEFAULT_RC_LIM_MIN_PAN  =  PAN_MECH_LIM_MIN + PAN_MARGIN  # yaw   min limit [degrees]
DEFAULT_RC_LIM_MAX_PAN  =  PAN_MECH_LIM_MAX - PAN_MARGIN  # yaw   max limit [degrees]


# ------------------------------------------------------------------
#  Damping parameters
#  These parameters control the damping of the gimbal. Adjust these
#  parameters to reduce overshoot and oscillations in response to fast
#  movements or disturbances. Higher values will result in more damping,
#  which can help stabilize the gimbal, but may also reduce responsiveness.
# ------------------------------------------------------------------
DEFAULT_TILT_DAMPING = 20  # pitch damping [0-100]
DEFAULT_ROLL_DAMPING = 15  # roll  damping [0-100]
DEFAULT_PAN_DAMPING  = 20  # yaw   damping [0-100]


# ------------------------------------------------------------------
#  Miscellaneous parameters
#  These parameters control various aspects of the gimbal's behavior and
#  configuration. Adjust these parameters as needed for your specific use case.
# ------------------------------------------------------------------
DEFAULT_GMB_HOME_PAN  = 0    # home pan angle           [degrees]
DEFAULT_MAPPING_ANGLE = 90   # mapping mode tilt angle  [degrees]
DEFAULT_GYRO_TRUST    = 220  # gyro sensor trust        [0-255]

DEFAULT_RC_DZONE_TILT = 0    # RC dead-zone - pitch [0-100]
DEFAULT_RC_DZONE_ROLL = 0    # RC dead-zone - roll  [0-100]
DEFAULT_RC_DZONE_PAN  = 0    # RC dead-zone - yaw   [0-100]

DEFAULT_RC_MODE_TILT  = 1     # RC mode - pitch [0/1]
DEFAULT_RC_MODE_ROLL  = 0     # RC mode - roll  [0/1]
DEFAULT_RC_MODE_PAN   = 1     # RC mode - yaw   [0/1]

DEFAULT_RC_TRIM_TILT  = 0     # RC trim - pitch [degrees]
DEFAULT_RC_TRIM_ROLL  = 0     # RC trim - roll  [degrees]

DEFAULT_RC_CHAN_STILT = 2    # RC channel - stilt/tilt speed
DEFAULT_RC_CHAN_SPAN  = 2    # RC channel - span/pan speed
DEFAULT_RC_CHAN_TILT  = 1    # RC channel - tilt angle
DEFAULT_RC_CHAN_ROLL  = 7    # RC channel - roll angle
DEFAULT_RC_CHAN_PAN   = 0    # RC channel - pan angle
DEFAULT_RC_CHAN_MODE  = 6    # RC channel - mode switch

DEFAULT_RC_TYPE         = 15 # RC input type
DEFAULT_RC_REVERSE_AXIS = 0  # RC axis reversal bitmask (bit 0 = pitch, bit 1 = roll, bit 2 = yaw)