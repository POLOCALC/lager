# these paths are used if the module is run as an independent script
DATA_PATH = '/home/polocalc/pointing_control_data'
DATA_FOLDER_PREFIX = 'telemetry'
CURRENT_DATA_FOLDER = 'current'

# logging defaults if the module is run as an independent script
LOGGING_LEVEL = "INFO"
LOGGING_FILE_NAME = 'controller.log'
LOGGING_FORMAT = '[%(asctime)s.%(msecs)03d %(levelname)s] (%(name)s): %(message)s'
LOGGING_DATEFMT = '%Y-%m-%d %H:%M:%S'

DECODED_DATA_FILENAME = 'pointing_control_data.pkl'

# GPIO settings for LED indicator
ENABLE_LED_INDICATOR = False
LED_INDICATOR_PIN = 17
LED_STATE_OFF = 0
LED_STATE_ON = 1
LED_STATE_BLINK = 2

# CLI settings
LOG_LINES_TO_SHOW = 20