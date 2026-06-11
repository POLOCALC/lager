DATA_PATH = '/home/polocalc/data'
DATA_FOLDER_PREFIX = 'telemetry'
CURRENT_DATA_FOLDER = 'current'

LOGGING_FILE_NAME = 'controller.log'
LOGGING_FORMAT = '[%(asctime)s.%(msecs)03d %(levelname)s] (%(name)s): %(message)s'
LOGGING_DATEFMT = '%Y-%m-%d %H:%M:%S'

DECODED_DATA_FILENAME = 'decoded_telemetry_data.pkl'

# GPIO settings for LED indicator
ENABLE_LED_INDICATOR = True
LED_INDICATOR_PIN = 17
LED_STATE_OFF = 0
LED_STATE_ON = 1
LED_STATE_BLINK = 2

# CLI settings
LOG_LINES_TO_SHOW = 20