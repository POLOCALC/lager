import sys
import time
sys.path.append('..')

from Controller import Controller

def main():
    config_file = '../config/follow_POI_config.yaml'
    controller = Controller(config_file)
    
    # TODO


if __name__ == "__main__":
    main()