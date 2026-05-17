import sys
if sys.prefix == '/home/david-wang/.venv':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/david-wang/Desktop/Projects/Wisconsin Robotics/WRoverSoftware_25-26/install/gps_tools'
