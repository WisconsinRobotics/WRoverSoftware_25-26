import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/david-wang/Desktop/Projects/Wisconsin Robotics/WRoverSoftware_25-26/install/autonomous'
