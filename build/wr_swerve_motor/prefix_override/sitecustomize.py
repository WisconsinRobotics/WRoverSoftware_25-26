import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/wizk/Desktop/Robotics/WRoverSoftware_25-26/install/wr_swerve_motor'
