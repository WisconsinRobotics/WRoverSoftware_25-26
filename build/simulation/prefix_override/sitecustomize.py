import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/teggatz/Documents/code/WRoverSoftware_25-26/install/simulation'
