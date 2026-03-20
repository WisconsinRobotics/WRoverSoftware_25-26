import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/teggatz/documents/WRoverSoftware_25-26/install/wr_path_planning'
