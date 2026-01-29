import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/wiscrobo/workspace/WRoverSoftware_25-26/localization_workspace/install/wr_fusion'
