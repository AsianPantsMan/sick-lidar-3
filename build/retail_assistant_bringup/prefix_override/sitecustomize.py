import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/retail-assistant/SLAM/install/retail_assistant_bringup'
