import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/kendell/retail_assistant_ws/install/retail_assistant_bringup'
