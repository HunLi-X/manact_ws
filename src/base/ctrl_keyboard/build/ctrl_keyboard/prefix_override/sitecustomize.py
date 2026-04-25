import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/unitree/ctrl_keyboard/install/ctrl_keyboard'
