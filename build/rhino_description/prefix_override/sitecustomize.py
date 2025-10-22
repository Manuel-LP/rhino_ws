import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/pin-pin/rhino2/rhino_ws/install/rhino_description'
