import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/min/Workspace/scout_mini/install/scout_mini_tools'
