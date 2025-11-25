import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/mostafa/www/robot-ter/robot_ter_ws/install/robot_ter_bringup'
