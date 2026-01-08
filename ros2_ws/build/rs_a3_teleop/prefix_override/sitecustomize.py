import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/wy/RS/A3/ros2_ws/install/rs_a3_teleop'
