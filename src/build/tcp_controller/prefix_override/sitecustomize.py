import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/schoolacc/ros2_ws/src/install/tcp_controller'
