import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/jayden/ros2_realController_ws/src/keyboard_controller/install/keyboard_controller'
