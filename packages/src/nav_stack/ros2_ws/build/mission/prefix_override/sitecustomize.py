import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/tom/aeac-2026/packages/src/nav_stack/ros2_ws/install/mission'
