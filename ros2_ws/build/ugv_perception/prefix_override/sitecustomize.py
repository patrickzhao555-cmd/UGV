import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/bluelule/ugv_project/ros2_ws/install/ugv_perception'
