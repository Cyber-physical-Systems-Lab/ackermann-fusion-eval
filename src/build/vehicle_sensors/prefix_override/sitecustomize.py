import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/meilin/zml/project/ros2_ws/src/install/vehicle_sensors'
