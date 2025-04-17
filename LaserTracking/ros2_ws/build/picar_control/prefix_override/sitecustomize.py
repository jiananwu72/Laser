import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/chen/Laser/LaserTracking/ros2_ws/install/picar_control'
