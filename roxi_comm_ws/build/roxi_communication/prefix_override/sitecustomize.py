import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/aleks/ROXI_Factory_Tool_ROS2/roxi_comm_ws/install/roxi_communication'
