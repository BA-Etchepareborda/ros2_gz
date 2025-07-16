import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/baltazar/proyectos_ros/basic_nav/install/mobile_robot'
