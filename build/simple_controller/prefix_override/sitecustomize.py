import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/munyaantony/Desktop/Cyber-Truck-and-Trailer-Simulation-in-Gazebo/install/simple_controller'
