# Cyber Truck Trailer Simulation

### Authors
> [Anirudh Swarankar (UID: 121150653)](mailto:aniswa@umd.edu)<br>
> [Antony Munyaradzi (UID: 120482731)](mailto:mantony2@umd.edu)<br>
> [Pranav Deshkulkarni Manjunath (UID: 121162090)](mailto:pdeshaku@umd.edu)<br>
> [Varad Nerlekar (UID: 120501135)](mailto:nerlekar@umd.edu)<br>


### Description
This project simulates a truck with a trailer in Gazebo, featuring teleoperation and a proportional controller for movement.

## Features
- **Gazebo Simulation**: Visualize and interact with the truck-trailer model in a realistic environment.
- **Teleoperation**: Control the truck using a keyboard or a joystick interface.
- **Proportional** Controller: Move the truck with simple proportional control through service calls.

## Installation
To set up the project, ensure you have the following dependencies installed:

- ROS2 Galactic Full Desktop Image
- Gazebo
- RViz
- cyber_interfaces
- tf-transformations (Python package)
- pygame (Python package)
- pynput (Python package)

>[!TIP]
> You may use the provided Docker files to set up the environment.
> Refer to [ROS-Docker github repository](https://github.com/VaradNerlekarUMD/ROS-Docker/tree/main)
> Use command `./start_docker` to launch the docker container and you can then open new instances of the docker container using the command `docker exec -it <container_id> bash`

## Usage
To run the simulation and control the truck, follow these steps:

### Keyboard Teleop Mode
Open Terminal 1: Launch the simulation in Gazebo or RViz.
```
ros2 launch cyber_truck_trailer rviz_gazebo.launch.py
```

Open Terminal 2: Run the teleop keyboard node.
```
ros2 run teleop_interface teleop_keyboard
```

### Proportional Controller Mode
Open Terminal 1: Launch the simulation in Gazebo or RViz.
```
ros2 launch cyber_truck_trailer rviz_gazebo.launch.py
```

Open Terminal 2: Run the proportional controller.
```
ros2 run simple_controller proportional_controller
```

Open Terminal 3: Send service calls to control the truck - (refer the following example).
```
ros2 service call /goal cyber_interfaces/srv/Goal '{"pose": {"position": {"x": 10, "y": 10, "z": 0}, "orientation": {"x": 0, "y": 0, "z": 0, "w": 0}}}'
```

### Joystick Teleop Mode
Open Terminal 1: Launch the simulation in Gazebo or RViz.
```
ros2 launch cyber_truck_trailer rviz_gazebo.launch.py
```

Open Terminal 2: Run the teleop keyboard node.
```
ros2 run teleop_interface teleop_joystick_<alternate/traditional>
```

---