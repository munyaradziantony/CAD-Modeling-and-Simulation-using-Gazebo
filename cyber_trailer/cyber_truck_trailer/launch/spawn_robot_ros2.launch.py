import os

import launch
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution

from launch.actions import (
    IncludeLaunchDescription,
    ExecuteProcess,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch_ros.substitutions import FindPackageShare
import launch_ros
from launch_ros.actions import Node
import xacro
import random


def generate_launch_description():
    ####### DATA INPUT ##########
    xacro_file = "cyber_truck_trailer.urdf.xacro"

    package_description = "cyber_truck_trailer"

    # Position and orientation
    # [X, Y, Z]
    position = [0.0, 0.0, 1.0]
    # [Roll, Pitch, Yaw]
    orientation = [0.0, 0.0, 0.0]
    # Base Name or robot
    robot_base_name = "cyber_truck_trailer"
    ####### DATA INPUT END ##########

    # Path to robot model XACRO File
    robot_desc_path = os.path.join(
        get_package_share_directory(package_description), "urdf", xacro_file
    )

    # Robot Description in XACRO Format
    robot_desc = xacro.process_file(robot_desc_path)

    # Robot Description in XML Format
    xml = robot_desc.toxml()

    # Entity Name
    entity_name = robot_base_name + "-" + str(random.random())

    robot_description_content = launch_ros.descriptions.ParameterValue(Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]), " ",
        PathJoinSubstitution(
            [FindPackageShare("cyber_truck_trailer"), "urdf", "cyber_truck_trailer.urdf.xacro"]
        ),
    ]))
    
    robot_description = {"robot_description": robot_description_content}


    # Spawn ROBOT Set Gazebo (Does not spwan robot only communicates with the Gazebo Client)
    spawn_robot = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        name="spawn_entity",
        output="screen",
        arguments=[
            "-entity",
            entity_name,
            "-x",
            str(position[0]),
            "-y",
            str(position[1]),
            "-z",
            str(position[2]),
            "-R",
            str(orientation[0]),
            "-P",
            str(orientation[1]),
            "-Y",
            str(orientation[2]),
            "-topic",
            "/robot_description",
        ],
    )

    # Publish Robot Desciption in String form in the topic /robot_description
    publish_robot_description = Node(
        package="cyber_truck_trailer",
        executable="robot_description_publisher.py",
        name="robot_description_publisher",
        output="screen",
        arguments=[
            "-xml_string",
            xml,
            "-robot_description_topic",
            "/robot_description",
        ],
    )

    # Launch Config for Simulation Time
    use_sim_time = LaunchConfiguration("use_sim_time", default="true")

    # Robot State Publisher Node
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description],
        output="both",
    )

    # Static TF Transform
    tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="screen",
        arguments=["1", "0", "0", "0", "0", "0", "1", "/map", "/dummy_link"],
    )

    # create and return launch description object
    return LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            name="gui",
            default_value="True",
            description="Flag to enable joint_state_publisher_gui",
        ),
        launch.actions.DeclareLaunchArgument(
            name="use_sim_time",
            default_value="True",
            description="Flag to enable use_sim_time",
        ),
        robot_state_publisher,
        publish_robot_description,
        spawn_robot,
    ])
