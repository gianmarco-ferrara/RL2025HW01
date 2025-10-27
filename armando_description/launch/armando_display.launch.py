import os

# Import utilities for locating packages and defining launch actions
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command

def generate_launch_description():
    # Path to the URDF file
    urdf_file_name = 'arm.urdf'
    urdf = os.path.join(
        get_package_share_directory('armando_description'), 
        'urdf',
        urdf_file_name
    )
    
    # Read the URDF file

    # with open(urdf, 'r') as infp:
    #     robot_desc = infp.read()  

    # robot_description_links = {"robot_description": robot_desc}

    # Use xacro to process the URDF file
    xacro_file_name = 'arm.urdf.xacro'
    xacro = os.path.join(get_package_share_directory('armando_description'), "urdf", xacro_file_name)

    robot_description_links = {"robot_description": Command(['xacro ', xacro])}

    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "rviz_config_file",
            default_value=PathJoinSubstitution([
                FindPackageShare("armando_description"), "config", "armando_description.rviz"
            ]),
            description="RViz config file (absolute path) to use when launching RViz."
        )
    )

    # Provides a GUI to move the robot’s joints manually
    joint_state_publisher_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
    )

    # Publishes the robot’s TF tree based on the URDF
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[robot_description_links],
    )

    # Starts RViz with the given configuration file
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", LaunchConfiguration("rviz_config_file")],
    )

    return LaunchDescription(declared_arguments + [
        joint_state_publisher_node,
        robot_state_publisher_node,
        rviz_node
    ])