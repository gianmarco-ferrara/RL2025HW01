import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler 
from launch.event_handlers import OnProcessExit 
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution 
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition

def generate_launch_description():

    # Path to packages
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_armando_description = get_package_share_directory('armando_description')

    # Include Gazebo launch file
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': '-r empty.sdf'}.items(),
    )

    #urdf_path = os.path.join(pkg_armando_description, "urdf", "arm.urdf")


    # Load robot description from xacro
    xacro_file_name = 'arm.urdf.xacro'
    xacro = os.path.join(get_package_share_directory('armando_description'), "urdf", xacro_file_name)

    robot_description_param = {"robot_description": Command(['xacro ', xacro])}

    # Publish robot joint states to TF transforms
    load_robot_description_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description_param],
    )

    # Spawn robot entity in Gazebo
    spawn_entity_node = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-topic', '/robot_description',
            '-name', 'armando',
            '-allow_renaming', 'true',
            '-z', '0.3'
        ],
    )

    # Camera bridge
    bridge_camera = Node(
        package='ros_ign_bridge', 
        executable='parameter_bridge',
        arguments=[
            '/camera@sensor_msgs/msg/Image@ignition.msgs.Image', 
            '/camera_info@sensor_msgs/msg/CameraInfo@ignition.msgs.CameraInfo', 
            '--ros-args', 
            '-r', '/camera:=/videocamera', 
        ],
        output='screen'
    )
    
    return LaunchDescription([
        gazebo_launch,
        load_robot_description_node,
        spawn_entity_node,
        bridge_camera
    ])