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

    #Arguments controller_type to specify which controller to use
    declared_arguments = []
    declared_arguments.append(DeclareLaunchArgument(
            'controller_type',
            default_value='position',
            choices=['position', 'trajectory'],
            description="Choose 'position' or 'trajectory' controller."
        )
    )
    controller_type = LaunchConfiguration('controller_type')

    # Start the Joint State Broadcaster
    joint_state_broadcaster_node = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )  
    # Spawn the appropriate controller based on the controller_type argument
    position_controller_node = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["position_controller", "--controller-manager", "/controller_manager"],  
        condition=IfCondition(PythonExpression(["'", controller_type, "' == 'position'"]))
    ) 

    trajectory_controller_node = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["trajectory_controller", "--controller-manager", "/controller_manager"],  
        condition=IfCondition(PythonExpression(["'", controller_type, "' == 'trajectory'"]))
    ) 

    return LaunchDescription(declared_arguments + [
        joint_state_broadcaster_node,
        trajectory_controller_node,
        position_controller_node
    ])