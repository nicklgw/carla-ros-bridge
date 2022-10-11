import launch
from launch.substitutions import EnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
import os
from ament_index_python.packages import get_package_share_directory
import yaml
import ament_index_python.packages


def generate_launch_description():
    
    mpc_parameters_configuration = os.path.join(os.getcwd(), 'src/ros-bridge/carla_shenlan_projects/carla_shenlan_mpc_controller/config', 'mpc_parameters_configuration_dynamics_coupled.yaml')
    rviz_config_dir = os.path.join(os.getcwd(), 'src/ros-bridge/carla_shenlan_projects/carla_shenlan_mpc_controller/rviz', 'mpc_vis.rviz')
    
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'node_prefix',
            default_value=[EnvironmentVariable("USER"), '_'],
            description='Prefix for node names'
        ),
        Node(
            package='carla_shenlan_mpc_controller',
            # namespace='mpc_trajectory_tracking',
            executable='mpc_trajectory_tracking_dynamics_coupled_node',
            name='mpc_trajectory_tracking_dynamics_coupled_node',
            parameters=[mpc_parameters_configuration],
            # remappings=None,
            # arguments=None,
            output='screen',
        ),
        Node(package='rviz2',
             executable='rviz2',
             output='screen',
             arguments=['-d', rviz_config_dir]),
    ])
