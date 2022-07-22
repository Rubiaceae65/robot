
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros import actions

from launch_ros.descriptions import ComposableNode
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

import os
import launch
import launch_ros.actions
import launch.substitutions
import yaml

from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory
def generate_launch_description():
   # param_config = os.path.join(get_package_share_directory('imu_filter_madgwick'), 'config', 'imu_filter.yaml')
   # with open(param_config, 'r') as f:
   #     params = yaml.safe_load(f)['imu_filter']['ros__parameters']

    container = ComposableNodeContainer(
        node_name='imu_filter_container',
        node_namespace='',
        package='rclcpp_components',
        node_executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='imu_filter_madgwick',
                node_plugin='ImuFilterMadgwickRos',
                node_name='imu_filter',
                parameters=[
                     {"publish_tf": False},
                     {"use_mag": False},
                ],
            
                )],
        output='screen',
    ) 


    ld = LaunchDescription([
        actions.Node(
            executable='/home/user/src/robot2/ros2-workspace/src/mpu9250_python/mpu9250_python/mpu_node.py',
            name='mpu9250_python',
            output='screen'
            ), 
        ]

    )
    #ld = LaunchDescription()
    ld.add_action(container)


    return ld
