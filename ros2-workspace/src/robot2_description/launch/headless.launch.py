import launch
from launch.substitutions import Command, LaunchConfiguration
import launch_ros
import os
from launch_ros.actions import Node

def generate_launch_description():
    

    pkg_share = launch_ros.substitutions.FindPackageShare(package='robot2_description').find('robot2_description')
    default_model_path = os.path.join(pkg_share, 'src/description/robot2.urdf.xacro')
    robot_localization_file_path = os.path.join(pkg_share, 'config/ekf.yaml') 
 
    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description':launch_ros.parameter_descriptions.ParameterValue(Command(['xacro ', LaunchConfiguration('model')]), value_type=str)}]
    )

   

    joint_state_publisher_node = launch_ros.actions.Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
    )
      # Start robot localization using an Extended Kalman filter
    robot_localization = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[robot_localization_file_path])


    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(name='model', default_value=default_model_path,
                                            description='Absolute path to robot urdf file'),

        robot_state_publisher_node,
  #      joint_state_publisher_node,
        robot_localization
    ])
