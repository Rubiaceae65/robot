# Requirements:
#   A realsense D400 series
#   Install realsense2 ros2 package (refactor branch)
# Example:
#   $ ros2 launch realsense2_camera rs_launch.py align_depth:=true
#
#   $ ros2 launch rtabmap_ros realsense_d400.launch.py
#   OR
#   $ ros2 launch rtabmap_ros rtabmap.launch.py frame_id:=camera_link args:="-d" rgb_topic:=/camera/color/image_raw depth_topic:=/camera/aligned_depth_to_color/image_raw camera_info_topic:=/camera/color/camera_info approx_sync:=false

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ComposableNode
from launch_ros import actions



#rtabmap_ros components
#rtabmap_ros::RGBDOdometry
#rtabmap_ros::StereoOdometry
#rtabmap_ros::ICPOdometry
#rtabmap_ros::RGBDRelay
#      rtabmap_ros::RGBDSync
#        rtabmap_ros::StereoSync
#          rtabmap_ros::PointCloudXYZ
#            rtabmap_ros::PointCloudXYZRGB
#              rtabmap_ros::PointCloudToDepthImage
#                rtabmap_ros::ObstaclesDetection
#                  rtabmap_ros::PointCloudAggregator
#                    rtabmap_ros::PointCloudAssembler
#                      rtabmap_ros::CoreWrapper

parameters=[{
          'frame_id':'ps5eye_left',
          'subscribe_depth':False,
          'approx_sync':True,
          'subscribe_stereo': True,
          'subscribe_depth': False,
          'subscribe_rgb': False,
          "decimation": 4,
          "voxel_size": 0.0,
          "queue_size": 100,

          #"queue_size": LaunchConfiguration('queue_size'),
          #"qos": LaunchConfiguration('qos_image'),
          #"qos_camera_info": LaunchConfiguration('qos_camera_info')
          }]

remappings=[
      #('rgb/image', '/kinect/left/image_raw'),
      #('rgb/camera_info', '/kinect/left/camera_info'),
      #('depth/image', '/camera/aligned_depth_to_color/image_raw')
      ("left/image_rect", '/ps5eye/left/image_rect'),
      ("left/camera_info", '/ps5eye/left/camera_info'),
      ("right/image_rect", '/ps5eye/right/image_rect'),
      ("right/camera_info", '/ps5eye/right/camera_info'),
      #("odom", '/odometry/filtered'),
      ("imu", '/imu/data'),
      ("gps/fix", '/gps2/fix'),
      ]



composable_nodes = actions.ComposableNodeContainer(
        name="rtabmap_container",
        package='rclcpp_components',
	namespace='rtabmap',
        executable='component_container',

        composable_node_descriptions=[
 
        ComposableNode(
            package='rtabmap_ros',
            plugin='rtabmap_ros::StereoSync',
            namespace='rtabmap',
            name='stereo_sync',
            parameters=parameters,
            remappings=remappings,
        ),

        ComposableNode(
            package='rtabmap_ros',
            plugin='rtabmap_ros::StereoOdometry',
            namespace='rtabmap',
            name='stereo_odometry',
            parameters=parameters,
            remappings=remappings,
        ),
        ComposableNode(
            package='rtabmap_ros', 
            plugin='rtabmap_ros::CoreWrapper',
            name='rtab_core',
            namespace='rtabmap',
            parameters=parameters,
            remappings=remappings,
        ),
        ComposableNode(
            package='rtabmap_ros', 
            plugin='rtabmap_ros::PointCloudXYZRGB',
            name='rtab_pointcloud',
            namespace='rtabmap',
            parameters=parameters,
            remappings=remappings,

            
        ),
        ]
        )
ld = LaunchDescription([
    Node(
        package='rtabmap_ros', 
        executable='rtabmapviz', 
        name='rtabmapvizw',
        output='screen',
        namespace='rtabmap',
        parameters=parameters,
        remappings=remappings),

])



def generate_launch_description():

    ld.add_action(composable_nodes)
    return ld
