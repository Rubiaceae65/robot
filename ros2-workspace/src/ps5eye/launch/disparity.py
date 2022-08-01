# Copyright (c) 2020, Open Source Robotics Foundation, Inc.
# All rights reserved.
#
# Software License Agreement (BSD License 2.0)
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of the copyright holder nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

# docs: https://docs.ros.org/en/foxy/Tutorials/Intermediate/Composition.html
# http://wiki.ros.org/stereo_image_proc/Tutorials/ChoosingGoodStereoParameters
'''
,
        actions.Node(
	  package='image_view',
	  executable='disparity_view',
	  name='ps5eye_disparity_view',
	  
          namespace='ps5eye',
	  parameters=[
	     {"exposure_auto": 0},
	  ],
	  
          remappings=[
	    ('image', 'disparity'),
	  ]

	),
        
	actions.Node(
	  package='image_proc',
	  executable='image_proc',
	  node_name='huhps5eye_left_image_proc',
	  namespace='ps5eye',
          remappings=[
	    ('image', 'left/image_raw'),
	    ('image_rect', 'left/image_rect'),
            ('image_mono', 'left/image_mono'),
            ('image_color', 'left/image_color'),



	    ('camera_info', 'left/camera_info'),
            ]
	),
        actions.Node(
	  package='image_proc',
	  executable='image_proc',
	  node_name='wtfps5eye_right_image_proc',
	  namespace='ps5eye',
          remappings=[
	    ('image', 'right/image_raw'),
	    ('image_rect', 'right/image_rect'),
	    ('camera_info', 'right/camera_info'),
            ]
	),



'''
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros import actions

from launch_ros.descriptions import ComposableNode
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
ns='test'
def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

#    urdf_file_name = 'stereo_camera.urdf.xacro'
#    urdf = os.path.join( get_package_share_directory('ps5eye'), urdf_file_name)
    urdf = "/home/user/src/robot2/ros2-workspace/src/ps5eye/urdf/test.urdf"
    with open(urdf, 'r') as infp:
        robot_desc = infp.read()


    image_processing = actions.ComposableNodeContainer(
        name="image_proc_container",
        package='rclcpp_components',
	namespace='ps5eye',
        executable='component_container',

        composable_node_descriptions=[
            ComposableNode(
                package='image_proc',
                node_plugin='image_proc::RectifyNode',
                namespace='ps5eye',
                node_name='rectify_left',
                remappings=[
                    ('image', 'left/image_raw'),
                    ('camera_info', 'left/camera_info'),
                    ('/ps5eye/camera_info', '/ps5eye/left/camera_info'),
                    ('image_rect', 'left/image_rect')
                ],
            ),
            ComposableNode(
                package='image_proc',
                node_plugin='image_proc::RectifyNode',
                namespace='ps5eye',
                node_name='rectify_right',
                remappings=[
                    ('image', 'right/image_raw'),
                    ('camera_info', 'right/camera_info'),
                    ('/ps5eye/camera_info', '/ps5eye/right/camera_info'),
                    ('image_rect', 'right/image_rect')
                ],
            ),
            ComposableNode(
                package='stereo_image_proc',
                node_plugin='stereo_image_proc::DisparityNode',
                namespace='ps5eye',
                node_name='disparity',
		parameters=[
		 {"approximate_sync": True},
	  	]


            ),
            ComposableNode(
                package='stereo_image_proc',
                node_plugin='stereo_image_proc::PointCloudNode',
                namespace='ps5eye',
                node_name='pointcloud',
                remappings=[
                    ('left/image_rect_color', 'left/image_rect'),
                ],
 		parameters=[
		 {"approximate_sync": True},
	  	]


            ),
            ComposableNode(
                #min_height (double, default: 2.2e-308) - The minimum height to sample in the point cloud in meters.
                #max_height (double, default: 1.8e+308) - The maximum height to sample in the point cloud in meters.
                #angle_min (double, default: -π) - The minimum scan angle in radians.
                #angle_max (double, default: π) - The maximum scan angle in radians.
                #angle_increment (double, default: π/180) - Resolution of laser scan in radians per ray.
                #queue_size (double, default: detected number of cores) - Input point cloud queue size.
                #scan_time (double, default: 1.0/30.0) - The scan rate in seconds. Only used to populate the scan_time field of the output laser scan message.
                #range_min (double, default: 0.0) - The minimum ranges to return in meters.
                #range_max (double, default: 1.8e+308) - The maximum ranges to return in meters.
                #target_frame (str, default: none) - If provided, transform the pointcloud into this frame before converting to a laser scan. Otherwise, laser scan will be generated in the same frame as the input point cloud.
                #transform_tolerance (double, default: 0.01) - Time tolerance for transform lookups. Only used if a target_frame is provided.
                #use_inf (boolean, default: true) - If disabled, report infinite range (no obstacle) as range_max + 1. Otherwise report infinite range as +inf.
                package='pointcloud_to_laserscan',
                node_plugin='pointcloud_to_laserscan::PointcloudToLaserScanNode',
                namespace='ps5eye',
                node_name='pointcloud_to_laserscan',
                remappings=[
                    ('left/image_rect_color', 'left/image_rect'),
                ],
 		parameters=[
		 {"approximate_sync": True},
	  	]


            ),
 

            ])

    ld = LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        actions.Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time, 'robot_description': robot_desc}],
            arguments=[urdf]
            ), 
        actions.Node(
            #package='ps5eye',
            executable='/home/user/src/robot2/ros2-workspace/src/ps5eye/ps5eye/publish_info.py',
            name='publish_info_ps5eye',
            output='screen'
            ),  
        actions.Node(
            executable='/home/user/src/robot2/ros2-workspace/src/ps5eye/scripts/gstcam.sh',
            name='gstcam',
            output='screen'
            ), 
 
        #actions.Node(
        #    package = "tf2_ros", 
        #    executable = "static_transform_publisher",
        #    arguments = ["0", "0", "0", "0", "0", "0", "odom", "laser"]
        #  )
        ]

    )
    #ld = LaunchDescription()
    ld.add_action(image_processing)


    return ld
