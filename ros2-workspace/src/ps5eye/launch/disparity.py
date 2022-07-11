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
from launch import LaunchDescription
from launch_ros import actions

from launch_ros.descriptions import ComposableNode

ns='test'
def generate_launch_description():

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
                    ('image_rect', 'right/image_rect')
                ],
            ),
            ComposableNode(
                package='stereo_image_proc',
                node_plugin='stereo_image_proc::DisparityNode',
                namespace='ps5eye',
                node_name='disparity',
            ),
            ComposableNode(
                package='stereo_image_proc',
                node_plugin='stereo_image_proc::PointCloudNode',
                namespace='ps5eye',
                node_name='pointcloud',
            ),
 


            ])

    ld = LaunchDescription([
	actions.Node(
	  package='stereo_image_proc',
	  executable='disparity_node',
	  name='ps5eye_disparity_stuff',
	  namespace='ps5eye',
          #remappings=[
	  #  ('image_raw', 'ps5eye_image'),
	  #  ('camera_info', 'ps5eye_camera_info'),
	  #]

	)
        ]

    )
    ld = LaunchDescription()
    ld.add_action(image_processing)


    return ld
