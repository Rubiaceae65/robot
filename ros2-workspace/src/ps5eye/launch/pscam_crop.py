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

from launch import LaunchDescription
from launch_ros import actions

from launch_ros.descriptions import ComposableNode

ns='test'
def generate_launch_description():

    image_processing = actions.ComposableNodeContainer(
        name="split_container",
        package='rclcpp_components',
	namespace='ps5eye',
        executable='component_container',

        composable_node_descriptions=[
	  ComposableNode(
                package='image_proc',
		namespace='ps5eye',
                plugin='image_proc::CropDecimateNode',
                name='crop_right',
		parameters=[
		  #{"offset_x": 0},
		  #{"offset_y": 0},
		  #{"width": 1920},
		  #{"height": 1080},
                  {"queue_size": 10},
                  {"camera_name": 'ps5eye_right'},
                  {"camera_info_url": 'file:///home/user/src/robot2/ros2-workspace/src/ps5eye/config/ps5eye_right.yaml'}

		  ],
      		remappings=[
	    	  ('in/image_raw', 'image_raw'),
		  #('in/camera_info', '/null/right/camera_info'),
                  ('in/camera_info', 'camera_info'),

		  ('out/image_raw', 'right/image_raw'),
                  #('out/image_raw/image_topics', 'right/image_raw/image_topics'),
   	          ('out/image_raw/compressed', 'right/image_raw/compressed'),
	          ('out/image_raw/compressedDepth', 'right/image_raw/compressedDepth'),
         	  ('out/image_raw/theora', 'right/image_raw/theora'),


		  ('out/camera_info', 'right/camera_info'),
                ]
            ),

	  ComposableNode(
                package='image_proc',
		namespace='ps5eye',
                plugin='image_proc::CropDecimateNode',
                name='crop_left',
		parameters=[
		  #{"offset_x": 1920},
		  #{"offset_y": 0},
		  #{"width": 1920},
		  #{"height": 1080},
                  {"queue_size": 10},
                  {"camera_name": 'ps5eye_left'},
                  {"camera_info_url": 'file:///home/user/src/robot2/ros2-workspace/src/ps5eye/config/ps5eye_left.yaml'}
		  ],
      		remappings=[
	    	  ('in/image_raw', 'image_raw'),
		  #('in/camera_info', '/null/right/camera_info'),
                  ('in/camera_info', 'camera_info'),


		  ('out/image_raw', 'left/image_raw'),
		  #('out/image_raw/image_topics', 'left/image_raw/image_topics'),
   	          ('out/image_raw/compressed', 'left/image_raw/compressed'),
	          ('out/image_raw/compressedDepth', 'left/image_raw/compressedDepth'),
         	  ('out/image_raw/theora', 'left/image_raw/theora'),

		  ('out/camera_info', 'left/camera_info'),
                ]
            ),
          ],
        output='screen'
    )



    ld = LaunchDescription([
	actions.Node(
	  package='v4l2_camera',
	  executable='v4l2_camera_node',
	  name='ps5eye',
	  namespace='ps5eye',
	  parameters=[
	     {"image_size": [3840,1080] },
	     {"time_per_frame": [1,30]},
	     {"exposure_auto": 0},
	     {"power_line_frequency": 1}
	  ]
	)
        ]
    )
    #ld = LaunchDescription()
    ld.add_action(image_processing)


    return ld
