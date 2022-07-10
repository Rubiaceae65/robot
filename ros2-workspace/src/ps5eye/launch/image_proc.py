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

    # Load composable container
image_processing = actions.ComposableNodeContainer(
        name="image_proc_container",
        package='rclcpp_components',
	namespace='testwtf',
        executable='component_container',

        composable_node_descriptions=[
	  ComposableNode(
	        package='v4l2_camera',
		namespace='ps5eye',
		plugin='v4l2_camera::V4L2Camera',
		name='ps5eye_v4l',
		parameters=[
		 {"image_size": [3840,1080] },
		 {"time_per_frame": [1,30]},
		 {"exposure_auto": 0},
		 {"power_line_frequency": 1}
	  	]

          ),		
	  ComposableNode(
                package='image_proc',
		namespace='right',
                plugin='image_proc::CropDecimateNode',
                name='crop_right',
		parameters=[
		  {"x_offset": 0},
		  {"y_offset": 0},
		  {"width": 1920},
		  {"height": 1080},
		  {'decimation_x': 4, 'decimation_y': 4}
		  ],
      		remappings=[
	    	  ('in/image_raw', '/ps5eye/image_raw'),
		  ('in/camera_info', '/ps5eye/camera_info'),
		  #('out/image_raw', 'wtfimage'),
		  #('out/camera_info', 'wtfinfo'),
                ]
            ),

	  ComposableNode(
                package='image_proc',
		namespace='left',
                plugin='image_proc::CropDecimateNode',
                name='crop_left',
		parameters=[
		  {"x_offset": 1920},
		  {"y_offset": 0},
		  {"width": 1920},
		  {"height": 1080},
		  {'decimation_x': 4, 'decimation_y': 4}
		  ],
      		remappings=[
	    	  ('in/image_raw', '/ps5eye/image_raw'),
		  ('in/camera_info', '/ps5eye/camera_info'),
		  #('out/image_raw', 'wtfimage'),
		  #('out/camera_info', 'wtfinfo'),
                ]
            ),
          ],
        output='screen'
    )

def generate_launch_description():
    '''
    ld = LaunchDescription([
	actions.Node(
	  package='v4l2_camera',
	  executable='v4l2_camera_node',
	  name='cam',
	  namespace=ns,
	  parameters=[
	     {"image_size": [3840,1080] },
	     {"time_per_frame": [1,30]},
	     {"exposure_auto": 0},
	     {"power_line_frequency": 1}
	  ],
	  remappings=[
	    ('image_raw', 'ps5eye_image'),
	    ('camera_info', 'ps5eye_camera_info'),
	  ]


	)]
    )
    '''
    ld = LaunchDescription()
    ld.add_action(image_processing)


    return ld
