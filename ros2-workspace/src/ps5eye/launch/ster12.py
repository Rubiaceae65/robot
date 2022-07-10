# This is an example of a launch file for ROS 2
 
# Import launch modules that are Python-compatible
from launch import LaunchDescription
from launch_ros.actions import Node
 
def generate_launch_description():
  """
    <!-- remap output to left image -->
        <remap from="camera_out/image_raw" to="/stereo/left/image_raw" />
            <!-- Dont use original camera info -->
                <remap from="/stereo/left/camera_info" to="/null/left/camera_info" />

        ('/output/cmd_vel', '/turtlesim2/turtle1/cmd_vel'),
    <param name="x_offset" type="int" value="1920" />
    <param name="y_offset" type="int" value="0" />
    <param name="width" type="int" value="1920" />
    <param name="height" type="int" value="1080" />
 
  """
  return LaunchDescription([
 
    Node(
      package='v4l2_camera',
      executable='v4l2_camera_node',
      name='cam',
      parameters=[
         {"image_size": [3840,1080] },
         {"time_per_frame": [1,30]},
         {"exposure_auto": 0},
         {"power_line_frequency": 1}
      ]
    ),
    Node(
      package="image_proc",
      executable="image_proc",
    #  plugin='image_proc::DebayerNode',

      name="image_left",
      parameters=[
          {"x_offset": 1920},
          {"y_offset": 0},
          {"width": 1920},
          {"height": 1080}
          ],
      remappings=[
          ('camera_out/image_raw', '/stereo/left/image_raw'),
          ('/stereo/left/camera_info', '/null/left/camera_info')
          ]
    ),   
    Node(
      package='image_view',
      executable='image_view',
      name='viewer',
      remappings=[
        ('/image', '/image_raw')
      ]

    )

    


    ])
