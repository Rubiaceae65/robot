# This is an example of a launch file for ROS 2
 
# Import launch modules that are Python-compatible
from launch import LaunchDescription
from launch_ros.actions import Node
 
def generate_launch_description():
  """
  This file is the launch description. It will launch three nodes:
  turtlesim1: A turtle
  turtlesim2: Another turtle
  mimic: Causes one turtle to mimic the movements of the other turtle
  """
  return LaunchDescription([
   
    # Launches a window with a turtle in it
    Node(
      package='turtlesim',
      namespace='turtlesim1',
      executable='turtlesim_node',
      name='sim'
    ),
    # Launches another window with a turtle in it
    Node(
      package='turtlesim',
      namespace='turtlesim2',
      executable='turtlesim_node',
      name='sim'
    ),
    # The mimic executable contains code that causes turtlesim2 to mimic
    # turtlesim1
    Node(
      package='turtlesim',
      executable='mimic',
      name='mimic',
      remappings=[
        ('/input/pose', '/turtlesim1/turtle1/pose'),
        ('/output/cmd_vel', '/turtlesim2/turtle1/cmd_vel'),
      ]
    )
  ])
