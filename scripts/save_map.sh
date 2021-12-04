
#https://github.com/marinaKollmitz/gazebo_ros_2Dmap_plugin

rosservice call /gazebo_2Dmap_plugin/generate_map

rosrun map_server map_saver -f <mapname> /map:=/map2d

