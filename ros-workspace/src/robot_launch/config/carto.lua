
include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "imu_sensor_link",
  published_frame = "base_link",
  publish_to_tf = true,
  odom_frame = "odom",
  provide_odom_frame = true,
  publish_frame_projected_to_2d = false,
  use_odometry = false,
  use_nav_sat = false,
  use_landmarks = false,
  num_laser_scans = 0,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 1,
  lookup_transform_timeout_sec = 1.,
  submap_publish_period_sec = 1,
  pose_publish_period_sec = 5e-3,
--  pose_publish_period_sec = 1,
  trajectory_publish_period_sec = 30e-3,
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 0.5,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,
}

MAP_BUILDER.num_background_threads = 4
MAP_BUILDER.use_trajectory_builder_2d = true
TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 10
TRAJECTORY_BUILDER_2D.use_imu_data = false

POSE_GRAPH.optimize_every_n_nodes = 0


POSE_GRAPH.optimize_every_n_nodes = 100 -- Decrease
MAP_BUILDER.num_background_threads = 4 -- Increase up to number of cores
POSE_GRAPH.global_sampling_ratio = 0.003 -- Decrease
POSE_GRAPH.constraint_builder.sampling_ratio = 0.4 -- Decrease
POSE_GRAPH.constraint_builder.min_score = 0.85 -- Increase
POSE_GRAPH.global_constraint_search_after_n_seconds = 30 -- Increase
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.ceres_solver_options.max_num_iterations = 5 -- Decrease

---------Global/Local SLAM---------
TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.min_num_points = 100 -- Decrease
TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.max_range = 10. -- Decrease
TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.max_length = 1.0 -- Increase
TRAJECTORY_BUILDER_2D.loop_closure_adaptive_voxel_filter.min_num_points = 50 -- Decrease
TRAJECTORY_BUILDER_2D.loop_closure_adaptive_voxel_filter.max_range = 10. -- Decrease
TRAJECTORY_BUILDER_2D.loop_closure_adaptive_voxel_filter.max_length = 1.8 -- Increase
TRAJECTORY_BUILDER_2D.voxel_filter_size = 0.05 -- Increase
-- TRAJECTORY_BUILDER_2D.submaps.resolution=0.05 -- Increase
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 100 -- Decrease
TRAJECTORY_BUILDER_2D.max_range = 10. -- Decrease
return options
