include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "base_link",
  -- tracking_frame = "imu_link",
  published_frame = "odom",
  odom_frame = "odom",
  provide_odom_frame = false,
  -- provide_odom_frame = true,   error
  publish_frame_projected_to_2d = true,
  use_odometry = true,
  use_nav_sat = false,
  -- use_landmarks = false,
  use_landmarks = true,
  num_laser_scans = 1,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 0,
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  -- submap_publish_period_sec = 0.1,
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,
}

MAP_BUILDER.use_trajectory_builder_2d = true

-- Bandpass filter to filter out long range and close range range data due to noise

TRAJECTORY_BUILDER_2D.min_range = 0.12
TRAJECTORY_BUILDER_2D.max_range = 3.5

-- Cartographer replaces ranges further than max_range by TRAJECTORY_BUILDER_2D.missing_data_ray_length

-- TRAJECTORY_BUILDER_2D.max_range = 40
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 3.0

-- Using IMU data useful for slam

-- TRAJECTORY_BUILDER_2D.use_imu_data = false
TRAJECTORY_BUILDER_2D.use_imu_data = true

TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true 
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(0.1)

POSE_GRAPH.constraint_builder.min_score = 0.65
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.7


-- The optimization is run in batches once a certain number of trajectory nodes was inserted. 
-- Depending on how frequently you need to run it, you can tune the size of these batches.
-- putting POSE_GRAPH.optimize_every_n_nodes = 0 will disable global map

POSE_GRAPH.optimize_every_n_nodes = 0
-- POSE_GRAPH.optimize_every_n_nodes = 0.8


return options