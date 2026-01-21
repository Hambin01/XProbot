-- Copyright 2016 The Cartographer Authors
--
-- Licensed under the Apache License, Version 2.0 (the "License");
-- you may not use this file except in compliance with the License.
-- You may obtain a copy of the License at
--
--      http://www.apache.org/licenses/LICENSE-2.0
--
-- Unless required by applicable law or agreed to in writing, software
-- distributed under the License is distributed on an "AS IS" BASIS,
-- WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
-- See the License for the specific language governing permissions and
-- limitations under the License.

include "xiaoma_map_builder_fast.lua"
include "xiaoma_trajectory_builder_fast.lua"

options = {

  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,

  num_subdivisions_per_laser_scan = 1,                   -- Crucial: scan data batch size

  -- Keep to default adjust
  num_laser_scans = 1,
  num_multi_echo_laser_scans = 0,
  num_point_clouds = 0,

  use_pose_extrapolator = false,                         -- Inference pose from imu, odom
  use_odometry = false,
  use_nav_sat = false,
  use_landmarks = false,

  map_frame = "map",
  tracking_frame = "base_link",
  published_frame = "base_link",
  odom_frame = "odom",

  publish_frame_projected_to_2d = false,                 -- TODO:
  provide_odom_frame = true,                            -- TODO: default: true

  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,
  lookup_transform_timeout_sec = 0.2,                    -- TODO: Meaning

  rangefinder_sampling_ratio = 0.5,
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,

}

MAP_BUILDER.use_trajectory_builder_2d = true
TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 1

return options
