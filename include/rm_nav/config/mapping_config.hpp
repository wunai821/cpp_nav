#pragma once

#include <string>

namespace rm_nav::config {

struct MappingConfig {
  bool enabled{false};
  int loop_hz{10};
  std::string output_dir{"../maps/warmup/active"};
  std::string waypoint_path{"warmup_waypoints.txt"};
  double route_speed_mps{0.7};
  double route_reach_tolerance_m{0.35};
  double voxel_size_m{0.10};
  int compression_interval_frames{5};
  double z_min_m{0.2};
  double z_max_m{1.2};
  double occupancy_resolution_m{0.1};
  double occupancy_padding_m{1.0};
  double synthetic_scan_radius_m{8.0};
  int synthetic_points_per_frame{480};
};

}  // namespace rm_nav::config
