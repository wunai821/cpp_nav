#pragma once

#include <string>

namespace rm_nav::config {

struct LocalizationConfig {
  bool enabled{true};
  std::string occupancy_path{"../maps/combat/active/occupancy.bin"};
  std::string global_map_pcd_path{"../maps/combat/active/global_map.pcd"};
  std::string map_meta_path{"../maps/combat/active/map_meta.json"};
  std::string matcher{"icp"};
  int max_iterations{8};
  double correspondence_distance_m{0.75};
  double min_match_score{0.55};
  double max_position_jump_m{1.0};
  double max_yaw_jump_rad{0.7};
  int map_downsample_step{1};
  int slow_match_threshold_ms{12};
  int reduced_max_iterations{4};
  int reduced_scan_stride{2};
  int relocalization_failure_threshold{4};
  int relocalization_retry_interval_ms{400};
  double relocalization_submap_radius_m{6.0};
  int relocalization_submap_max_points{640};
  int relocalization_max_iterations{14};
  double relocalization_min_match_score{0.35};
  double relocalization_linear_search_step_m{1.5};
  double relocalization_yaw_search_step_rad{0.35};
  double map_to_odom_guard_translation_m{0.8};
  double map_to_odom_guard_yaw_rad{0.45};
};

}  // namespace rm_nav::config
