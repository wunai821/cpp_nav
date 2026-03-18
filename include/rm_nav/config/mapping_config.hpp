#pragma once

#include <string>

namespace rm_nav::config {

struct MappingConfig {
  bool enabled{false};
  int loop_hz{10};
  std::string output_dir{"../maps/warmup/active"};
  std::string staging_dir{};
  std::string last_good_dir{};
  std::string failed_dir{};
  std::string waypoint_path{"warmup_waypoints.txt"};
  double route_speed_mps{0.7};
  double route_reach_tolerance_m{0.35};
  double voxel_size_m{0.10};
  int compression_interval_frames{5};
  double z_min_m{0.2};
  double z_max_m{1.2};
  double occupancy_resolution_m{0.1};
  double occupancy_padding_m{1.0};
  double occupancy_hit_log_odds{0.85};
  double occupancy_miss_log_odds{0.40};
  double occupancy_min_log_odds{-2.0};
  double occupancy_max_log_odds{3.5};
  double occupancy_free_threshold_log_odds{-0.2};
  double occupancy_occupied_threshold_log_odds{0.6};
  double occupancy_inflation_radius_m{0.15};
  double synthetic_scan_radius_m{8.0};
  int synthetic_points_per_frame{480};
  bool dynamic_suppression_enabled{true};
  double dynamic_near_field_radius_m{1.0};
  int dynamic_consistency_frames{2};
  int dynamic_pending_ttl_frames{3};
  bool dynamic_known_obstacle_mask_enabled{true};
  double dynamic_known_obstacle_margin_m{0.25};
  double dynamic_known_obstacle_min_confidence{0.35};
  std::string pose_source{"odom"};
  int frontend_match_max_iterations{10};
  double frontend_correspondence_distance_m{0.75};
  double frontend_min_match_score{0.2};
  int frontend_match_max_points{160};
  int frontend_min_map_points{80};
  double frontend_submap_radius_m{2.0};
  int frontend_submap_max_points{320};
  double keyframe_translation_threshold_m{0.5};
  double keyframe_yaw_threshold_rad{0.2};
  int max_keyframes{64};
  int keyframe_max_points{160};
  double loop_candidate_distance_threshold_m{1.0};
  double loop_candidate_min_time_separation_s{5.0};
  double loop_candidate_max_yaw_delta_rad{0.6};
  std::string loop_matcher{"icp"};
  int loop_match_max_iterations{8};
  double loop_match_correspondence_distance_m{0.75};
  double loop_match_min_score{0.55};
  int loop_match_max_points{160};
  double loop_correction_min_score{0.6};
  double loop_correction_max_translation_m{0.25};
  double loop_correction_max_yaw_rad{0.15};
  int loop_correction_max_consecutive_failures{3};
  double loop_correction_apply_translation_step_m{0.05};
  double loop_correction_apply_yaw_step_rad{0.03};
  int validation_min_global_points{200};
  int validation_min_occupied_cells{20};
  int validation_min_width{8};
  int validation_min_height{8};
  double validation_min_occupied_ratio{0.001};
  double validation_max_loop_translation_regression_m{0.05};
  double validation_max_loop_yaw_regression_rad{0.03};
};

}  // namespace rm_nav::config
