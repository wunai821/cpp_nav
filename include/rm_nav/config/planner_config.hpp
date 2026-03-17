#pragma once

namespace rm_nav::config {

struct PlannerConfig {
  int loop_hz{20};
  double supply_start_x_m{1.2};
  double supply_start_y_m{2.0};
  double center_goal_x_m{6.0};
  double center_goal_y_m{3.0};
  double center_radius_m{0.6};
  double yaw_align_tolerance_rad{0.12};
  double recenter_threshold_m{0.9};
  double max_vx_mps{1.0};
  double max_vy_mps{0.8};
  double max_wz_radps{1.0};
  double hold_max_v_mps{0.25};
  double hold_max_wz_radps{0.35};
  int dwa_linear_samples{5};
  int dwa_lateral_samples{5};
  int dwa_yaw_samples{5};
  double dwa_horizon_s{1.0};
  double dwa_dt_s{0.2};
  double weight_heading{2.0};
  double weight_clearance{5.0};
  double weight_velocity{0.2};
  double weight_dynamic{8.0};
  double dynamic_inflation_m{0.2};
  double dynamic_influence_distance_m{1.6};
  double dynamic_emergency_distance_m{0.2};
};

}  // namespace rm_nav::config
