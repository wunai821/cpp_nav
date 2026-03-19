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
  int center_hold_settle_frames{5};
  int center_hold_settle_time_ms{500};
  double recenter_threshold_m{0.9};
  double max_vx_mps{1.0};
  double max_vy_mps{0.8};
  double max_wz_radps{1.0};
  double hold_max_v_mps{0.25};
  double hold_max_wz_radps{0.35};
  double center_hold_slot_radius_m{0.45};
  double center_hold_occupied_radius_m{0.40};
  double center_hold_dynamic_bias_radius_m{1.10};
  double center_hold_max_bias_m{0.28};
  double center_hold_position_deadband_m{0.06};
  double center_hold_yaw_deadband_rad{0.05};
  double center_hold_micro_max_v_mps{0.12};
  double center_hold_micro_max_wz_radps{0.18};
  double center_hold_position_kp{0.8};
  double center_hold_yaw_kp{0.9};
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
  double dynamic_high_risk_penalty_scale{1.0};
  double dynamic_crossing_penalty_scale{1.4};
  int recovery_light_escalate_cycles{18};
  int recovery_medium_escalate_cycles{24};
  int recovery_heavy_exhaust_cycles{30};
  int recovery_heavy_restart_ticks{8};
  double recovery_light_rotate_wz_radps{0.28};
  double recovery_light_shift_vy_mps{0.10};
  double recovery_light_forward_vx_mps{0.08};
  double recovery_medium_backoff_vx_mps{0.14};
  double recovery_medium_shift_vy_mps{0.18};
  double recovery_medium_rotate_wz_radps{0.20};
  double recovery_reapproach_forward_vx_mps{0.08};
  double recovery_reapproach_lateral_offset_m{0.45};
  int recovery_reapproach_lookahead_points{4};
  double recovery_clearance_weight_scale{1.5};
  double recovery_slow_restart_linear_scale{0.25};
  double recovery_slow_restart_yaw_scale{0.35};
};

}  // namespace rm_nav::config
