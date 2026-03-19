#pragma once

namespace rm_nav::config {

struct SafetyConfig {
  int loop_hz{50};
  double emergency_stop_distance_m{0.5};
  int heartbeat_timeout_ms{100};
  int deadman_timeout_ms{250};
  int costmap_timeout_ms{250};
  int mode_transition_freeze_ms{120};
  double collision_check_lookahead_s{0.8};
  double collision_check_dt_s{0.1};
  int hold_timeout_ms{1200};
  int planner_fail_timeout_ms{1500};
  int localization_fail_timeout_ms{800};
  int mission_timeout_ms{15000};
  double footprint_half_length_m{0.30};
  double footprint_half_width_m{0.25};
  double max_vx_mps{1.0};
  double max_vy_mps{0.8};
  double max_wz_radps{1.0};
  double max_delta_v_per_tick{0.08};
  double max_delta_w_per_tick{0.12};
  double recovery_speed_scale{0.35};
  double recovery_yaw_scale{0.5};
};

}  // namespace rm_nav::config
