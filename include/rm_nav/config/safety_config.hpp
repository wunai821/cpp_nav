#pragma once

namespace rm_nav::config {

struct SafetyConfig {
  int loop_hz{50};
  double emergency_stop_distance_m{0.5};
  int heartbeat_timeout_ms{100};
  int deadman_timeout_ms{250};
  double collision_check_lookahead_s{0.8};
  double collision_check_dt_s{0.1};
  double recovery_speed_scale{0.35};
  double recovery_yaw_scale{0.5};
};

}  // namespace rm_nav::config
