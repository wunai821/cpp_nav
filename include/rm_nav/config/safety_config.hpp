#pragma once

namespace rm_nav::config {

struct SafetyConfig {
  int loop_hz{50};
  double emergency_stop_distance_m{0.5};
  int heartbeat_timeout_ms{100};
};

}  // namespace rm_nav::config
