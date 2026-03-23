#pragma once

#include <string>

namespace rm_nav::config {

struct ThreadAffinityConfig {
  int driver_cpu{-1};
  int sync_cpu{-1};
  int pose_core_cpu{-1};
  int perception_cpu{-1};
  int planner_cpu{-1};
  int safety_fsm_cpu{-1};
  int debug_cpu{-1};
};

struct SystemConfig {
  std::string app_name{"rm_nav_main"};
  std::string version{"0.1.0"};
  std::string log_level{"INFO"};
  std::string bringup_mode{"none"};
  int auto_shutdown_ms{1000};
  int manual_mode_selector{-1};
  bool require_referee_start_for_warmup{true};
  bool require_referee_start_for_combat{true};
  ThreadAffinityConfig thread_affinity{};
};

}  // namespace rm_nav::config
