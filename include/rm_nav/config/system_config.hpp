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
  std::string log_file_path{"logs/rm_nav.log"};
  bool console_io_only{false};
  int log_max_queue_size{4096};
  bool match_mode_enabled{false};
  int match_low_hp_threshold{100};
  int match_spawn_wait_ms{5000};
  std::string bringup_mode{"none"};
  int auto_shutdown_ms{1000};
  int manual_mode_selector{-1};
  ThreadAffinityConfig thread_affinity{};
};

}  // namespace rm_nav::config
