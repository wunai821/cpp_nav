#pragma once

#include <string>

namespace rm_nav::config {

struct DebugConfig {
  bool websocket_enabled{false};
  std::string websocket_host{"127.0.0.1"};
  int websocket_port{8765};
  int publish_hz{10};
  int pointcloud_publish_hz{5};
  int scalar_publish_hz{20};
  int watchdog_write_hz{1};
  int high_load_debug_threshold_ms{8};
  bool drop_pointcloud_on_high_load{true};
  bool write_snapshots_only_on_change{true};
  int debug_scan_max_points{1600};
};

}  // namespace rm_nav::config
