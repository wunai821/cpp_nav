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
  int high_load_debug_threshold_ms{8};
  bool drop_pointcloud_on_high_load{true};
};

}  // namespace rm_nav::config
