#pragma once

#include <string>

namespace rm_nav::config {

struct FramesConfig {
  std::string map{"map"};
  std::string odom{"odom"};
  std::string base_link{"base_link"};
  std::string laser_link{"laser_link"};
  std::string imu_link{"imu_link"};
};

}  // namespace rm_nav::config
