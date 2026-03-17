#pragma once

#include <string>

namespace rm_nav::config {

struct SensorConfig {
  bool lidar_enabled{true};
  std::string lidar_model{"L1"};
  bool imu_enabled{true};
  std::string imu_model{"GenericIMU"};
};

}  // namespace rm_nav::config
