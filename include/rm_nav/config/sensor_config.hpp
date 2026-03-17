#pragma once

#include <string>

namespace rm_nav::config {

struct RectMaskConfig {
  bool enabled{false};
  float x_min_m{-0.3F};
  float x_max_m{0.3F};
  float y_min_m{-0.25F};
  float y_max_m{0.25F};
};

struct ExtrinsicConfig {
  float x_m{0.0F};
  float y_m{0.0F};
  float z_m{0.0F};
  float roll_rad{0.0F};
  float pitch_rad{0.0F};
  float yaw_rad{0.0F};
};

struct SensorConfig {
  bool lidar_enabled{true};
  std::string lidar_model{"L1"};
  ExtrinsicConfig lidar_mount{0.20F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F};
  RectMaskConfig lidar_self_mask{};
  bool imu_enabled{true};
  std::string imu_model{"GenericIMU"};
  ExtrinsicConfig imu_mount{-0.10F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F};
};

}  // namespace rm_nav::config
