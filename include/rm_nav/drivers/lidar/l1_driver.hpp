#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "rm_nav/common/status.hpp"
#include "rm_nav/data/lidar_frame.hpp"

namespace rm_nav::drivers::lidar {

struct L1DriverConfig {
  std::string source{"synthetic"};
  std::string synthetic_map_pcd_path{};
  double frame_rate_hz{10.0};
  std::size_t points_per_frame{360};
  float radius_m{5.0F};
};

class L1Driver {
 public:
  common::Status Configure(const L1DriverConfig& config);
  std::optional<data::LidarFrame> PollFrame();

 private:
  std::vector<data::PointXYZI> synthetic_map_points_{};
  L1DriverConfig config_{};
  common::TimePoint next_frame_time_{};
  std::uint32_t frame_index_{0};
  bool configured_{false};
};

}  // namespace rm_nav::drivers::lidar
