#pragma once

#include <array>
#include <vector>

#include "rm_nav/common/status.hpp"
#include "rm_nav/common/types.hpp"
#include "rm_nav/data/lidar_frame.hpp"

namespace rm_nav::perception {

struct PreprocessConfig;

class RangeCropFilter {
 public:
  common::Status Configure(const PreprocessConfig& config);
  common::Status Apply(const std::vector<data::PointXYZI>& input,
                       std::vector<data::PointXYZI>* output) const;

 private:
  bool PointInSelfMask(float x, float y) const;

  float min_range_m_{0.2F};
  float max_range_m_{8.0F};
  float blind_zone_radius_m_{0.25F};
  float min_height_m_{-0.5F};
  float max_height_m_{1.5F};
  bool self_mask_enabled_{false};
  std::array<common::Vec2f, 4> self_mask_polygon_{};
  bool self_mask_polygon_valid_{false};
  float self_mask_x_min_m_{-0.3F};
  float self_mask_x_max_m_{0.3F};
  float self_mask_y_min_m_{-0.25F};
  float self_mask_y_max_m_{0.25F};
  bool configured_{false};
};

}  // namespace rm_nav::perception
