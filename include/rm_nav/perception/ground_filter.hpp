#pragma once

#include <vector>

#include "rm_nav/common/status.hpp"
#include "rm_nav/data/point_types.hpp"

namespace rm_nav::perception {

struct PreprocessConfig;

class GroundFilter {
 public:
  common::Status Configure(const PreprocessConfig& config);
  common::Status Apply(const std::vector<data::PointXYZI>& input,
                       std::vector<data::PointXYZI>* output) const;

 private:
  float ground_z_max_m_{-0.05F};
  float ground_margin_m_{0.03F};
  bool configured_{false};
};

}  // namespace rm_nav::perception
