#pragma once

#include "rm_nav/common/status.hpp"
#include "rm_nav/data/grid_map.hpp"

namespace rm_nav::perception {

struct LocalCostmapConfig;

class InflationLayer {
 public:
  common::Status Configure(const LocalCostmapConfig& config);
  common::Status Apply(data::GridMap2D* costmap) const;

 private:
  float inflation_radius_m_{0.30F};
  bool configured_{false};
};

}  // namespace rm_nav::perception
