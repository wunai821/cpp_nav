#pragma once

#include <vector>

#include "rm_nav/common/status.hpp"
#include "rm_nav/data/dynamic_obstacle.hpp"
#include "rm_nav/data/grid_map.hpp"
#include "rm_nav/data/pose.hpp"

namespace rm_nav::perception {

struct LocalCostmapConfig;

class DynamicLayer {
 public:
  common::Status Configure(const LocalCostmapConfig& config);
  common::Status Apply(const data::Pose3f& pose,
                       const std::vector<data::DynamicObstacle>& obstacles,
                       const data::GridMap2D& reference_grid,
                       data::GridMap2D* dynamic_layer) const;

 private:
  float dynamic_obstacle_inflation_m_{0.45F};
  bool configured_{false};
};

}  // namespace rm_nav::perception
