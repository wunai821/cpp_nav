#pragma once

#include <vector>

#include "rm_nav/common/time.hpp"
#include "rm_nav/data/dynamic_obstacle.hpp"
#include "rm_nav/data/grid_map.hpp"

namespace rm_nav::data {

using LocalCostmap = GridMap2D;

struct DynamicObstacleSet {
  common::TimePoint stamp{};
  std::vector<DynamicObstacle> obstacles{};
};

}  // namespace rm_nav::data
