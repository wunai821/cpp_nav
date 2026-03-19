#pragma once

#include "rm_nav/common/status.hpp"
#include "rm_nav/data/grid_map.hpp"
#include "rm_nav/data/lidar_frame.hpp"

namespace rm_nav::perception {

struct LocalCostmapConfig;

class ObstacleLayer {
 public:
  common::Status Configure(const LocalCostmapConfig& config);
  common::Status Apply(const data::LidarFrame& filtered_frame,
                       data::GridMap2D* costmap) const;

 private:
  float obstacle_layer_height_m_{0.05F};
  bool configured_{false};
};

}  // namespace rm_nav::perception
