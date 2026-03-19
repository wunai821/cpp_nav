#pragma once

#include "rm_nav/common/double_buffer.hpp"
#include "rm_nav/common/status.hpp"
#include "rm_nav/data/dynamic_obstacle.hpp"
#include "rm_nav/data/grid_map.hpp"
#include "rm_nav/data/lidar_frame.hpp"
#include "rm_nav/data/local_map.hpp"
#include "rm_nav/data/pose.hpp"
#include "rm_nav/perception/dynamic_layer.hpp"
#include "rm_nav/perception/inflation_layer.hpp"
#include "rm_nav/perception/obstacle_layer.hpp"

namespace rm_nav::perception {

struct LocalCostmapConfig {
  std::uint32_t width{80};
  std::uint32_t height{80};
  float resolution_m{0.10F};
  float obstacle_layer_height_m{0.05F};
  float inflation_radius_m{0.30F};
  float dynamic_obstacle_inflation_m{0.45F};
};

class LocalCostmapBuilder {
 public:
  common::Status Configure(const LocalCostmapConfig& config);
  common::Status Build(const data::LidarFrame& filtered_frame,
                       const data::Pose3f& pose,
                       data::GridMap2D* costmap);
  common::Status Build(const data::LidarFrame& filtered_frame,
                       const data::Pose3f& pose,
                       const std::vector<data::DynamicObstacle>& obstacles,
                       data::GridMap2D* costmap);
  common::Status BuildAndPublish(const data::LidarFrame& filtered_frame,
                                 const data::Pose3f& pose);
  common::Status BuildAndPublish(const data::LidarFrame& filtered_frame,
                                 const data::Pose3f& pose,
                                 const std::vector<data::DynamicObstacle>& obstacles);
  data::LocalCostmap LatestCostmap() const { return latest_costmap_.ReadSnapshot(); }
  data::GridMap2D LatestStaticLayer() const { return latest_static_layer_.ReadSnapshot(); }
  data::GridMap2D LatestDynamicLayer() const { return latest_dynamic_layer_.ReadSnapshot(); }

 private:
  ObstacleLayer obstacle_layer_{};
  InflationLayer inflation_layer_{};
  DynamicLayer dynamic_layer_{};
  LocalCostmapConfig config_{};
  common::DoubleBuffer<data::LocalCostmap> latest_costmap_{};
  common::DoubleBuffer<data::GridMap2D> latest_static_layer_{};
  common::DoubleBuffer<data::GridMap2D> latest_dynamic_layer_{};
  bool configured_{false};
};

}  // namespace rm_nav::perception
