#pragma once

#include "rm_nav/common/double_buffer.hpp"
#include "rm_nav/common/status.hpp"
#include "rm_nav/data/dynamic_obstacle.hpp"
#include "rm_nav/data/grid_map.hpp"
#include "rm_nav/data/lidar_frame.hpp"
#include "rm_nav/data/local_map.hpp"
#include "rm_nav/data/pose.hpp"

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

 private:
  LocalCostmapConfig config_{};
  common::DoubleBuffer<data::LocalCostmap> latest_costmap_{};
};

}  // namespace rm_nav::perception
