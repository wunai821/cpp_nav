#pragma once

#include "rm_nav/common/double_buffer.hpp"
#include "rm_nav/common/status.hpp"
#include "rm_nav/data/grid_map.hpp"
#include "rm_nav/data/lidar_frame.hpp"
#include "rm_nav/data/local_map.hpp"
#include "rm_nav/data/pose.hpp"

namespace rm_nav::perception {

class LocalCostmapBuilder {
 public:
  common::Status Build(const data::LidarFrame& filtered_frame,
                       const data::Pose3f& pose,
                       data::GridMap2D* costmap);
  common::Status BuildAndPublish(const data::LidarFrame& filtered_frame,
                                 const data::Pose3f& pose);
  data::LocalCostmap LatestCostmap() const { return latest_costmap_.ReadSnapshot(); }

 private:
  common::DoubleBuffer<data::LocalCostmap> latest_costmap_{};
};

}  // namespace rm_nav::perception
