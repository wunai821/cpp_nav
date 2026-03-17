#pragma once

#include "rm_nav/common/status.hpp"
#include "rm_nav/data/grid_map.hpp"
#include "rm_nav/data/path.hpp"
#include "rm_nav/data/pose.hpp"

namespace rm_nav::planning {

class GlobalAStar {
 public:
  common::Status Plan(const data::GridMap2D& static_map,
                      const data::Pose3f& start_pose,
                      const data::Pose3f& goal_pose,
                      data::Path2D* path) const;
};

}  // namespace rm_nav::planning
