#pragma once

#include <vector>

#include "rm_nav/common/status.hpp"
#include "rm_nav/data/chassis_cmd.hpp"
#include "rm_nav/data/dynamic_obstacle.hpp"
#include "rm_nav/data/grid_map.hpp"
#include "rm_nav/data/pose.hpp"
#include "rm_nav/data/safety_event.hpp"

namespace rm_nav::safety {

class SafetyManager {
 public:
  virtual ~SafetyManager() = default;

  virtual common::Status Evaluate(
      const data::Pose3f& current_pose, const data::GridMap2D& costmap,
      const std::vector<data::DynamicObstacle>& obstacles,
      const data::ChassisCmd& proposed_cmd, data::ChassisCmd* gated_cmd,
      std::vector<data::SafetyEvent>* events) = 0;
};

}  // namespace rm_nav::safety
