#pragma once

#include <cstddef>
#include <string>
#include <vector>

#include "rm_nav/common/status.hpp"
#include "rm_nav/data/pose.hpp"

namespace rm_nav::mapping {

class WaypointManager {
 public:
  common::Status Load(const std::string& path);
  void Reset();
  bool empty() const { return waypoints_.empty(); }
  std::size_t waypoint_count() const { return waypoints_.size(); }
  std::size_t current_index() const { return current_index_; }
  data::Pose3f CurrentGoal() const;
  bool AdvanceIfReached(const data::Pose3f& current_pose, float reach_tolerance_m);

 private:
  std::vector<data::Pose3f> waypoints_{};
  std::size_t current_index_{0};
};

}  // namespace rm_nav::mapping
