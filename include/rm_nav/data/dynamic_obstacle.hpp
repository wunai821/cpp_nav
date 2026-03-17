#pragma once

#include "rm_nav/common/types.hpp"
#include "rm_nav/data/pose.hpp"

namespace rm_nav::data {

struct DynamicObstacle {
  common::ObjectId id{0};
  Pose3f pose{};
  common::Vec3f velocity{};
  Pose3f predicted_pose_05s{};
  Pose3f predicted_pose_10s{};
  float radius_m{0.0F};
  float confidence{0.0F};
  std::uint32_t age{0};
  std::uint32_t missed_frames{0};
  bool is_confirmed{false};
};

}  // namespace rm_nav::data
