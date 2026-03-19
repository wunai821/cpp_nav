#pragma once

#include <cstdint>

#include "rm_nav/common/types.hpp"
#include "rm_nav/data/pose.hpp"

namespace rm_nav::data {

enum class DynamicObstacleRiskLevel : std::uint8_t {
  kLow = 0,
  kMedium = 1,
  kHigh = 2,
};

struct DynamicObstacle {
  common::ObjectId id{0};
  Pose3f pose{};
  common::Vec3f velocity{};
  Pose3f predicted_pose_05s{};
  Pose3f predicted_pose_10s{};
  float radius_m{0.0F};
  float predicted_radius_m{0.0F};
  float confidence{0.0F};
  float risk_score{0.0F};
  std::uint32_t age{0};
  std::uint32_t missed_frames{0};
  bool is_confirmed{false};
  DynamicObstacleRiskLevel risk_level{DynamicObstacleRiskLevel::kLow};
};

}  // namespace rm_nav::data
