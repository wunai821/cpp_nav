#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

#include "rm_nav/common/status.hpp"
#include "rm_nav/config/mapping_config.hpp"
#include "rm_nav/data/pose.hpp"

namespace rm_nav::mapping {

struct MappingKeyframe;

struct LoopClosureCandidate {
  bool found{false};
  std::size_t keyframe_index{0};
  std::uint32_t frame_index{0};
  common::TimePoint stamp{};
  float distance_m{0.0F};
  float yaw_delta_rad{0.0F};
  common::TimeNs time_separation_ns{0};
};

class LoopCandidateDetector {
 public:
  common::Status FindCandidate(const config::MappingConfig& config,
                               const data::Pose3f& current_pose,
                               common::TimePoint current_stamp,
                               std::uint32_t current_frame_index,
                               const std::vector<MappingKeyframe>& keyframes,
                               LoopClosureCandidate* candidate) const;
};

}  // namespace rm_nav::mapping
