#pragma once

#include <cstdint>

#include "rm_nav/common/time.hpp"
#include "rm_nav/data/pose.hpp"

namespace rm_nav::mapping {

struct MappingTrajectorySample {
  common::TimePoint stamp{};
  std::uint32_t frame_index{0};
  data::Pose3f external_pose{};
  data::Pose3f predicted_pose{};
  data::Pose3f optimized_pose{};
  bool loop_consistency_evaluated{false};
  bool loop_candidate_found{false};
  bool loop_match_converged{false};
  bool loop_correction_accepted{false};
  float raw_loop_translation_error_m{0.0F};
  float raw_loop_yaw_error_rad{0.0F};
  float optimized_loop_translation_error_m{0.0F};
  float optimized_loop_yaw_error_rad{0.0F};
};

}  // namespace rm_nav::mapping
