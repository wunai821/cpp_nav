#pragma once

#include <cstddef>
#include <cstdint>
#include <string>

#include "rm_nav/common/status.hpp"
#include "rm_nav/config/mapping_config.hpp"
#include "rm_nav/data/pose.hpp"
#include "rm_nav/mapping/loop_candidate_detector.hpp"
#include "rm_nav/mapping/loop_closure_matcher.hpp"

namespace rm_nav::mapping {

enum class LoopCorrectionDecisionReason {
  kNone = 0,
  kNoCandidate,
  kMatcherFailed,
  kNotConverged,
  kScoreTooLow,
  kTranslationTooLarge,
  kYawTooLarge,
  kTooManyRecentFailures,
  kAccepted,
};

struct LoopCorrectionDecision {
  bool evaluated{false};
  bool accepted{false};
  bool candidate_found{false};
  std::size_t keyframe_index{0};
  std::uint32_t frame_index{0};
  float score{0.0F};
  int iterations{0};
  float translation_correction_m{0.0F};
  float yaw_correction_rad{0.0F};
  int consecutive_failures_before{0};
  int consecutive_failures_after{0};
  LoopCorrectionDecisionReason reason{LoopCorrectionDecisionReason::kNone};
  std::string reason_message{"not evaluated"};
  data::Pose3f accepted_pose_candidate_frame{};
};

class LoopCorrectionGate {
 public:
  common::Status Evaluate(const config::MappingConfig& config,
                          const LoopClosureCandidate& candidate,
                          const LoopClosureMatchResult& match,
                          int consecutive_failures_before,
                          LoopCorrectionDecision* decision) const;
};

}  // namespace rm_nav::mapping
