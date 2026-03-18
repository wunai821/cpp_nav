#pragma once

#include <cstddef>
#include <cstdint>
#include <string>

#include "rm_nav/common/status.hpp"
#include "rm_nav/config/mapping_config.hpp"
#include "rm_nav/data/pose.hpp"
#include "rm_nav/data/synced_frame.hpp"
#include "rm_nav/localization/icp_matcher.hpp"
#include "rm_nav/localization/ndt_matcher.hpp"
#include "rm_nav/mapping/loop_candidate_detector.hpp"

namespace rm_nav::mapping {

struct MappingKeyframe;

struct LoopClosureMatchResult {
  bool attempted{false};
  bool candidate_found{false};
  bool converged{false};
  std::size_t keyframe_index{0};
  std::uint32_t frame_index{0};
  float score{0.0F};
  int iterations{0};
  float translation_correction_m{0.0F};
  float yaw_correction_rad{0.0F};
  common::TimeNs processing_latency_ns{0};
  data::Pose3f initial_guess_candidate_frame{};
  data::Pose3f matched_pose_candidate_frame{};
  common::StatusCode status_code{common::StatusCode::kNotReady};
  std::string status_message{"no loop match attempted"};
};

class LoopClosureMatcher {
 public:
  common::Status Configure(const config::MappingConfig& config);
  common::Status Match(const data::SyncedFrame& frame, const data::Pose3f& current_map_to_base,
                       const MappingKeyframe& keyframe,
                       const LoopClosureCandidate& candidate,
                       LoopClosureMatchResult* result) const;

 private:
  localization::ScanMatchConfig scan_match_config_{};
  localization::IcpMatcher icp_matcher_{};
  localization::NdtMatcher ndt_matcher_{};
  std::string matcher_{"icp"};
  int max_points_{160};
};

}  // namespace rm_nav::mapping
