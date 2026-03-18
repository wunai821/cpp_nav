#pragma once

#include <vector>
#include <string_view>

#include "rm_nav/common/status.hpp"
#include "rm_nav/config/localization_config.hpp"
#include "rm_nav/localization/icp_matcher.hpp"
#include "rm_nav/localization/map_loader.hpp"
#include "rm_nav/localization/ndt_matcher.hpp"

namespace rm_nav::localization {

struct RelocalizationStatus {
  bool active{false};
  bool attempted{false};
  bool succeeded{false};
  bool fallback_matcher_used{false};
  int candidates_tested{0};
  float best_score{0.0F};
  common::TimeNs attempt_latency_ns{0};
  std::string_view matcher_used{"none"};
  data::Pose3f best_initial_guess{};
  data::Pose3f matched_pose{};
};

class RelocalizationManager {
 public:
  common::Status Configure(const config::LocalizationConfig& config);
  common::Status Attempt(const StaticMap& map, const data::LidarFrame& scan,
                         const data::Pose3f& predicted_guess,
                         const data::Pose3f& fallback_guess, common::TimePoint stamp,
                         ScanMatchResult* best_match,
                         RelocalizationStatus* status);

 private:
  StaticMap BuildLocalSubmap(const StaticMap& map, const data::Pose3f& center_pose) const;
  std::vector<data::Pose3f> BuildCandidates(const data::Pose3f& predicted_guess,
                                            const data::Pose3f& fallback_guess) const;

  config::LocalizationConfig config_{};
  IcpMatcher coarse_icp_matcher_{};
  NdtMatcher coarse_ndt_matcher_{};
  common::TimePoint last_attempt_stamp_{};
  bool configured_{false};
};

}  // namespace rm_nav::localization
