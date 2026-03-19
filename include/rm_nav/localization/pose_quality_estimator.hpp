#pragma once

#include <cstdint>
#include <string>

#include "rm_nav/config/localization_config.hpp"
#include "rm_nav/localization/icp_matcher.hpp"

namespace rm_nav::localization {

struct LocalizationStatus {
  float match_score{0.0F};
  int iterations{0};
  bool converged{false};
  float pose_jump_m{0.0F};
  float yaw_jump_rad{0.0F};
  std::uint32_t consecutive_failures{0};
  bool pose_trusted{false};
  bool map_loaded{false};
  std::string rejection_reason{"none"};
  std::string degraded_mode{"none"};
};

class PoseQualityEstimator {
 public:
  explicit PoseQualityEstimator(const config::LocalizationConfig& config)
      : config_(config) {}

  LocalizationStatus Evaluate(const data::Pose3f& previous_pose,
                              const ScanMatchResult& match,
                              std::uint32_t consecutive_failures,
                              bool map_loaded,
                              bool allow_large_jump) const;

 private:
  config::LocalizationConfig config_{};
};

}  // namespace rm_nav::localization
