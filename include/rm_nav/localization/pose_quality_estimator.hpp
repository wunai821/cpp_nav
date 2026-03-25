#pragma once

#include "rm_nav/common/time.hpp"
#include <cstdint>
#include <string>

#include "rm_nav/config/localization_config.hpp"
#include "rm_nav/localization/icp_matcher.hpp"

namespace rm_nav::localization {

struct LocalizationRejectionCounters {
  std::uint32_t total{0};
  std::uint32_t map_unloaded{0};
  std::uint32_t matcher_not_converged{0};
  std::uint32_t low_match_score{0};
  std::uint32_t pose_jump_guard{0};
  std::uint32_t consecutive_failures{0};
  std::uint32_t map_to_odom_guard{0};
  std::uint32_t relocalization_stabilization_failed{0};
  std::uint32_t stabilization_window{0};
  std::uint32_t other{0};
};

struct LocalizationStatus {
  float match_score{0.0F};
  int iterations{0};
  bool converged{false};
  float pose_jump_m{0.0F};
  float yaw_jump_rad{0.0F};
  std::uint32_t consecutive_failures{0};
  bool pose_trusted{false};
  bool map_loaded{false};
  bool lost_lock{false};
  std::string rejection_reason{"none"};
  std::string rejected_because{"none"};
  std::string degraded_mode{"none"};
  std::string source{"none"};
  std::uint32_t consecutive_rejections{0};
  LocalizationRejectionCounters rejection_counters{};
  common::TimeNs last_good_stamp_ns{0};
  float map_to_odom_guard_translation_m{0.0F};
  float map_to_odom_guard_yaw_rad{0.0F};
  bool map_to_odom_guard_failed_translation{false};
  bool map_to_odom_guard_failed_yaw{false};
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
