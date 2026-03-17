#pragma once

#include <cstddef>

#include "rm_nav/common/status.hpp"
#include "rm_nav/data/lidar_frame.hpp"
#include "rm_nav/data/pose.hpp"
#include "rm_nav/localization/map_loader.hpp"

namespace rm_nav::localization {

struct ScanMatchConfig {
  int max_iterations{8};
  float correspondence_distance_m{0.75F};
  float min_match_score{0.55F};
};

struct ScanMatchResult {
  data::Pose3f matched_pose{};
  float score{0.0F};
  int iterations{0};
  std::size_t correspondence_count{0};
  bool converged{false};
};

class ScanMatcher {
 public:
  virtual ~ScanMatcher() = default;

  virtual common::Status Configure(const ScanMatchConfig& config) = 0;
  virtual common::Status Match(const StaticMap& map, const data::LidarFrame& scan,
                               const data::Pose3f& initial_guess,
                               ScanMatchResult* result) const = 0;
};

class IcpMatcher final : public ScanMatcher {
 public:
  common::Status Configure(const ScanMatchConfig& config) override;
  common::Status Match(const StaticMap& map, const data::LidarFrame& scan,
                       const data::Pose3f& initial_guess,
                       ScanMatchResult* result) const override;

 private:
  ScanMatchConfig config_{};
};

}  // namespace rm_nav::localization
