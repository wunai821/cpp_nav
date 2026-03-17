#pragma once

#include "rm_nav/localization/icp_matcher.hpp"

namespace rm_nav::localization {

class NdtMatcher final : public ScanMatcher {
 public:
  common::Status Configure(const ScanMatchConfig& config) override;
  common::Status Match(const StaticMap& map, const data::LidarFrame& scan,
                       const data::Pose3f& initial_guess,
                       ScanMatchResult* result) const override;
};

}  // namespace rm_nav::localization
