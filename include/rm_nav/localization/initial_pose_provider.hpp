#pragma once

#include "rm_nav/config/spawn_config.hpp"
#include "rm_nav/data/pose.hpp"

namespace rm_nav::localization {

class InitialPoseProvider {
 public:
  explicit InitialPoseProvider(const config::SpawnConfig& config) : config_(config) {}

  data::Pose3f MakeInitialPose(common::TimePoint stamp) const;

 private:
  config::SpawnConfig config_{};
};

}  // namespace rm_nav::localization
