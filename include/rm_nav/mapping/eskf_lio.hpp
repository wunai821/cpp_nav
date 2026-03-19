#pragma once

#include "rm_nav/common/status.hpp"
#include "rm_nav/config/mapping_config.hpp"
#include "rm_nav/data/pose.hpp"
#include "rm_nav/data/preint_block.hpp"

namespace rm_nav::mapping {

struct EskfLitePrediction {
  data::Pose3f predicted_pose{};
  bool used_preintegration{false};
};

class EskfLio {
 public:
  common::Status Configure(const config::MappingConfig& config);
  common::Status PredictPose(const data::Pose3f& previous_external_pose,
                             const data::Pose3f& previous_mapping_pose,
                             const data::Pose3f& current_external_pose,
                             const data::PreintegratedImuBlock& preint,
                             EskfLitePrediction* prediction) const;

 private:
  config::MappingConfig config_{};
};

}  // namespace rm_nav::mapping
