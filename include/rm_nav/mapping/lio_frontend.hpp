#pragma once

#include "rm_nav/common/status.hpp"
#include "rm_nav/config/mapping_config.hpp"
#include "rm_nav/data/pose.hpp"
#include "rm_nav/data/synced_frame.hpp"
#include "rm_nav/mapping/eskf_lio.hpp"

namespace rm_nav::mapping {

struct LioFrontendPrediction {
  data::Pose3f predicted_pose{};
  bool used_imu_prediction{false};
  bool motion_compensation_recommended{false};
};

class LioFrontend {
 public:
  common::Status Configure(const config::MappingConfig& config);
  common::Status PredictPose(const data::Pose3f& previous_external_pose,
                             const data::Pose3f& previous_mapping_pose,
                             const data::Pose3f& current_external_pose,
                             const data::PreintegratedImuBlock& preint,
                             LioFrontendPrediction* prediction) const;
  common::Status PrepareFrame(const data::SyncedFrame& input,
                              data::SyncedFrame* output) const;

 private:
  config::MappingConfig config_{};
  EskfLio eskf_lio_{};
};

}  // namespace rm_nav::mapping
