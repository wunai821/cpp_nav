#include "rm_nav/mapping/lio_frontend.hpp"

#include <algorithm>
#include <cmath>

namespace rm_nav::mapping {
namespace {

float MaxPointRelativeTime(const data::LidarFrame& scan) {
  float max_relative_time = 0.0F;
  for (const auto& point : scan.points) {
    max_relative_time = std::max(max_relative_time, point.relative_time_s);
  }
  return max_relative_time;
}

}  // namespace

common::Status LioFrontend::Configure(const config::MappingConfig& config) {
  config_ = config;
  return eskf_lio_.Configure(config);
}

common::Status LioFrontend::PredictPose(const data::Pose3f& previous_external_pose,
                                        const data::Pose3f& previous_mapping_pose,
                                        const data::Pose3f& current_external_pose,
                                        const data::PreintegratedImuBlock& preint,
                                        LioFrontendPrediction* prediction) const {
  if (prediction == nullptr) {
    return common::Status::InvalidArgument("lio frontend prediction output must not be null");
  }

  *prediction = {};
  EskfLitePrediction eskf_prediction;
  auto status = eskf_lio_.PredictPose(previous_external_pose, previous_mapping_pose,
                                      current_external_pose, preint, &eskf_prediction);
  if (!status.ok()) {
    return status;
  }

  prediction->predicted_pose = eskf_prediction.predicted_pose;
  prediction->used_imu_prediction = eskf_prediction.used_preintegration;
  prediction->motion_compensation_recommended =
      config_.frontend_motion_compensation_enabled && preint.is_valid &&
      std::fabs(preint.delta_rpy.z) >=
          static_cast<float>(config_.frontend_motion_compensation_min_yaw_rad);
  return common::Status::Ok();
}

common::Status LioFrontend::PrepareFrame(const data::SyncedFrame& input,
                                         data::SyncedFrame* output) const {
  if (output == nullptr) {
    return common::Status::InvalidArgument("prepared frame output must not be null");
  }

  *output = input;
  if (!config_.frontend_motion_compensation_enabled || !input.preint.is_valid ||
      input.lidar.points.empty() ||
      std::fabs(input.preint.delta_rpy.z) <
          static_cast<float>(config_.frontend_motion_compensation_min_yaw_rad)) {
    return common::Status::Ok();
  }

  const float scan_duration_s = std::max(
      MaxPointRelativeTime(input.lidar),
      static_cast<float>(common::ToNanoseconds(input.preint.duration)) / 1.0e9F);
  if (scan_duration_s <= 1.0e-4F) {
    return common::Status::Ok();
  }

  for (auto& point : output->lidar.points) {
    const float alpha = std::clamp(point.relative_time_s / scan_duration_s, 0.0F, 1.0F);
    const float undo_yaw = -alpha * input.preint.delta_rpy.z;
    const float cos_yaw = std::cos(undo_yaw);
    const float sin_yaw = std::sin(undo_yaw);
    const float translated_x = point.x - alpha * input.preint.delta_position.x;
    const float translated_y = point.y - alpha * input.preint.delta_position.y;
    const float corrected_x = cos_yaw * translated_x - sin_yaw * translated_y;
    const float corrected_y = sin_yaw * translated_x + cos_yaw * translated_y;
    point.x = corrected_x;
    point.y = corrected_y;
  }
  return common::Status::Ok();
}

}  // namespace rm_nav::mapping
