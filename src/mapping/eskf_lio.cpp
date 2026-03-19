#include "rm_nav/mapping/eskf_lio.hpp"

#include <algorithm>
#include <cmath>

#include "rm_nav/tf/frame_ids.hpp"

namespace rm_nav::mapping {
namespace {

float NormalizeAngle(float angle) {
  constexpr float kPi = 3.14159265358979323846F;
  while (angle > kPi) {
    angle -= 2.0F * kPi;
  }
  while (angle < -kPi) {
    angle += 2.0F * kPi;
  }
  return angle;
}

float Clamp01(double value) {
  if (value <= 0.0) {
    return 0.0F;
  }
  if (value >= 1.0) {
    return 1.0F;
  }
  return static_cast<float>(value);
}

data::Pose3f RelativePose2D(const data::Pose3f& reference, const data::Pose3f& current) {
  data::Pose3f relative;
  relative.stamp = current.stamp;
  relative.reference_frame = tf::kBaseLinkFrame;
  relative.child_frame = tf::kBaseLinkFrame;
  relative.is_valid = reference.is_valid && current.is_valid;
  if (!relative.is_valid) {
    return relative;
  }

  const float dx = current.position.x - reference.position.x;
  const float dy = current.position.y - reference.position.y;
  const float cos_yaw = std::cos(reference.rpy.z);
  const float sin_yaw = std::sin(reference.rpy.z);
  relative.position.x = cos_yaw * dx + sin_yaw * dy;
  relative.position.y = -sin_yaw * dx + cos_yaw * dy;
  relative.position.z = current.position.z - reference.position.z;
  relative.rpy.z = NormalizeAngle(current.rpy.z - reference.rpy.z);
  return relative;
}

data::Pose3f ComposePose2D(const data::Pose3f& parent, const data::Pose3f& relative) {
  data::Pose3f composed = parent;
  const float cos_yaw = std::cos(parent.rpy.z);
  const float sin_yaw = std::sin(parent.rpy.z);
  composed.position.x =
      parent.position.x + cos_yaw * relative.position.x - sin_yaw * relative.position.y;
  composed.position.y =
      parent.position.y + sin_yaw * relative.position.x + cos_yaw * relative.position.y;
  composed.position.z = parent.position.z + relative.position.z;
  composed.rpy.z = NormalizeAngle(parent.rpy.z + relative.rpy.z);
  composed.is_valid = parent.is_valid && relative.is_valid;
  return composed;
}

float Hypot2D(float x, float y) {
  return std::sqrt(x * x + y * y);
}

}  // namespace

common::Status EskfLio::Configure(const config::MappingConfig& config) {
  config_ = config;
  return common::Status::Ok();
}

common::Status EskfLio::PredictPose(const data::Pose3f& previous_external_pose,
                                    const data::Pose3f& previous_mapping_pose,
                                    const data::Pose3f& current_external_pose,
                                    const data::PreintegratedImuBlock& preint,
                                    EskfLitePrediction* prediction) const {
  if (prediction == nullptr) {
    return common::Status::InvalidArgument("eskf-lite prediction output must not be null");
  }

  *prediction = {};
  if (!previous_external_pose.is_valid || !previous_mapping_pose.is_valid ||
      !current_external_pose.is_valid) {
    prediction->predicted_pose = current_external_pose;
    return common::Status::Ok();
  }

  data::Pose3f relative = RelativePose2D(previous_external_pose, current_external_pose);
  if (preint.is_valid && preint.sample_count > 0U) {
    const float yaw_weight = Clamp01(config_.frontend_imu_yaw_weight);
    float position_weight = Clamp01(config_.frontend_imu_position_weight);
    const float external_distance = Hypot2D(relative.position.x, relative.position.y);
    const float imu_distance = Hypot2D(preint.delta_position.x, preint.delta_position.y);
    if (imu_distance < 1.0e-3F ||
        std::fabs(imu_distance - external_distance) >
        std::max(0.10F, external_distance + 0.15F)) {
      position_weight = 0.0F;
    }

    relative.position.x =
        (1.0F - position_weight) * relative.position.x + position_weight * preint.delta_position.x;
    relative.position.y =
        (1.0F - position_weight) * relative.position.y + position_weight * preint.delta_position.y;
    relative.rpy.z = NormalizeAngle(
        (1.0F - yaw_weight) * relative.rpy.z + yaw_weight * preint.delta_rpy.z);
    prediction->used_preintegration = true;
  }

  prediction->predicted_pose = ComposePose2D(previous_mapping_pose, relative);
  return common::Status::Ok();
}

}  // namespace rm_nav::mapping
