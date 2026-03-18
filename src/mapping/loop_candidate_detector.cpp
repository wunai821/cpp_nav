#include "rm_nav/mapping/loop_candidate_detector.hpp"

#include <cmath>
#include <limits>

#include "rm_nav/common/time.hpp"
#include "rm_nav/mapping/mapping_engine.hpp"

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

}  // namespace

common::Status LoopCandidateDetector::FindCandidate(
    const config::MappingConfig& config, const data::Pose3f& current_pose,
    common::TimePoint current_stamp, std::uint32_t current_frame_index,
    const std::vector<MappingKeyframe>& keyframes, LoopClosureCandidate* candidate) const {
  if (candidate == nullptr) {
    return common::Status::InvalidArgument("loop candidate output must not be null");
  }

  *candidate = {};
  if (!current_pose.is_valid || keyframes.empty()) {
    return common::Status::Ok();
  }

  const auto min_separation_ns = static_cast<common::TimeNs>(
      config.loop_candidate_min_time_separation_s * static_cast<double>(common::kNanosecondsPerSecond));
  float best_distance = std::numeric_limits<float>::max();

  for (std::size_t index = 0; index < keyframes.size(); ++index) {
    const auto& keyframe = keyframes[index];
    if (!keyframe.map_to_base.is_valid || keyframe.frame_index == current_frame_index) {
      continue;
    }

    const common::TimeNs separation_ns =
        common::ToNanoseconds(current_stamp - keyframe.stamp);
    if (separation_ns < min_separation_ns) {
      continue;
    }

    const float dx = current_pose.position.x - keyframe.map_to_base.position.x;
    const float dy = current_pose.position.y - keyframe.map_to_base.position.y;
    const float distance = std::sqrt(dx * dx + dy * dy);
    if (distance > static_cast<float>(config.loop_candidate_distance_threshold_m)) {
      continue;
    }

    const float yaw_delta =
        std::fabs(NormalizeAngle(current_pose.rpy.z - keyframe.map_to_base.rpy.z));
    if (yaw_delta > static_cast<float>(config.loop_candidate_max_yaw_delta_rad)) {
      continue;
    }

    if (distance < best_distance) {
      best_distance = distance;
      candidate->found = true;
      candidate->keyframe_index = index;
      candidate->frame_index = keyframe.frame_index;
      candidate->stamp = keyframe.stamp;
      candidate->distance_m = distance;
      candidate->yaw_delta_rad = yaw_delta;
      candidate->time_separation_ns = separation_ns;
    }
  }

  return common::Status::Ok();
}

}  // namespace rm_nav::mapping
