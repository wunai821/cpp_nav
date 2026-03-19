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

float Clamp01(float value) {
  if (value < 0.0F) {
    return 0.0F;
  }
  if (value > 1.0F) {
    return 1.0F;
  }
  return value;
}

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
  float best_score = -std::numeric_limits<float>::max();

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

    const float distance_score =
        1.0F - (distance /
                std::max(0.01F, static_cast<float>(config.loop_candidate_distance_threshold_m)));
    const float yaw_score =
        1.0F - (yaw_delta /
                std::max(0.01F, static_cast<float>(config.loop_candidate_max_yaw_delta_rad)));
    const float time_score = Clamp01(
        static_cast<float>(separation_ns) /
        std::max(1.0F, 2.0F * static_cast<float>(min_separation_ns)));
    const float revisit_score =
        0.50F * Clamp01(distance_score) + 0.20F * Clamp01(yaw_score) + 0.30F * time_score;
    if (revisit_score < static_cast<float>(config.loop_candidate_min_revisit_score)) {
      continue;
    }

    if (revisit_score > best_score) {
      best_score = revisit_score;
      candidate->found = true;
      candidate->keyframe_index = index;
      candidate->frame_index = keyframe.frame_index;
      candidate->stamp = keyframe.stamp;
      candidate->distance_m = distance;
      candidate->yaw_delta_rad = yaw_delta;
      candidate->revisit_score = revisit_score;
      candidate->time_separation_ns = separation_ns;
    }
  }

  return common::Status::Ok();
}

}  // namespace rm_nav::mapping
