#include "rm_nav/planning/mission_manager.hpp"

#include <chrono>
#include <algorithm>
#include <cmath>

namespace rm_nav::planning {
namespace {

constexpr float kTargetChangePositionEpsilonM = 0.05F;
constexpr float kTargetChangeYawEpsilonRad = 0.05F;

}  // namespace

void MissionManager::Reset() {
  phase_ = MissionPhase::kApproach;
  tracked_target_ = {};
  ResetSettleTracking();
}

MissionStatus MissionManager::Update(const data::Pose3f& current_pose, const GoalState& goal) {
  MissionStatus status;
  if (!current_pose.is_valid || !goal.target_pose.is_valid) {
    Reset();
    return status;
  }
  if (TargetChanged(goal.target_pose)) {
    phase_ = MissionPhase::kApproach;
    tracked_target_ = goal.target_pose;
    ResetSettleTracking();
  }

  const bool inside_goal_window = goal.within_center_radius && goal.yaw_aligned;
  if (phase_ == MissionPhase::kCenterHold &&
      goal.distance_to_center_m > static_cast<float>(config_.recenter_threshold_m)) {
    status.hold_drifted = true;
    phase_ = MissionPhase::kApproach;
    ResetSettleTracking();
  }

  if (phase_ != MissionPhase::kCenterHold) {
    if (inside_goal_window) {
      if (consecutive_in_goal_frames_ == 0) {
        in_goal_since_ = current_pose.stamp;
      }
      ++consecutive_in_goal_frames_;

      const int required_frames = std::max(1, config_.center_hold_settle_frames);
      const bool frames_ready = consecutive_in_goal_frames_ >= required_frames;
      bool time_ready = config_.center_hold_settle_time_ms <= 0;
      if (!time_ready) {
        if (current_pose.stamp == common::TimePoint{} || in_goal_since_ == common::TimePoint{}) {
          time_ready = true;
        } else {
          const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                                   current_pose.stamp - in_goal_since_)
                                   .count();
          time_ready = elapsed >= config_.center_hold_settle_time_ms;
        }
      }

      phase_ = (frames_ready && time_ready) ? MissionPhase::kCenterHold
                                            : MissionPhase::kSettling;
    } else {
      phase_ = MissionPhase::kApproach;
      ResetSettleTracking();
    }
  }

  status.phase = phase_;
  status.mode =
      phase_ == MissionPhase::kCenterHold ? GoalMode::kCenterHold : GoalMode::kApproachCenter;
  status.reached = phase_ == MissionPhase::kCenterHold;
  status.settling = phase_ == MissionPhase::kSettling;
  status.consecutive_in_goal_frames = consecutive_in_goal_frames_;
  if (in_goal_since_ != common::TimePoint{} && current_pose.stamp != common::TimePoint{} &&
      current_pose.stamp >= in_goal_since_) {
    status.settle_elapsed_ns = common::ToNanoseconds(current_pose.stamp - in_goal_since_);
  }
  return status;
}

bool MissionManager::TargetChanged(const data::Pose3f& target_pose) const {
  if (!tracked_target_.is_valid) {
    return true;
  }
  const float dx = tracked_target_.position.x - target_pose.position.x;
  const float dy = tracked_target_.position.y - target_pose.position.y;
  const float dyaw = tracked_target_.rpy.z - target_pose.rpy.z;
  return std::fabs(dx) > kTargetChangePositionEpsilonM ||
         std::fabs(dy) > kTargetChangePositionEpsilonM ||
         std::fabs(dyaw) > kTargetChangeYawEpsilonRad;
}

void MissionManager::ResetSettleTracking() {
  consecutive_in_goal_frames_ = 0;
  in_goal_since_ = {};
}

}  // namespace rm_nav::planning
