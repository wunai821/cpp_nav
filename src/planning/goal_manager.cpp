#include "rm_nav/planning/goal_manager.hpp"

#include <cmath>

#include "rm_nav/tf/frame_ids.hpp"

namespace rm_nav::planning {
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

GoalState GoalManager::Update(const data::Pose3f& current_pose) {
  data::Pose3f target_pose;
  target_pose.stamp = current_pose.stamp;
  target_pose.reference_frame = tf::kMapFrame;
  target_pose.child_frame = tf::kBaseLinkFrame;
  target_pose.position.x = static_cast<float>(config_.center_goal_x_m);
  target_pose.position.y = static_cast<float>(config_.center_goal_y_m);
  target_pose.rpy.z = 0.0F;
  target_pose.is_valid = true;
  return BuildState(current_pose, target_pose);
}

GoalState GoalManager::UpdateToward(const data::Pose3f& current_pose,
                                    const data::Pose3f& target_pose) {
  return BuildState(current_pose, target_pose);
}

GoalState GoalManager::BuildState(const data::Pose3f& current_pose,
                                  const data::Pose3f& target_pose) {
  GoalState state;
  state.target_pose = target_pose;
  state.target_pose.stamp = current_pose.stamp;
  state.target_pose.reference_frame = tf::kMapFrame;
  state.target_pose.child_frame = tf::kBaseLinkFrame;
  state.target_pose.is_valid = true;

  const float dx = state.target_pose.position.x - current_pose.position.x;
  const float dy = state.target_pose.position.y - current_pose.position.y;
  state.distance_to_target_m = std::sqrt(dx * dx + dy * dy);
  state.distance_to_center_m = state.distance_to_target_m;
  state.yaw_error_rad = NormalizeAngle(state.target_pose.rpy.z - current_pose.rpy.z);
  state.within_center_radius =
      state.distance_to_center_m <= static_cast<float>(config_.center_radius_m);
  state.yaw_aligned =
      std::fabs(state.yaw_error_rad) <=
      static_cast<float>(config_.yaw_align_tolerance_rad);
  state.mode = GoalMode::kApproachCenter;
  state.reached = false;
  return state;
}

}  // namespace rm_nav::planning
