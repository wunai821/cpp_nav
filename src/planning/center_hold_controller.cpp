#include "rm_nav/planning/center_hold_controller.hpp"

namespace rm_nav::planning {

common::Status CenterHoldController::BuildHoldCommand(const data::Pose3f& current_pose,
                                                      const data::Pose3f& target_pose,
                                                      data::Path2D* path,
                                                      data::ChassisCmd* cmd) const {
  if (path == nullptr || cmd == nullptr) {
    return common::Status::InvalidArgument("center hold output is null");
  }

  path->stamp = current_pose.stamp;
  path->points.clear();
  data::PathPoint2f point;
  point.position.x = target_pose.is_valid ? target_pose.position.x : current_pose.position.x;
  point.position.y = target_pose.is_valid ? target_pose.position.y : current_pose.position.y;
  point.heading_rad = target_pose.is_valid ? target_pose.rpy.z : current_pose.rpy.z;
  point.target_speed_mps = 0.0F;
  path->points.push_back(point);

  cmd->stamp = current_pose.stamp;
  cmd->vx_mps = 0.0F;
  cmd->vy_mps = 0.0F;
  cmd->wz_radps = 0.0F;
  cmd->brake = true;
  return common::Status::Ok();
}

}  // namespace rm_nav::planning
