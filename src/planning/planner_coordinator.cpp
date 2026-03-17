#include "rm_nav/planning/planner_coordinator.hpp"

#include <algorithm>
#include <cmath>

#include "rm_nav/common/time.hpp"

namespace rm_nav::planning {
namespace {

float ClampAbs(float value, float limit) {
  return std::max(-limit, std::min(limit, value));
}

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

bool IsNearZero(const data::ChassisCmd& cmd) {
  return std::fabs(cmd.vx_mps) + std::fabs(cmd.vy_mps) + std::fabs(cmd.wz_radps) < 0.01F;
}

void ApplyPathFollowerFallback(const config::PlannerConfig& config,
                               const data::Pose3f& current_pose,
                               const GoalState& goal,
                               const data::Path2D& path,
                               data::ChassisCmd* cmd) {
  if (cmd == nullptr || path.points.size() < 2U) {
    return;
  }

  const std::size_t lookahead_index = std::min<std::size_t>(3U, path.points.size() - 1U);
  const auto& target = path.points[lookahead_index];
  const float dx_world = target.position.x - current_pose.position.x;
  const float dy_world = target.position.y - current_pose.position.y;
  const float cos_yaw = std::cos(current_pose.rpy.z);
  const float sin_yaw = std::sin(current_pose.rpy.z);
  const float dx_body = cos_yaw * dx_world + sin_yaw * dy_world;
  const float dy_body = -sin_yaw * dx_world + cos_yaw * dy_world;

  const float max_vx = static_cast<float>(
      goal.mode == GoalMode::kCenterHold ? config.hold_max_v_mps : config.max_vx_mps);
  const float max_vy = static_cast<float>(
      goal.mode == GoalMode::kCenterHold ? config.hold_max_v_mps : config.max_vy_mps);
  const float max_wz = static_cast<float>(
      goal.mode == GoalMode::kCenterHold ? config.hold_max_wz_radps : config.max_wz_radps);

  cmd->vx_mps = ClampAbs(dx_body * 0.8F, max_vx);
  cmd->vy_mps = ClampAbs(dy_body * 0.8F, max_vy);
  cmd->wz_radps = ClampAbs(NormalizeAngle(goal.target_pose.rpy.z - current_pose.rpy.z) * 0.8F,
                           max_wz);
  cmd->brake = false;
}

}  // namespace

common::Status PlannerCoordinator::Initialize(const config::PlannerConfig& config,
                                              const localization::StaticMap& static_map) {
  config_ = config;
  static_map_ = static_map;
  goal_manager_ = GoalManager(config);
  omni_dwa_ = OmniDwa(config);
  global_path_.Publish(data::Path2D{});
  latest_cmd_.Publish(data::ChassisCmd{});
  latest_status_.Publish(PlannerStatus{});
  initialized_ = true;
  return common::Status::Ok();
}

common::Status PlannerCoordinator::Plan(
    const data::Pose3f& current_pose, const data::GridMap2D& costmap,
    const std::vector<data::DynamicObstacle>& obstacles, data::Path2D* path,
    data::ChassisCmd* cmd) {
  return PlanToGoal(current_pose, {}, costmap, obstacles, path, cmd);
}

common::Status PlannerCoordinator::PlanToGoal(
    const data::Pose3f& current_pose, const data::Pose3f& goal_pose,
    const data::GridMap2D& costmap, const std::vector<data::DynamicObstacle>& obstacles,
    data::Path2D* path, data::ChassisCmd* cmd) {
  if (!initialized_) {
    return common::Status::NotReady("planner is not initialized");
  }
  if (path == nullptr || cmd == nullptr) {
    return common::Status::InvalidArgument("planner output is null");
  }

  const auto begin_ns = common::NowNs();
  const GoalState goal =
      goal_pose.is_valid ? goal_manager_.UpdateToward(current_pose, goal_pose)
                         : goal_manager_.Update(current_pose);
  auto status = global_astar_.Plan(static_map_.occupancy, current_pose, goal.target_pose, path);
  if (!status.ok()) {
    return status;
  }
  path->stamp = current_pose.stamp;
  if (!path->points.empty()) {
    path->points.front().position.x = current_pose.position.x;
    path->points.front().position.y = current_pose.position.y;
  }

  DwaScore dwa_score;
  status = omni_dwa_.Plan(current_pose, goal, *path, costmap, obstacles,
                          latest_cmd_.ReadSnapshot(), cmd, &dwa_score);
  if (!status.ok()) {
    return status;
  }
  if (!goal.reached && IsNearZero(*cmd)) {
    ApplyPathFollowerFallback(config_, current_pose, goal, *path, cmd);
  }
  cmd->stamp = current_pose.stamp;

  PlannerStatus planner_status;
  planner_status.mode = goal.mode;
  planner_status.distance_to_goal_m = goal.distance_to_target_m;
  planner_status.distance_to_center_m = goal.distance_to_center_m;
  planner_status.yaw_error_rad = goal.yaw_error_rad;
  planner_status.reached = goal.reached;
  planner_status.path_available = !path->points.empty();
  planner_status.dwa_score = dwa_score;
  planner_status.planning_latency_ns = common::NowNs() - begin_ns;
  latest_status_.Publish(planner_status);
  return common::Status::Ok();
}

common::Status PlannerCoordinator::PlanAndPublish(
    const data::Pose3f& current_pose, const data::GridMap2D& costmap,
    const std::vector<data::DynamicObstacle>& obstacles) {
  return PlanAndPublishToGoal(current_pose, {}, costmap, obstacles);
}

common::Status PlannerCoordinator::PlanAndPublishToGoal(
    const data::Pose3f& current_pose, const data::Pose3f& goal_pose,
    const data::GridMap2D& costmap, const std::vector<data::DynamicObstacle>& obstacles) {
  data::Path2D path;
  data::ChassisCmd cmd;
  const auto status = PlanToGoal(current_pose, goal_pose, costmap, obstacles, &path, &cmd);
  if (!status.ok()) {
    return status;
  }
  global_path_.Publish(std::move(path));
  latest_cmd_.Publish(cmd);
  return common::Status::Ok();
}

}  // namespace rm_nav::planning
