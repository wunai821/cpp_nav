#include "rm_nav/planning/planner_coordinator.hpp"

#include <algorithm>
#include <cmath>
#include <sstream>

#include "rm_nav/common/time.hpp"
#include "rm_nav/utils/logger.hpp"

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

const char* ToString(GoalMode mode) {
  switch (mode) {
    case GoalMode::kApproachCenter:
      return "approach_center";
    case GoalMode::kCenterHold:
      return "center_hold";
    default:
      return "unknown";
  }
}

void LogPlannerStatusIfChanged(const PlannerStatus& previous, const PlannerStatus& current) {
  const bool changed = previous.global_plan_succeeded != current.global_plan_succeeded ||
                       previous.local_plan_succeeded != current.local_plan_succeeded ||
                       previous.reached != current.reached ||
                       previous.fallback_cmd_used != current.fallback_cmd_used ||
                       previous.temporary_goal_active != current.temporary_goal_active ||
                       std::fabs(previous.clearance_weight_scale -
                                 current.clearance_weight_scale) > 1.0e-4F ||
                       previous.degraded_mode != current.degraded_mode ||
                       previous.failure_reason != current.failure_reason ||
                       previous.mode != current.mode;
  if (!changed) {
    return;
  }

  std::ostringstream stream;
  stream << "mode=" << ToString(current.mode)
         << " map=" << current.map_version
         << " global=" << (current.global_plan_succeeded ? "ok" : "fail")
         << " local=" << (current.local_plan_succeeded ? "ok" : "fail")
         << " reached=" << (current.reached ? "true" : "false")
         << " fallback=" << (current.fallback_cmd_used ? "true" : "false")
         << " temp_goal=" << (current.temporary_goal_active ? "true" : "false")
         << " clearance_scale=" << current.clearance_weight_scale
         << " degraded=" << current.degraded_mode
         << " reason=" << current.failure_reason
         << " dist=" << current.distance_to_goal_m;
  if (current.global_plan_succeeded && current.local_plan_succeeded) {
    utils::LogInfo("planner", stream.str());
  } else {
    utils::LogWarn("planner", stream.str());
  }
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
  mission_manager_ = MissionManager(config);
  center_hold_controller_ = CenterHoldController(config);
  omni_dwa_ = OmniDwa(config);
  global_path_.Publish(data::Path2D{});
  latest_cmd_.Publish(data::ChassisCmd{});
  PlannerStatus initial_status;
  initial_status.map_version = static_map_.version_label;
  latest_status_.Publish(initial_status);
  last_logged_status_ = initial_status;
  initialized_ = true;
  return common::Status::Ok();
}

common::Status PlannerCoordinator::Plan(
    const data::Pose3f& current_pose, const data::GridMap2D& costmap,
    const std::vector<data::DynamicObstacle>& obstacles, data::Path2D* path,
    data::ChassisCmd* cmd, const PlanningOverrides* overrides) {
  return PlanToGoal(current_pose, {}, costmap, obstacles, path, cmd, overrides);
}

common::Status PlannerCoordinator::PlanToGoal(
    const data::Pose3f& current_pose, const data::Pose3f& goal_pose,
    const data::GridMap2D& costmap, const std::vector<data::DynamicObstacle>& obstacles,
    data::Path2D* path, data::ChassisCmd* cmd, const PlanningOverrides* overrides) {
  if (!initialized_) {
    return common::Status::NotReady("planner is not initialized");
  }
  if (path == nullptr || cmd == nullptr) {
    return common::Status::InvalidArgument("planner output is null");
  }

  const auto begin_ns = common::NowNs();
  const data::Pose3f selected_goal_pose =
      overrides != nullptr && overrides->temporary_goal_valid ? overrides->temporary_goal
                                                              : goal_pose;
  const float clearance_weight_scale =
      overrides != nullptr ? std::max(0.1F, overrides->clearance_weight_scale) : 1.0F;
  const GoalState goal =
      selected_goal_pose.is_valid ? goal_manager_.UpdateToward(current_pose, selected_goal_pose)
                         : goal_manager_.Update(current_pose);
  const MissionStatus mission = mission_manager_.Update(current_pose, goal);
  PlannerStatus planner_status;
  planner_status.mode = mission.mode;
  planner_status.distance_to_goal_m = goal.distance_to_target_m;
  planner_status.distance_to_center_m = goal.distance_to_center_m;
  planner_status.yaw_error_rad = goal.yaw_error_rad;
  planner_status.reached = mission.reached;
  planner_status.settling = mission.settling;
  planner_status.hold_drifted = mission.hold_drifted;
  planner_status.hold_frames_in_goal = mission.consecutive_in_goal_frames;
  planner_status.hold_settle_elapsed_ns = mission.settle_elapsed_ns;
  planner_status.map_version = static_map_.version_label;
  planner_status.clearance_weight_scale = clearance_weight_scale;
  planner_status.temporary_goal_active =
      overrides != nullptr && overrides->temporary_goal_valid;
  if (mission.reached) {
    auto status = center_hold_controller_.BuildHoldCommand(current_pose, goal.target_pose, costmap,
                                                           obstacles, path, cmd);
    if (!status.ok()) {
      planner_status.failure_reason = status.message;
      planner_status.degraded_mode = "center_hold_failed";
      planner_status.planning_latency_ns = common::NowNs() - begin_ns;
      latest_status_.Publish(planner_status);
      LogPlannerStatusIfChanged(last_logged_status_, planner_status);
      last_logged_status_ = planner_status;
      return status;
    }
    planner_status.global_plan_succeeded = true;
    planner_status.local_plan_succeeded = true;
    planner_status.reached = true;
    planner_status.path_available = !path->points.empty();
    planner_status.degraded_mode =
        planner_status.temporary_goal_active || planner_status.clearance_weight_scale > 1.01F
            ? "center_hold_recovery_bias"
            : "center_hold";
    planner_status.planning_latency_ns = common::NowNs() - begin_ns;
    latest_status_.Publish(planner_status);
    LogPlannerStatusIfChanged(last_logged_status_, planner_status);
    last_logged_status_ = planner_status;
    return common::Status::Ok();
  }
  center_hold_controller_.Reset();

  auto status = global_astar_.Plan(static_map_.occupancy, current_pose, goal.target_pose, path);
  if (!status.ok()) {
    planner_status.global_plan_succeeded = false;
    planner_status.local_plan_succeeded = false;
    planner_status.path_available = false;
    planner_status.failure_reason = status.message;
    planner_status.degraded_mode = "global_plan_failed";
    planner_status.planning_latency_ns = common::NowNs() - begin_ns;
    latest_status_.Publish(planner_status);
    LogPlannerStatusIfChanged(last_logged_status_, planner_status);
    last_logged_status_ = planner_status;
    return status;
  }
  planner_status.global_plan_succeeded = true;
  path->stamp = current_pose.stamp;
  if (!path->points.empty()) {
    path->points.front().position.x = current_pose.position.x;
    path->points.front().position.y = current_pose.position.y;
  }

  DwaScore dwa_score;
  status = omni_dwa_.Plan(current_pose, goal, *path, costmap, obstacles,
                          latest_cmd_.ReadSnapshot(), cmd, &dwa_score,
                          planner_status.clearance_weight_scale);
  if (!status.ok()) {
    planner_status.local_plan_succeeded = false;
    planner_status.path_available = !path->points.empty();
    planner_status.failure_reason = status.message;
    planner_status.degraded_mode = "local_plan_failed";
    planner_status.planning_latency_ns = common::NowNs() - begin_ns;
    latest_status_.Publish(planner_status);
    LogPlannerStatusIfChanged(last_logged_status_, planner_status);
    last_logged_status_ = planner_status;
    return status;
  }
  planner_status.local_plan_succeeded = true;
  if (!goal.reached && IsNearZero(*cmd)) {
    ApplyPathFollowerFallback(config_, current_pose, goal, *path, cmd);
    planner_status.fallback_cmd_used = true;
    planner_status.degraded_mode = "path_follower_fallback";
  } else if (planner_status.temporary_goal_active ||
             planner_status.clearance_weight_scale > 1.01F) {
    planner_status.degraded_mode = "recovery_planner_bias";
  }
  cmd->stamp = current_pose.stamp;

  planner_status.path_available = !path->points.empty();
  planner_status.dwa_score = dwa_score;
  planner_status.planning_latency_ns = common::NowNs() - begin_ns;
  latest_status_.Publish(planner_status);
  LogPlannerStatusIfChanged(last_logged_status_, planner_status);
  last_logged_status_ = planner_status;
  return common::Status::Ok();
}

common::Status PlannerCoordinator::PlanAndPublish(
    const data::Pose3f& current_pose, const data::GridMap2D& costmap,
    const std::vector<data::DynamicObstacle>& obstacles,
    const PlanningOverrides* overrides) {
  return PlanAndPublishToGoal(current_pose, {}, costmap, obstacles, overrides);
}

common::Status PlannerCoordinator::PlanAndPublishToGoal(
    const data::Pose3f& current_pose, const data::Pose3f& goal_pose,
    const data::GridMap2D& costmap, const std::vector<data::DynamicObstacle>& obstacles,
    const PlanningOverrides* overrides) {
  data::Path2D path;
  data::ChassisCmd cmd;
  const auto status = PlanToGoal(current_pose, goal_pose, costmap, obstacles, &path, &cmd,
                                 overrides);
  if (!status.ok()) {
    return status;
  }
  global_path_.Publish(std::move(path));
  latest_cmd_.Publish(cmd);
  return common::Status::Ok();
}

}  // namespace rm_nav::planning
