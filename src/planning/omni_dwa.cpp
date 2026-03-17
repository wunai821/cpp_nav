#include "rm_nav/planning/omni_dwa.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

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

float DesiredHeading(const GoalState& goal, float x, float y) {
  if (goal.mode == GoalMode::kCenterHold) {
    return goal.target_pose.rpy.z;
  }
  return std::atan2(goal.target_pose.position.y - y, goal.target_pose.position.x - x);
}

float DistanceToPath(const data::Path2D& path, float x, float y) {
  float best = std::numeric_limits<float>::max();
  const std::size_t begin_index = path.points.size() > 1U ? 1U : 0U;
  for (std::size_t index = begin_index; index < path.points.size(); ++index) {
    const auto& point = path.points[index];
    const float dx = point.position.x - x;
    const float dy = point.position.y - y;
    best = std::min(best, std::sqrt(dx * dx + dy * dy));
  }
  return best == std::numeric_limits<float>::max() ? 0.0F : best;
}

bool InCollision(const data::GridMap2D& costmap, float local_x, float local_y) {
  if (costmap.width == 0U || costmap.height == 0U || costmap.occupancy.empty()) {
    return false;
  }
  const int center_x = static_cast<int>(costmap.width / 2U);
  const int center_y = static_cast<int>(costmap.height / 2U);
  const int gx = center_x + static_cast<int>(std::round(local_x / costmap.resolution_m));
  const int gy = center_y + static_cast<int>(std::round(local_y / costmap.resolution_m));
  if (gx < 0 || gy < 0 || gx >= static_cast<int>(costmap.width) ||
      gy >= static_cast<int>(costmap.height)) {
    return true;
  }
  return costmap.occupancy[static_cast<std::size_t>(gy) * costmap.width +
                           static_cast<std::size_t>(gx)] >= 50U;
}

float DistanceToObstacleSet(const std::vector<data::DynamicObstacle>& obstacles, float world_x,
                            float world_y) {
  float best = std::numeric_limits<float>::max();
  for (const auto& obstacle : obstacles) {
    const float dx = obstacle.pose.position.x - world_x;
    const float dy = obstacle.pose.position.y - world_y;
    best = std::min(best, std::sqrt(dx * dx + dy * dy) - obstacle.radius_m);
  }
  return best == std::numeric_limits<float>::max() ? 10.0F : best;
}

data::Pose3f PredictObstaclePose(const data::DynamicObstacle& obstacle, float t_s) {
  if (t_s <= 0.0F) {
    return obstacle.pose;
  }
  if (t_s <= 0.5F) {
    const float alpha = t_s / 0.5F;
    data::Pose3f pose = obstacle.pose;
    pose.position.x += alpha * (obstacle.predicted_pose_05s.position.x - obstacle.pose.position.x);
    pose.position.y += alpha * (obstacle.predicted_pose_05s.position.y - obstacle.pose.position.y);
    return pose;
  }
  if (t_s <= 1.0F) {
    const float alpha = (t_s - 0.5F) / 0.5F;
    data::Pose3f pose = obstacle.predicted_pose_05s;
    pose.position.x +=
        alpha * (obstacle.predicted_pose_10s.position.x - obstacle.predicted_pose_05s.position.x);
    pose.position.y +=
        alpha * (obstacle.predicted_pose_10s.position.y - obstacle.predicted_pose_05s.position.y);
    return pose;
  }

  data::Pose3f pose = obstacle.predicted_pose_10s;
  const float extra_t = t_s - 1.0F;
  pose.position.x += obstacle.velocity.x * extra_t;
  pose.position.y += obstacle.velocity.y * extra_t;
  return pose;
}

float DynamicClearanceAtTime(const std::vector<data::DynamicObstacle>& obstacles, float world_x,
                             float world_y, float t_s, float inflation_m) {
  float best_clearance = std::numeric_limits<float>::max();
  for (const auto& obstacle : obstacles) {
    const auto predicted_pose = PredictObstaclePose(obstacle, t_s);
    const float dx = predicted_pose.position.x - world_x;
    const float dy = predicted_pose.position.y - world_y;
    const float clearance =
        std::sqrt(dx * dx + dy * dy) - (obstacle.radius_m + inflation_m);
    best_clearance = std::min(best_clearance, clearance);
  }
  return best_clearance == std::numeric_limits<float>::max() ? 10.0F : best_clearance;
}

float DynamicRiskAtTime(const std::vector<data::DynamicObstacle>& obstacles, float world_x,
                        float world_y, float robot_world_vx, float robot_world_vy, float t_s,
                        const config::PlannerConfig& config) {
  if (obstacles.empty()) {
    return 0.0F;
  }

  const float influence_distance =
      std::max(static_cast<float>(config.dynamic_influence_distance_m), 0.2F);
  const float emergency_distance =
      std::clamp(static_cast<float>(config.dynamic_emergency_distance_m), 0.0F,
                 influence_distance - 0.05F);
  const float inflation_m = std::max(static_cast<float>(config.dynamic_inflation_m), 0.0F);

  float total_risk = 0.0F;
  for (const auto& obstacle : obstacles) {
    const auto predicted_pose = PredictObstaclePose(obstacle, t_s);
    const float dx = predicted_pose.position.x - world_x;
    const float dy = predicted_pose.position.y - world_y;
    const float distance = std::sqrt(dx * dx + dy * dy);
    const float clearance = distance - (obstacle.radius_m + inflation_m);
    if (clearance <= 0.0F) {
      return 10.0F;
    }
    if (clearance >= influence_distance) {
      continue;
    }

    const float confidence =
        std::clamp(obstacle.is_confirmed ? std::max(obstacle.confidence, 0.6F)
                                         : std::max(obstacle.confidence, 0.25F),
                   0.0F, 1.0F);
    const float unit_x = distance > 1.0e-4F ? dx / distance : 0.0F;
    const float unit_y = distance > 1.0e-4F ? dy / distance : 0.0F;
    const float rel_vx = obstacle.velocity.x - robot_world_vx;
    const float rel_vy = obstacle.velocity.y - robot_world_vy;
    const float closing_speed = std::max(0.0F, -(rel_vx * unit_x + rel_vy * unit_y));

    float risk = 0.0F;
    if (clearance <= emergency_distance) {
      const float denom = std::max(emergency_distance, 0.05F);
      risk = 2.0F + (emergency_distance - clearance) / denom;
    } else {
      const float denom = std::max(influence_distance - emergency_distance, 0.05F);
      const float ratio = (influence_distance - clearance) / denom;
      risk = ratio * ratio;
    }
    risk *= confidence * (1.0F + 0.5F * closing_speed);
    total_risk += risk;
  }
  return std::min(total_risk, 10.0F);
}

float SampleValue(int sample_index, int total_samples, float max_abs) {
  if (total_samples <= 1) {
    return 0.0F;
  }
  const float alpha =
      static_cast<float>(sample_index) / static_cast<float>(total_samples - 1);
  return -max_abs + 2.0F * max_abs * alpha;
}

}  // namespace

common::Status OmniDwa::Plan(const data::Pose3f& current_pose, const GoalState& goal,
                             const data::Path2D& global_path,
                             const data::GridMap2D& local_costmap,
                             const std::vector<data::DynamicObstacle>& obstacles,
                             const data::ChassisCmd& previous_cmd, data::ChassisCmd* cmd,
                             DwaScore* score) const {
  if (cmd == nullptr || score == nullptr) {
    return common::Status::InvalidArgument("dwa output is null");
  }

  const float max_vx = static_cast<float>(
      goal.mode == GoalMode::kCenterHold ? config_.hold_max_v_mps : config_.max_vx_mps);
  const float max_vy = static_cast<float>(
      goal.mode == GoalMode::kCenterHold ? config_.hold_max_v_mps : config_.max_vy_mps);
  const float max_wz = static_cast<float>(
      goal.mode == GoalMode::kCenterHold ? config_.hold_max_wz_radps : config_.max_wz_radps);

  data::ChassisCmd best_cmd;
  best_cmd.brake = true;
  DwaScore best_score;

  for (int ix = 0; ix < config_.dwa_linear_samples; ++ix) {
    const float vx = SampleValue(ix, config_.dwa_linear_samples, max_vx);
    for (int iy = 0; iy < config_.dwa_lateral_samples; ++iy) {
      const float vy = SampleValue(iy, config_.dwa_lateral_samples, max_vy);
      for (int iw = 0; iw < config_.dwa_yaw_samples; ++iw) {
        const float wz = SampleValue(iw, config_.dwa_yaw_samples, max_wz);

        float local_x = 0.0F;
        float local_y = 0.0F;
        float local_yaw = current_pose.rpy.z;
        bool collision = false;
        float integrated_dynamic_risk = 0.0F;
        float max_dynamic_risk = 0.0F;
        float min_dynamic_clearance = std::numeric_limits<float>::max();
        for (double t = config_.dwa_dt_s; t <= config_.dwa_horizon_s + 1.0e-6;
             t += config_.dwa_dt_s) {
          local_x += (vx * std::cos(local_yaw) - vy * std::sin(local_yaw)) *
                     static_cast<float>(config_.dwa_dt_s);
          local_y += (vx * std::sin(local_yaw) + vy * std::cos(local_yaw)) *
                     static_cast<float>(config_.dwa_dt_s);
          local_yaw = NormalizeAngle(local_yaw + wz * static_cast<float>(config_.dwa_dt_s));
          if (InCollision(local_costmap, local_x, local_y)) {
            collision = true;
            break;
          }
          const float future_world_x = current_pose.position.x + local_x;
          const float future_world_y = current_pose.position.y + local_y;
          const float robot_world_vx = vx * std::cos(local_yaw) - vy * std::sin(local_yaw);
          const float robot_world_vy = vx * std::sin(local_yaw) + vy * std::cos(local_yaw);
          const float risk_here = DynamicRiskAtTime(
              obstacles, future_world_x, future_world_y, robot_world_vx, robot_world_vy,
              static_cast<float>(t), config_);
          const float dynamic_clearance = DynamicClearanceAtTime(
              obstacles, future_world_x, future_world_y, static_cast<float>(t),
              static_cast<float>(config_.dynamic_inflation_m));
          integrated_dynamic_risk += risk_here * static_cast<float>(config_.dwa_dt_s);
          max_dynamic_risk = std::max(max_dynamic_risk, risk_here);
          min_dynamic_clearance = std::min(min_dynamic_clearance, dynamic_clearance);
          if (risk_here >= 10.0F || dynamic_clearance <= 0.0F) {
            collision = true;
            break;
          }
        }
        if (collision) {
          continue;
        }

        const float end_x = current_pose.position.x + local_x;
        const float end_y = current_pose.position.y + local_y;
        const float dx_goal = goal.target_pose.position.x - end_x;
        const float dy_goal = goal.target_pose.position.y - end_y;
        const float dist_to_goal = std::sqrt(dx_goal * dx_goal + dy_goal * dy_goal);
        const float progress = std::max(0.0F, goal.distance_to_target_m - dist_to_goal);
        const float obstacle_clearance =
            DistanceToObstacleSet(obstacles, end_x, end_y);
        const float path_dist = DistanceToPath(global_path, end_x, end_y);
        const float smooth_delta = std::fabs(vx - previous_cmd.vx_mps) +
                                   std::fabs(vy - previous_cmd.vy_mps) +
                                   0.5F * std::fabs(wz - previous_cmd.wz_radps);
        const float linear_speed = std::sqrt(vx * vx + vy * vy);
        const float heading_error = std::fabs(
            NormalizeAngle(DesiredHeading(goal, end_x, end_y) - local_yaw));
        const float dynamic_risk_05 = DynamicRiskAtTime(
            obstacles, end_x, end_y, vx * std::cos(local_yaw) - vy * std::sin(local_yaw),
            vx * std::sin(local_yaw) + vy * std::cos(local_yaw), 0.5F, config_);
        const float dynamic_risk_10 = DynamicRiskAtTime(
            obstacles, end_x, end_y, vx * std::cos(local_yaw) - vy * std::sin(local_yaw),
            vx * std::sin(local_yaw) + vy * std::cos(local_yaw), 1.0F, config_);
        const float dynamic_penalty =
            0.45F * max_dynamic_risk + 0.25F * integrated_dynamic_risk +
            0.20F * dynamic_risk_05 + 0.10F * dynamic_risk_10;

        DwaScore candidate_score;
        candidate_score.goal_score = 10.0F * progress;
        candidate_score.path_score = 0.8F / (0.2F + path_dist);
        candidate_score.smooth_score = 0.4F / (0.2F + smooth_delta);
        candidate_score.heading_score = static_cast<float>(config_.weight_heading) *
                                        (0.8F / (0.2F + heading_error));
        candidate_score.clearance_score = static_cast<float>(config_.weight_clearance) *
                                          std::min(1.0F, obstacle_clearance);
        candidate_score.velocity_score = static_cast<float>(config_.weight_velocity) *
                                         linear_speed;
        candidate_score.dynamic_risk_score =
            -static_cast<float>(config_.weight_dynamic) * dynamic_penalty;
        candidate_score.dynamic_max_risk = max_dynamic_risk;
        candidate_score.dynamic_integrated_risk = integrated_dynamic_risk;
        candidate_score.dynamic_clearance_min =
            min_dynamic_clearance == std::numeric_limits<float>::max() ? 10.0F
                                                                       : min_dynamic_clearance;
        candidate_score.dynamic_risk_05 = dynamic_risk_05;
        candidate_score.dynamic_risk_10 = dynamic_risk_10;
        candidate_score.total_score = candidate_score.goal_score +
                                      candidate_score.path_score +
                                      candidate_score.smooth_score +
                                      candidate_score.heading_score +
                                      candidate_score.clearance_score +
                                      candidate_score.velocity_score +
                                      candidate_score.dynamic_risk_score +
                                      (goal.mode == GoalMode::kCenterHold ? 0.3F : 0.0F);
        if (!goal.reached && goal.distance_to_target_m > 0.5F && linear_speed < 0.05F) {
          candidate_score.total_score -= 6.0F;
        }

        if (candidate_score.total_score > best_score.total_score) {
          best_score = candidate_score;
          best_cmd.stamp = current_pose.stamp;
          best_cmd.vx_mps = vx;
          best_cmd.vy_mps = vy;
          best_cmd.wz_radps = wz;
          best_cmd.brake = false;
        }
      }
    }
  }

  if (goal.reached) {
    best_cmd = {};
    best_cmd.stamp = current_pose.stamp;
    best_cmd.brake = true;
    best_score.total_score = 0.0F;
  }

  *cmd = best_cmd;
  *score = best_score;
  return common::Status::Ok();
}

}  // namespace rm_nav::planning
